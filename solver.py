from typing import List, Tuple, Dict, Set
import mip
import networkx as nx
import numpy as np
from utils import Point3D, calc_time_between_points


class DroneRoutingSolver:
    """
    Solver
    """

    def __init__(
        self,
        points: List[Point3D],
        base_point: Point3D,
        entry_threshold: float,
        k_drones: int = 4,
        speed_up: float = 1.0,
        speed_down: float = 2.0,
        speed_horizontal: float = 1.5,
        verbose: bool = False,
    ):
        self.points = [base_point] + points
        self.entry_threshold = entry_threshold
        self.k_drones = k_drones
        self.verbose = verbose
        # Constants from problem description
        self.speed_up = speed_up
        self.speed_down = speed_down
        self.speed_horizontal = speed_horizontal

        self.num_nodes = len(self.points)

        # Precompute valid arcs and costs
        self.arcs: Set[Tuple[int, int]] = set()
        self.entry_points_idx: Set[int] = set()
        self.costs: Dict[Tuple[int, int], float] = {}
        self.graph = nx.DiGraph()
        self._build_graph()

    def _build_graph(self):
        """Builds the graph nodes, arcs and calculates travel times using vectorized operations."""
        if self.verbose:
            print("Building graph with NetworkX (Directed)...")

        self.graph.add_nodes_from(range(self.num_nodes))

        # Convert points to numpy array for vectorized distance calculation
        coords = np.array([[p.x, p.y, p.z] for p in self.points])

        # Calculate all pairwise differences
        # shape: (num_nodes, num_nodes, 3)
        diffs = coords[:, np.newaxis, :] - coords[np.newaxis, :, :]

        # Euclidean distances
        # shape: (num_nodes, num_nodes)
        dists = np.sqrt(np.sum(diffs**2, axis=-1))

        # Condition 1: Euclidean distance <= 4.0
        cond1 = dists <= 4.0

        # Condition 2: Euclidean distance <= 11.0 AND at least 2 coordinates differ by <= 0.5
        abs_diffs = np.abs(diffs)
        small_diffs_count = np.sum(abs_diffs <= 0.5, axis=2)
        cond2 = (dists <= 11.0) & (small_diffs_count >= 2)

        # Combined connectivity (symmetric)
        is_connected = cond1 | cond2

        # We only care about i < j for the loop, but we need both directions
        # Get indices where is_connected is True and i < j
        # np.triu ensures i <= j, k=1 ensures i < j
        rows, cols = np.where(np.triu(is_connected, k=1))

        if self.verbose:
            print(f"Found {len(rows)} connected pairs.")

        # Add edges
        for i, j in zip(rows, cols):
            if i == 0 or j == 0:
                continue
            p_i = self.points[i]
            p_j = self.points[j]

            # Calculate costs for both directions
            cost_ij = calc_time_between_points(
                p_i, p_j, self.speed_horizontal, self.speed_up, self.speed_down
            )
            cost_ji = calc_time_between_points(
                p_j, p_i, self.speed_horizontal, self.speed_up, self.speed_down
            )
            self.graph.add_edge(int(i), int(j), weight=cost_ij)
            self.graph.add_edge(int(j), int(i), weight=cost_ji)

        # 2. Identify entry points and add edges to base (node 0)
        # Vectorized check for entry points
        y_coords = coords[:, 1]
        # indices where y <= threshold and i > 0
        entry_indices = np.where(
            (y_coords <= self.entry_threshold) & (np.arange(self.num_nodes) > 0)
        )[0]

        for i in entry_indices:
            self.entry_points_idx.add(int(i))
            p_0 = self.points[0]
            p_i = self.points[i]

            cost_0i = calc_time_between_points(
                p_0, p_i, self.speed_horizontal, self.speed_up, self.speed_down
            )
            cost_i0 = calc_time_between_points(
                p_i, p_0, self.speed_horizontal, self.speed_up, self.speed_down
            )

            self.graph.add_edge(0, int(i), weight=cost_0i)
            self.graph.add_edge(int(i), 0, weight=cost_i0)

        if self.verbose:
            print(
                f"NetworkX Graph: {self.graph.number_of_nodes()} nodes, {self.graph.number_of_edges()} edges"
            )

        # 3. Populate arcs and costs for the MIP solver
        self.out_edges = {i: [] for i in range(self.num_nodes)}
        for u, v, data in self.graph.edges(data=True):
            self.arcs.add((u, v))
            self.costs[(u, v)] = data["weight"]
            self.out_edges[u].append(v)

        if not self.entry_points_idx:
            raise ValueError(
                "No entry points satisfy the threshold; base cannot connect to the grid."
            )

        if self.verbose:
            print(f"Total directed arcs for MIP: {len(self.arcs)}")

    def get_graph(self):
        """
        Returns the graph representation: nodes, arcs, and costs.

        Returns:
            Tuple containing:
                - List of points (nodes)
                - Set of arcs (tuples of node indices)
                - Dictionary of costs for each arc
                - Set of entry point indices
        """
        return self.points, self.arcs, self.costs, self.entry_points_idx

    def solve(self, max_seconds: int = 300, mip_gap: float = 0.02):
        """
        Builds and solves the MIP model for the Drone Routing Problem.

        Args:
            max_seconds (int): Maximum time allowed for the solver in seconds.
            mip_gap (float): Relative MIP gap tolerance (trade optimality for speed).
        """

        model = mip.Model(sense=mip.MINIMIZE, solver_name=mip.CBC)
        model.max_mip_gap = mip_gap
        model.threads = -1  # Use all available threads for performance
        model.verbose = self.verbose

        # Sets
        K = range(self.k_drones)
        V = range(self.num_nodes)
        P = [i for i in V if i != 0]  # Target points

        # Precompute in-edges for faster constraint building
        in_edges = {i: [] for i in V}
        for u, v in self.arcs:
            in_edges[v].append(u)

        # Variables
        # x[k, i, j]: 1 if drone k flies arc (i, j), 0 otherwise
        x = {}
        for k in K:
            for i, j in self.arcs:
                x[(k, i, j)] = model.add_var(var_type=mip.BINARY, name=f"x_{k}_{i}_{j}")

        # y[k, j]: 1 if grid node j is owned by drone k
        y = {}
        for k in K:
            for j in P:
                y[(k, j)] = model.add_var(var_type=mip.BINARY, name=f"y_{k}_{j}")

        # T: makespan
        T = model.add_var(var_type=mip.CONTINUOUS, lb=0.0, name="T")

        # z[k]: 1 if drone k is used
        z = {k: model.add_var(var_type=mip.BINARY, name=f"z_{k}") for k in K}

        # f[k, i, j]: flow variables for connectivity
        f = {}
        for k in K:
            for i, j in self.arcs:
                f[(k, i, j)] = model.add_var(
                    var_type=mip.CONTINUOUS, lb=0.0, name=f"f_{k}_{i}_{j}"
                )

        # Objective
        # Minimize makespan + small penalty on total travel time (lexicographic)
        max_cost = max(self.costs.values()) if self.costs else 1.0
        epsilon = 1e-6 * max_cost
        model.objective = T + mip.xsum(
            (epsilon * self.costs[(i, j)]) * x[(k, i, j)]
            for k in K
            for (i, j) in self.arcs
        )

        # Constraints

        # 1. Exclusive assignment
        for j in P:
            model.add_constr(mip.xsum(y[(k, j)] for k in K) == 1, name=f"assign_{j}")

        # 2. Visit exactly once if owned
        for k in K:
            for j in P:
                # Incoming == Owned
                model.add_constr(
                    mip.xsum(x[(k, i, j)] for i in in_edges[j]) == y[(k, j)],
                    name=f"visit_in_{k}_{j}",
                )

                # Outgoing == Owned
                outgoing = self.out_edges[j]
                model.add_constr(
                    mip.xsum(x[(k, j, i)] for i in outgoing) == y[(k, j)],
                    name=f"visit_out_{k}_{j}",
                )

        # 3. Base Station Constraints
        for k in K:
            # Depart from base exactly once
            base_outgoing = self.out_edges[0]
            model.add_constr(
                mip.xsum(x[(k, 0, j)] for j in base_outgoing) == z[k],
                name=f"base_out_{k}",
            )

            # Return to base exactly once
            base_incoming = in_edges[0]
            model.add_constr(
                mip.xsum(x[(k, i, 0)] for i in base_incoming) == z[k],
                name=f"base_in_{k}",
            )

        # 4. Subtour Elimination (Single-Commodity Flow)
        for k in K:
            # Supply at base = number of owned nodes
            base_outgoing = self.out_edges[0]
            model.add_constr(
                mip.xsum(f[(k, 0, j)] for j in base_outgoing)
                == mip.xsum(y[(k, p)] for p in P),
                name=f"flow_supply_{k}",
            )

            # Flow conservation on owned nodes: In - Out = Owned
            for v in P:
                outgoing = self.out_edges[v]
                model.add_constr(
                    mip.xsum(f[(k, i, v)] for i in in_edges[v])
                    - mip.xsum(f[(k, v, j)] for j in outgoing)
                    == y[(k, v)],
                    name=f"flow_conservation_{k}_{v}",
                )

            # Capacity linking
            for i, j in self.arcs:
                model.add_constr(
                    f[(k, i, j)] <= (len(P) - 1) * x[(k, i, j)],
                    name=f"flow_capacity_{k}_{i}_{j}",
                )

        # 5. Makespan linking
        for k in K:
            model.add_constr(
                T
                >= mip.xsum(self.costs[(i, j)] * x[(k, i, j)] for (i, j) in self.arcs),
                name=f"makespan_{k}",
            )

        # 6. Symmetry Breaking
        # Order drones by number of visited nodes: |P_k| >= |P_{k+1}|
        for k in range(self.k_drones - 1):
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) >= mip.xsum(y[(k + 1, j)] for j in P),
                name=f"symmetry_size_{k}",
            )

        # Activation linking: a drone must be active to own nodes
        big_m_nodes = len(P)
        for k in K:
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) <= big_m_nodes * z[k],
                name=f"activation_{k}",
            )
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) >= z[k],
                name=f"activation_lower_{k}",
            )

        # Solve
        status = model.optimize(max_seconds=max_seconds)

        if (
            status == mip.OptimizationStatus.OPTIMAL
            or status == mip.OptimizationStatus.FEASIBLE
        ):
            if self.verbose:
                print(f"Solution found! Objective: {model.objective_value}")
            return self._extract_solution(x, z)
        else:
            if self.verbose:
                print("No solution found.")
            return []

    def _extract_solution(self, x, z):
        paths = []
        for k in range(self.k_drones):
            active = z[k].x is not None and z[k].x > 0.5
            # Build multigraph adjacency list for this drone
            adj = {}
            for (d, u, v), var in x.items():
                if d == k and var.x is not None and var.x > 0.5:
                    count = int(round(var.x))
                    if u not in adj:
                        adj[u] = []
                    for _ in range(count):
                        adj[u].append(v)

            if 0 not in adj or not adj[0]:
                path = [0, 0]
                if self.verbose:
                    print(f"Drone {k+1}: {'-'.join(map(str, path))}")
                paths.append(path)
                continue

            # Hierholzer's algorithm for Eulerian path/circuit
            # Since we start at 0 and must return to 0, it's a circuit.
            stack = [0]
            circuit = []

            while stack:
                u = stack[-1]
                if u in adj and adj[u]:
                    v = adj[u].pop()
                    stack.append(v)
                else:
                    circuit.append(stack.pop())

            # The circuit is built in reverse order of finishing
            path = circuit[::-1]

            if self.verbose:
                path_str = "-".join(map(str, path))
                print(f"Drone {k+1}: {path_str}")
            paths.append(path)

        return paths
