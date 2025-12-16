from typing import List, Tuple, Dict, Set
import mip
import networkx as nx
import numpy as np
from utils import Point3D, calc_time_between_points


class DroneRoutingSolver:
    """
    Solver implementing mTSP with optional drones and NO revisits (simple cycles).
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

        # Identify entry points and add edges to base (node 0)
        y_coords = coords[:, 1]
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

        # Populate arcs and costs for the MIP solver
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

    def _generate_no_revisit_greedy_solution(self):
        """
        Generates a feasible solution where drones never revisit a node (simple cycles).
        Uses a parallel nearest neighbor heuristic with minimax objective.
        """
        if self.verbose:
            print("Generating no-revisit greedy initial solution...")

        routes = {k: [0] for k in range(self.k_drones)}
        drone_times = {k: 0.0 for k in range(self.k_drones)}
        unvisited = set(range(1, self.num_nodes))

        # Precompute neighbors for fast lookup (only outgoing edges needed)
        neighbors = {u: [] for u in range(self.num_nodes)}
        for (u, v), cost in self.costs.items():
            neighbors[u].append((v, cost))

        while unvisited:
            best_move = None
            best_new_makespan = float("inf")
            current_max_time = max(drone_times.values())
            candidate_found = False

            for k in range(self.k_drones):
                u = routes[k][-1]
                # Only consider moves that do not revisit nodes
                for v, cost in neighbors[u]:
                    if v in unvisited:
                        new_time = drone_times[k] + cost
                        new_makespan = max(new_time, current_max_time)

                        if new_makespan < best_new_makespan:
                            best_new_makespan = new_makespan
                            best_move = (k, v, cost)
                            candidate_found = True
                        elif new_makespan == best_new_makespan:
                            if best_move and cost < best_move[2]:
                                best_move = (k, v, cost)
                                candidate_found = True

            if not candidate_found:
                if self.verbose:
                    print(
                        "No-revisit heuristic stuck: cannot reach any unvisited node."
                    )
                return None

            k, v, cost = best_move
            routes[k].append(v)
            drone_times[k] += cost
            unvisited.remove(v)

        # Return to base (must be possible from last node; otherwise fail)
        for k in range(self.k_drones):
            u = routes[k][-1]
            if u != 0:
                if (u, 0) in self.costs:
                    cost = self.costs[(u, 0)]
                    routes[k].append(0)
                    drone_times[k] += cost
                else:
                    if self.verbose:
                        print(
                            f"No-revisit heuristic failed: Drone {k} cannot return to base from {u}."
                        )
                    return None

        # Sort drones by route length for symmetry breaking
        drone_sizes = [(k, len(routes[k])) for k in range(self.k_drones)]
        drone_sizes.sort(key=lambda x: x[1], reverse=True)
        new_to_old = {new_k: old_k for new_k, (old_k, _) in enumerate(drone_sizes)}

        x_sol = {}
        y_sol = {}
        z_sol = {}

        for new_k in range(self.k_drones):
            old_k = new_to_old[new_k]
            route = routes[old_k]

            is_active = len(route) > 2  # has at least one target
            z_sol[new_k] = 1.0 if is_active else 0.0

            for node in route:
                if node != 0:
                    y_sol[(new_k, node)] = 1.0

            for i in range(len(route) - 1):
                u, v = route[i], route[i + 1]
                x_sol[(new_k, u, v)] = 1.0  # no revisits -> at most one traversal

        greedy_makespan = max(drone_times.values())
        return x_sol, y_sol, z_sol, greedy_makespan

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

    def solve(
        self, max_seconds: int = 300, mip_gap: float = 0.02, warm_start: bool = True
    ):
        """
        Builds and solves the MIP model for the mTSP variant:
        - Optional drones
        - No revisits of target nodes (binary x, degree = 1 in/out for owned nodes)
        - Minimize makespan
        """

        model = mip.Model(sense=mip.MINIMIZE, solver_name=mip.CBC)
        model.max_mip_gap = mip_gap
        model.threads = -1
        model.verbose = self.verbose
        model.cuts = 2  # Aggressive cut generation

        # Sets
        K = range(self.k_drones)
        V = range(self.num_nodes)
        P = [i for i in V if i != 0]  # Target points

        # Precompute in-edges for faster constraint building
        in_edges = {i: [] for i in V}
        for u, v in self.arcs:
            in_edges[v].append(u)

        # Variables
        # x[k, i, j]: Binary arc usage (no revisits)
        x = {}
        for k in K:
            for i, j in self.arcs:
                x[(k, i, j)] = model.add_var(var_type=mip.BINARY, name=f"x_{k}_{i}_{j}")

        # y[k, j]: 1 if node j is owned by drone k
        y = {}
        for k in K:
            for j in P:
                y[(k, j)] = model.add_var(var_type=mip.BINARY, name=f"y_{k}_{j}")

        # T: makespan
        T = model.add_var(var_type=mip.CONTINUOUS, lb=0.0, name="T")  # type: ignore

        # f[k, i, j]: flow variables for connectivity
        f = {}
        for k in K:
            for i, j in self.arcs:
                f[(k, i, j)] = model.add_var(
                    var_type=mip.CONTINUOUS, lb=0.0, name=f"f_{k}_{i}_{j}"  # type: ignore
                )

        # Objective: pure minimax
        model.objective = T

        # Constraints

        # 1. Exclusive assignment
        for j in P:
            model.add_constr(mip.xsum(y[(k, j)] for k in K) == 1, name=f"assign_{j}")

        # 2. Degree constraints (no revisits): exactly one in and one out if owned; none otherwise
        for k in K:
            for j in P:
                outgoing = self.out_edges[j] if j in self.out_edges else []
                model.add_constr(
                    mip.xsum(x[(k, i, j)] for i in in_edges[j]) == y[(k, j)],
                    name=f"in_degree_{k}_{j}",
                )
                model.add_constr(
                    mip.xsum(x[(k, j, m)] for m in outgoing) == y[(k, j)],
                    name=f"out_degree_{k}_{j}",
                )

        # 3. Base Station Constraints (All drones used)
        for k in K:
            base_outgoing = self.out_edges[0]
            model.add_constr(
                mip.xsum(x[(k, 0, j)] for j in base_outgoing) == 1,
                name=f"base_out_{k}",
            )

            base_incoming = in_edges[0]
            model.add_constr(
                mip.xsum(x[(k, i, 0)] for i in base_incoming) == 1,
                name=f"base_in_{k}",
            )

        # 4. Subtour Elimination (Single-Commodity Flow)
        for k in K:
            base_outgoing = self.out_edges[0]
            model.add_constr(
                mip.xsum(f[(k, 0, j)] for j in base_outgoing)
                == mip.xsum(y[(k, p)] for p in P),
                name=f"flow_supply_{k}",
            )

            for v in P:
                outgoing = self.out_edges[v] if v in self.out_edges else []
                model.add_constr(
                    mip.xsum(f[(k, i, v)] for i in in_edges[v])
                    - mip.xsum(f[(k, v, j)] for j in outgoing)
                    == y[(k, v)],
                    name=f"flow_conservation_{k}_{v}",
                )

            for i, j in self.arcs:
                model.add_constr(
                    f[(k, i, j)] <= len(P) * x[(k, i, j)],
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
        for k in range(self.k_drones - 1):
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) >= mip.xsum(y[(k + 1, j)] for j in P),
                name=f"symmetry_size_{k}",
            )

        # 7. Minimum workload (All drones used)
        for k in K:
            # Each drone must visit at least one target node
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) >= 1,
                name=f"activation_lower_{k}",
            )

        # 6.5 Valid Inequality: Lower Bound on Z
        total_time = mip.xsum(
            self.costs[(i, j)] * x[(k, i, j)] for k in K for (i, j) in self.arcs
        )
        model += T >= total_time / self.k_drones  # type: ignore

        # Warm Start
        if warm_start:
            try:
                greedy_result = self._generate_no_revisit_greedy_solution()
                if greedy_result:
                    x_sol, y_sol, z_sol, greedy_makespan = greedy_result
                    start_list = []

                    for (k, u, v), val in x_sol.items():
                        if (k, u, v) in x:
                            start_list.append((x[(k, u, v)], val))

                    for (k, j), val in y_sol.items():
                        if (k, j) in y:
                            start_list.append((y[(k, j)], val))

                    model.start = start_list
                    T.ub = greedy_makespan  # type: ignore

                    if self.verbose:
                        print(
                            f"Warm start solution provided with {len(start_list)} variables. UB set to {greedy_makespan:.2f}"
                        )
            except Exception as e:
                if self.verbose:
                    print(f"Failed to generate warm start solution: {e}")

        max_min_trip = self._get_makespan_lower_bound()
        if self.verbose:
            print(f"Setting lower bound for makespan T to {max_min_trip:.2f}")
        T.lb = max_min_trip  # type: ignore

        # Solve
        status = model.optimize(max_seconds=float(max_seconds)) # type: ignore

        if (
            status == mip.OptimizationStatus.OPTIMAL
            or status == mip.OptimizationStatus.FEASIBLE
        ):
            if self.verbose:
                print(f"Solution found! Objective: {model.objective_value}")
            return self._extract_solution(x)
        else:
            if self.verbose:
                print("No solution found.")
            return []

    def _get_makespan_lower_bound(self) -> float:
        """
        Computes a lower bound for the makespan T based on shortest round trips.
        T >= max(shortest_round_trip(0 -> j -> 0)) for all j
        """
        try:
            dists_from_base = nx.shortest_path_length(
                self.graph, source=0, weight="weight"
            )
            dists_to_base = nx.shortest_path_length(
                self.graph.reverse(), source=0, weight="weight"
            )

            max_min_trip = 0.0
            for j in range(1, self.num_nodes):
                if j in dists_from_base and j in dists_to_base:
                    trip = dists_from_base[j] + dists_to_base[j]
                    max_min_trip = max(max_min_trip, trip)

            return max_min_trip
        except Exception:
            print("An error occurred: Could not compute lower bound for makespan T.")
            return 0.0

    def _extract_solution(self, x):
        paths = []
        for k in range(self.k_drones):
            # Build adjacency (each arc used at most once)
            adj = {}
            for (d, u, v), var in x.items():
                if d == k and var.x is not None and var.x > 0.5:
                    adj.setdefault(u, []).append(v)

            if 0 not in adj or not adj[0]:
                path = [0, 0]
                paths.append(path)
                if self.verbose:
                    print(f"Drone {k+1}: {'-'.join(map(str, path))}")
                continue

            # Simple path reconstruction since degrees are 1 (Eulerian but without repeats)
            path = [0]
            current = 0
            while current in adj and adj[current]:
                nxt = adj[current].pop()
                path.append(nxt)
                current = nxt
                if current == 0:
                    break

            if current != 0:
                path.append(0)  # safety fallback

            if self.verbose:
                print(f"Drone {k+1}: {'-'.join(map(str, path))}")
            paths.append(path)

        return paths
