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

    def _generate_greedy_solution(self):
        """
        Generates a feasible solution using a greedy tree-expansion heuristic.
        Each drone builds a tree of visited nodes, traversing edges back and forth.
        This guarantees connectivity and respects the strict ownership constraint (no visiting others' nodes).
        """
        if self.verbose:
            print("Generating greedy initial solution...")

        # Data structures
        # owned_nodes[k] = set of nodes owned by drone k
        owned_nodes = {k: {0} for k in range(self.k_drones)}

        # edges_count[k][(u, v)] = count
        edges_count = {k: {} for k in range(self.k_drones)}

        # drone_times[k] = current total time
        drone_times = {k: 0.0 for k in range(self.k_drones)}

        unvisited = set(range(1, self.num_nodes))

        # Precompute valid neighbors for fast lookup
        # neighbors[u] = [(v, cost_uv, cost_vu), ...]
        neighbors = {u: [] for u in range(self.num_nodes)}
        for (u, v), cost in self.costs.items():
            if (v, u) in self.costs:  # Ensure bidirectional for "there and back"
                neighbors[u].append((v, cost, self.costs[(v, u)]))

        while unvisited:
            best_move = None
            best_obj = float("inf")

            # Find best node to add
            candidate_found = False

            current_max = max(drone_times.values())

            for k in range(self.k_drones):
                # Optimization: Iterate through owned nodes of all drones to find neighbors in unvisited
                for v in owned_nodes[k]:
                    # Constraint: Cannot branch from base (0) if already has a branch
                    # This ensures degree of base is exactly 2 (1 out, 1 in)
                    if v == 0 and len(owned_nodes[k]) > 1:
                        continue

                    for u, cost_vu, cost_uv in neighbors[v]:
                        if u in unvisited:
                            # Cost to add u: go v->u and u->v
                            added_cost = cost_vu + cost_uv
                            new_time = drone_times[k] + added_cost

                            # Objective: minimize the new global makespan
                            new_obj = max(new_time, current_max)

                            # Tie-breaker: prefer smaller added_cost
                            if new_obj < best_obj or (
                                new_obj == best_obj
                                and added_cost
                                < (best_move[3] if best_move else float("inf"))
                            ):
                                best_obj = new_obj
                                best_move = (k, v, u, added_cost)
                                candidate_found = True

            if not candidate_found:
                if self.verbose:
                    print(
                        "Warning: Greedy heuristic got stuck. Graph might not be connected."
                    )
                break

            # Apply best move
            k, v, u, added_cost = best_move
            owned_nodes[k].add(u)
            unvisited.remove(u)
            drone_times[k] += added_cost

            # Add edges (v, u) and (u, v)
            edges_count[k][(v, u)] = edges_count[k].get((v, u), 0) + 1
            edges_count[k][(u, v)] = edges_count[k].get((u, v), 0) + 1

        if unvisited:
            if self.verbose:
                print(
                    f"Greedy heuristic failed to visit {len(unvisited)} nodes. Skipping warm start."
                )
            return None

        # Sort drones by number of owned nodes (descending) to satisfy symmetry breaking
        # We need to sort the keys of owned_nodes and edges_count together

        # Get list of (k, num_nodes)
        drone_sizes = [(k, len(owned_nodes[k])) for k in range(self.k_drones)]
        # Sort by size desc
        drone_sizes.sort(key=lambda x: x[1], reverse=True)

        # Create mapping: new_k -> old_k
        new_to_old = {new_k: old_k for new_k, (old_k, _) in enumerate(drone_sizes)}

        # Convert to solution format
        x_sol = {}
        y_sol = {}
        z_sol = {}

        for new_k in range(self.k_drones):
            old_k = new_to_old[new_k]

            # z
            is_active = len(owned_nodes[old_k]) > 1  # Has more than just base
            z_sol[new_k] = 1.0 if is_active else 0.0

            # y
            for node in owned_nodes[old_k]:
                if node != 0:
                    y_sol[(new_k, node)] = 1.0

            # x
            for (u, v), count in edges_count[old_k].items():
                x_sol[(new_k, u, v)] = float(count)

        # Calculate makespan of the greedy solution
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
        Builds and solves the MIP model for the Drone Routing Problem.

        Args:
            max_seconds (int): Maximum time allowed for the solver in seconds.
            mip_gap (float): Relative MIP gap tolerance (trade optimality for speed).
            warm_start (bool): Whether to generate and use a greedy initial solution.
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
        # x[k, i, j]: Number of times drone k flies arc (i, j)
        x = {}
        for k in K:
            for i, j in self.arcs:
                x[(k, i, j)] = model.add_var(
                    var_type=mip.INTEGER, lb=0.0, name=f"x_{k}_{i}_{j}"
                )

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

        # Objective: pure minimax (no secondary tie-breaker)
        model.objective = T

        # Constraints

        # 1. Exclusive assignment
        for j in P:
            model.add_constr(mip.xsum(y[(k, j)] for k in K) == 1, name=f"assign_{j}")

        # 2. Tour Connectivity & Ownership (Revisits Allowed)
        M_visits = len(P)  # Sufficiently large number (increased for safety)
        for k in K:
            for j in P:
                outgoing = self.out_edges[j]

                # Flow Balance: Incoming == Outgoing
                model.add_constr(
                    mip.xsum(x[(k, i, j)] for i in in_edges[j])
                    == mip.xsum(x[(k, j, m)] for m in outgoing),
                    name=f"flow_balance_{k}_{j}",
                )

                # Service Requirement: If owned, must enter at least once
                model.add_constr(
                    y[(k, j)] <= mip.xsum(x[(k, i, j)] for i in in_edges[j]),
                    name=f"service_min_{k}_{j}",
                )

                # Exclusivity: If not owned, cannot enter
                model.add_constr(
                    mip.xsum(x[(k, i, j)] for i in in_edges[j]) <= M_visits * y[(k, j)],
                    name=f"service_max_{k}_{j}",
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
        # Order drones by number of visited nodes: |P_k| >= |P_{k+1}|
        for k in range(self.k_drones - 1):
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) >= mip.xsum(y[(k + 1, j)] for j in P),
                name=f"symmetry_size_{k}",
            )

        # Additional symmetry: enforce activation ordering z_k >= z_{k+1}
        for k in range(self.k_drones - 1):
            model.add_constr(z[k] >= z[k + 1], name=f"symmetry_active_{k}")

        # Activation linking: a drone must be active to own nodes
        big_m_nodes = float(len(P))
        for k in K:
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) <= big_m_nodes * z[k],
                name=f"activation_{k}",
            )
            model.add_constr(
                mip.xsum(y[(k, j)] for j in P) >= z[k],
                name=f"activation_lower_{k}",
            )

        # Warm Start
        if warm_start:
            try:
                greedy_result = self._generate_greedy_solution()
                if greedy_result:
                    x_sol, y_sol, z_sol, greedy_makespan = greedy_result
                    start_list = []

                    # Add x variables
                    for (k, u, v), val in x_sol.items():
                        if (k, u, v) in x:
                            start_list.append((x[(k, u, v)], val))

                    # Add y variables
                    for (k, j), val in y_sol.items():
                        if (k, j) in y:
                            start_list.append((y[(k, j)], val))

                    # Add z variables
                    for k, val in z_sol.items():
                        if k in z:
                            start_list.append((z[k], val))

                    model.start = start_list

                    # Set Upper Bound from Greedy Solution
                    # This significantly prunes the search tree
                    T.ub = greedy_makespan

                    if self.verbose:
                        print(
                            f"Warm start solution provided with {len(start_list)} variables. UB set to {greedy_makespan:.2f}"
                        )
            except Exception as e:
                if self.verbose:
                    print(f"Failed to generate warm start solution: {e}")

        # Solver Configuration for Speed
        model.cuts = 2  # Aggressive cut generation
        #model.presolve = -1  # Enable presolve

        max_min_trip = self._get_makespan_lower_bound()
        if self.verbose:
            print(f"Setting lower bound for makespan T to {max_min_trip:.2f}")
        T.lb = max_min_trip

        # Solve
        status = model.optimize(max_seconds=float(max_seconds))

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

    def _get_makespan_lower_bound(self) -> float:
        """
        Computes a lower bound for the makespan T based on shortest round trips.
        T >= max(shortest_round_trip(0 -> j -> 0)) for all j
        This helps the solver prune branches that can't possibly be optimal.
        Since node 0 is only connected to entry points, this implicitly finds
        the optimal entry and exit points for each node j.

        Here is the step-by-step reasoning:

        1. Definition of Makespan:
            T is the time taken by the slowest drone. In other words,
            T ≥ Time(Drone k) for all drones k
        2. Mandatory Visit: Every node j in the grid must be visited by some drone.
        3. Minimum Time for Node j: For a drone to visit node j, it must start at the base (0), get to j,
            and eventually return to the base (0). The absolute minimum time this takes is the shortest path from Base → j plus the shortest path from j → Base.
            MinTrip(j)=dist(0,j)+dist(j,0)
        4. The Bottleneck: Since
            T must be greater than the travel time of any drone, and some drone has to visit the "furthest" node (the one with the largest round-trip time), the makespan cannot be smaller than that largest round-trip time.

        Example:
            Node A takes at least 10 minutes to visit and return.
            Node B takes at least 50 minutes to visit and return.
            Node C takes at least 20 minutes to visit and return.
            Even if we send one drone just to visit Node B and come back immediately (the most efficient possible route for B), that drone will take 50 minutes.
            Since
            T is the maximum of all drone times, T must be at least 50 minutes.

        Therefore:
            T ≥ max_j∈P_(MinTrip(j))

        That is why we compute the minimum round-trip for every node and take the maximum of those values to set the lower bound.

        Returns:
            A float representing the lower bound for makespan T.
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
