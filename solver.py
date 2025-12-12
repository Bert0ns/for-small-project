"""
Drone Routing Problem Solver using MIP and Heuristic DFS Warm Start (v8)
"""

import time
import multiprocessing
import random
import sys
from typing import List, Tuple, Dict, Set
import mip
import networkx as nx
import numpy as np
from utils import Point3D, calc_time_between_points


def solve_dfs(num_nodes, k_drones, arcs, out_edges, costs, seed=None):
    """
    Standalone DFS worker for parallel execution.
    """
    sys.setrecursionlimit(3000)

    if seed is not None:
        random.seed(seed)

    # State
    routes = [[0] for _ in range(k_drones)]
    current_nodes = [0] * k_drones
    unvisited = set(range(1, num_nodes))
    drones_finished = [False] * k_drones

    # Track costs to balance
    drone_costs = [0.0] * k_drones

    def dfs():
        if not unvisited:
            # All nodes visited. Try to return active drones to base.
            moves_to_undo = []

            for k in range(k_drones):
                if not drones_finished[k]:
                    u = current_nodes[k]
                    if u != 0:
                        if (u, 0) in arcs:
                            routes[k].append(0)
                            moves_to_undo.append(k)
                        else:
                            # Backtrack changes
                            for dk in moves_to_undo:
                                routes[dk].pop()
                            return False
            return True

        # Pick drone to move
        # Prioritize drones that are not finished and have lower cost
        active_drones = [k for k in range(k_drones) if not drones_finished[k]]
        # Sort by cost to keep balanced
        active_drones.sort(key=lambda k: drone_costs[k])

        # If randomized, maybe shuffle equal cost drones or add noise?
        # For now, let's just rely on candidate randomization if seed is set.

        for k in active_drones:
            u = current_nodes[k]

            # 1. Try moving to unvisited nodes
            candidates = [v for v in out_edges[u] if v in unvisited]

            # Lookahead Pruning
            viable_candidates = []
            for v in candidates:
                can_return = (v, 0) in arcs
                has_next = any(n in unvisited and n != v for n in out_edges[v])

                if can_return or has_next:
                    viable_candidates.append(v)

            # Sort by cost
            # Add randomization to cost if seed is present to explore different paths
            if seed is not None:
                viable_candidates.sort(
                    key=lambda v: costs[(u, v)] * random.uniform(0.9, 1.1)
                )
            else:
                viable_candidates.sort(key=lambda v: costs[(u, v)])

            # Beam Search: Limit to top 3
            viable_candidates = viable_candidates[:3]

            for v in viable_candidates:
                # Move
                routes[k].append(v)
                current_nodes[k] = v
                unvisited.remove(v)
                cost = costs[(u, v)]
                drone_costs[k] += cost

                if dfs():
                    return True

                # Backtrack
                drone_costs[k] -= cost
                unvisited.add(v)
                current_nodes[k] = u
                routes[k].pop()

            # 2. Try finishing this drone
            if len(active_drones) > 1 and u != 0 and (u, 0) in arcs:
                drones_finished[k] = True
                routes[k].append(0)
                cost = costs[(u, 0)]
                drone_costs[k] += cost
                current_nodes[k] = 0

                if dfs():
                    return True

                # Backtrack
                current_nodes[k] = u
                drone_costs[k] -= cost
                routes[k].pop()
                drones_finished[k] = False

        return False

    if dfs():
        return routes
    return None


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
        model.threads = -1
        model.verbose = int(self.verbose)

        # Sets
        V = range(self.num_nodes)
        V_prime = range(1, self.num_nodes)
        K = range(self.k_drones)

        # Variables
        # x[k, i, j] = 1 if drone k travels from i to j
        x = {}
        for k in K:
            for i, j in self.arcs:
                x[(k, i, j)] = model.add_var(var_type=mip.BINARY, name=f"x_{k}_{i}_{j}")

        Z = model.add_var(var_type=mip.CONTINUOUS, name="Z")

        # Objective
        model.objective = mip.minimize(Z)

        # 6.1 Assignment Constraint
        for j in V_prime:
            model += (
                mip.xsum(x[(k, i, j)] for k in K for i in V if (i, j) in self.arcs) == 1
            )

        # 6.2 Flow Conservation
        for k in K:
            for j in V_prime:
                incoming = mip.xsum(x[(k, i, j)] for i in V if (i, j) in self.arcs)
                outgoing = mip.xsum(x[(k, j, l)] for l in V if (j, l) in self.arcs)
                model += incoming == outgoing

        # 6.3 Depot Constraints (Optional Drones)
        # Each drone may leave and return at most once
        for k in K:
            model += mip.xsum(x[(k, 0, j)] for j in V_prime if (0, j) in self.arcs) <= 1  # type: ignore
            model += mip.xsum(x[(k, i, 0)] for i in V_prime if (i, 0) in self.arcs) <= 1  # type: ignore

        # Balance constraint: if a drone leaves, it must return
        for k in K:
            outgoing = mip.xsum(x[(k, 0, j)] for j in V_prime if (0, j) in self.arcs)
            incoming = mip.xsum(x[(k, i, 0)] for i in V_prime if (i, 0) in self.arcs)
            model += outgoing == incoming

        # 6.4 Min-Max Time Bound
        for k in K:
            travel_time = mip.xsum(
                self.costs[(i, j)] * x[(k, i, j)] for (i, j) in self.arcs
            )
            model += travel_time <= Z

        # 6.5 Valid Inequality: Lower Bound on Z
        total_time = mip.xsum(
            self.costs[(i, j)] * x[(k, i, j)] for k in K for (i, j) in self.arcs
        )
        model += Z >= total_time / self.k_drones  # type: ignore

        # 6.6 Symmetry Breaking
        # 1. Force drones to be used in order: if k+1 is used, k must be used.
        # sum(x_0j^{k+1}) <= sum(x_0j^k)
        # 2. If both are used, force ordering by first visited node index.
        # sum(j * x_0j^k) + 1 <= sum(j * x_0j^{k+1}) + M * (1 - sum(x_0j^{k+1}))

        M = self.num_nodes + 2
        for k in range(self.k_drones - 1):
            u_k = mip.xsum(x[(k, 0, j)] for j in V_prime if (0, j) in self.arcs)
            u_next = mip.xsum(x[(k + 1, 0, j)] for j in V_prime if (0, j) in self.arcs)

            s_k = mip.xsum(j * x[(k, 0, j)] for j in V_prime if (0, j) in self.arcs)
            s_next = mip.xsum(
                j * x[(k + 1, 0, j)] for j in V_prime if (0, j) in self.arcs
            )

            # Force usage order
            model += u_next <= u_k  # type: ignore

            # Force index ordering if next drone is used
            model += s_k + 1 <= s_next + M * (1 - u_next)  # type: ignore

        # Initial Lower Bound for Z
        max_min_trip = self._get_makespan_lower_bound()
        if self.verbose:
            print(f"Setting lower bound for makespan T to {max_min_trip:.2f}")
        model += Z >= max_min_trip  # type: ignore

        # Warm start
        start_vars = []
        if warm_start:
            initial_solution = self._greedy_warm_start()
            if initial_solution:
                for k, i, j in initial_solution:
                    if (k, i, j) in x:
                        start_vars.append((x[(k, i, j)], 1.0))
                model.start = start_vars
            else:
                if self.verbose:
                    print("No valid warm start solution found.")

        # Iterative Subtour Elimination
        start_time = time.time()
        iteration = 0

        while True:
            iteration += 1

            # Re-inject warm start for every iteration to ensure feasibility
            if start_vars:
                model.start = start_vars
            elapsed = time.time() - start_time
            remaining_time = max_seconds - elapsed
            if remaining_time <= 0:
                if self.verbose:
                    print("Time limit reached.")
                break

            status = model.optimize(max_seconds=remaining_time)

            if status not in [
                mip.OptimizationStatus.OPTIMAL,
                mip.OptimizationStatus.FEASIBLE,
            ]:
                if self.verbose:
                    print("No feasible solution found.")
                return []

            # Check for subtours
            subtours = self._find_subtours(x, V_prime)
            if not subtours:
                if self.verbose:
                    print(
                        f"Optimal solution found in iteration {iteration} with Objective: {model.objective_value}"
                    )
                return self._extract_solution(x, K)

            if self.verbose:
                print(
                    f"--------------Iteration {iteration}-------------- \nFound {len(subtours)} subtours. Adding constraints..."
                )

            for S in subtours:
                # Constraint: sum(x_ij for i in S, j in S) <= |S| - 1
                # Sum over all drones
                model += (
                    mip.xsum(
                        x[(k, i, j)]
                        for k in K
                        for i in S
                        for j in S
                        if (i, j) in self.arcs
                    )
                    <= len(S) - 1
                )  # type: ignore

        return []

    def _greedy_warm_start(self) -> List[Tuple[int, int, int]]:
        """
        Generates a feasible initial solution using Parallel Heuristic DFS.
        """
        if self.verbose:
            print("Generating greedy warm start (Parallel Heuristic DFS)...")

        # Prepare seeds: 1 deterministic + 7 random
        seeds = [None] + [i for i in range(7)]

        found_routes = None

        # Use multiprocessing.Pool to allow immediate termination of workers
        # This prevents the solver from waiting for all DFS branches to finish (or timeout)
        # when a solution has already been found.
        pool = multiprocessing.Pool(processes=len(seeds))
        async_results = []

        for seed in seeds:
            res = pool.apply_async(
                solve_dfs,
                (
                    self.num_nodes,
                    self.k_drones,
                    self.arcs,
                    self.out_edges,
                    self.costs,
                    seed,
                ),
            )
            async_results.append(res)

        start_time = time.time()
        time_limit = 60.0

        try:
            while True:
                elapsed = time.time() - start_time
                if elapsed > time_limit:
                    if self.verbose:
                        print("Greedy warm start timed out.")
                    break

                # Check if any result is ready
                all_done = True
                for res in async_results:
                    if res.ready():
                        val = None
                        try:
                            val = res.get(timeout=0.01)
                        except Exception:
                            pass  # Worker failed or no result

                        if val:
                            if self.verbose:
                                print("Solution found by a worker!")
                            found_routes = val
                            raise StopIteration  # Break out of loop
                    else:
                        all_done = False

                if all_done:
                    break

                time.sleep(0.1)  # Polling interval

        except StopIteration:
            pass
        finally:
            # Terminate all workers immediately
            pool.terminate()
            pool.join()

        if not found_routes:
            if self.verbose:
                print("No solution found by any worker.")
            return []

        # Post-process the found routes (Symmetry Breaking)
        routes = found_routes

        # Sort routes based on the first visited node to satisfy symmetry breaking constraints
        valid_routes = [r for r in routes if len(r) > 1]
        valid_routes.sort(key=lambda r: r[1])
        sorted_routes = valid_routes + [r for r in routes if len(r) <= 1]

        solution = []
        for k, route in enumerate(sorted_routes):
            if k >= self.k_drones:
                break
            for i in range(len(route) - 1):
                u, v = route[i], route[i + 1]
                solution.append((k, u, v))

        return solution

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

    def _find_subtours(self, x_vars, V_prime):
        """
        Finds subtours in the current solution.
        Returns a list of lists, where each inner list is a set of nodes in a subtour.
        """
        # Build adjacency list for active edges
        adj = {i: [] for i in range(self.num_nodes)}
        for (k, i, j), var in x_vars.items():
            if var.x is not None and var.x >= 0.99:
                adj[i].append(j)

        # Find all nodes reachable from 0
        reachable = set()
        stack = [0]
        while stack:
            u = stack.pop()
            if u in reachable:
                continue
            reachable.add(u)
            for v in adj[u]:
                stack.append(v)

        # Nodes in V_prime that are visited but not reachable from 0 are in subtours
        # Note: In this model, all j in V_prime MUST be visited.
        # So any j not in reachable is part of a subtour.
        unreachable = [j for j in V_prime if j not in reachable]

        if not unreachable:
            return []

        # Group unreachable nodes into connected components (subtours)
        subtours = []
        unvisited_set = set(unreachable)

        while unvisited_set:
            start_node = next(iter(unvisited_set))
            component = set()
            stack = [start_node]

            while stack:
                u = stack.pop()
                if u in component:
                    continue
                component.add(u)
                if u in unvisited_set:
                    unvisited_set.remove(u)

                for v in adj[u]:
                    if v in unvisited_set or v in component:
                        stack.append(v)

            subtours.append(list(component))

        return subtours

    def _extract_solution(self, x, K_range):
        paths = []

        # Pre-process adjacency for each drone
        drone_adj = {k: {} for k in K_range}
        for (k, u, v), var in x.items():
            if var.x is not None and var.x > 0.5:
                if u not in drone_adj[k]:
                    drone_adj[k][u] = []
                drone_adj[k][u].append(v)

        for k in K_range:
            adj = drone_adj[k]

            # Start at 0
            path = [0]
            curr = 0

            # Follow the path
            while True:
                if curr not in adj or not adj[curr]:
                    break
                next_node = adj[curr].pop()
                path.append(next_node)
                curr = next_node
                if curr == 0:
                    break

            if self.verbose:
                path_str = "-".join(map(str, path))
                print(f"Drone {k+1}: {path_str}")
            paths.append(path)

        return paths
