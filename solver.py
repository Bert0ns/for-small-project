from typing import List, Tuple, Dict, Set
import mip
import networkx as nx
from utils import Point3D, calc_time_between_points
import time


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
    ):
        self.points = [base_point] + points
        self.entry_threshold = entry_threshold
        self.k_drones = k_drones

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
        print("Building graph with NetworkX (Directed)...")

        self.graph.add_nodes_from(range(self.num_nodes))

        # 1. Edges between target points
        for i in range(1, self.num_nodes):
            for j in range(i + 1, self.num_nodes):
                p_i = self.points[i]
                p_j = self.points[j]
                is_connected = False

                euclidean_dist = p_i.distance_to(p_j)
                if euclidean_dist <= 4.0:
                    is_connected = True
                elif euclidean_dist <= 11.0:
                    diffs = [abs(p_i.x - p_j.x), abs(p_i.y - p_j.y), abs(p_i.z - p_j.z)]
                    small_diffs = sum(1 for d in diffs if d <= 0.5)
                    if small_diffs >= 2:
                        is_connected = True

                if is_connected:
                    # Calculate costs for both directions
                    cost_ij = calc_time_between_points(
                        p_i, p_j, self.speed_horizontal, self.speed_up, self.speed_down
                    )
                    cost_ji = calc_time_between_points(
                        p_j, p_i, self.speed_horizontal, self.speed_up, self.speed_down
                    )
                    self.graph.add_edge(i, j, weight=cost_ij)
                    self.graph.add_edge(j, i, weight=cost_ji)

        # 2. Identify entry points and add edges to base (node 0)
        for i in range(1, self.num_nodes):
            if self.points[i].y <= self.entry_threshold:
                self.entry_points_idx.add(i)
                p_0 = self.points[0]
                p_i = self.points[i]

                cost_0i = calc_time_between_points(
                    p_0, p_i, self.speed_horizontal, self.speed_up, self.speed_down
                )
                cost_i0 = calc_time_between_points(
                    p_i, p_0, self.speed_horizontal, self.speed_up, self.speed_down
                )

                self.graph.add_edge(0, i, weight=cost_0i)
                self.graph.add_edge(i, 0, weight=cost_i0)

        print(
            f"NetworkX Graph: {self.graph.number_of_nodes()} nodes, {self.graph.number_of_edges()} edges"
        )

        # 3. Populate arcs and costs for the MIP solver
        for u, v, data in self.graph.edges(data=True):
            self.arcs.add((u, v))
            self.costs[(u, v)] = data["weight"]

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

    def solve(self, max_seconds: int = 300):
        """
        Builds and solves the MIP model for the Drone Routing Problem.

        Args:
            max_seconds (int): Maximum time allowed for the solver in seconds.
        """
        model = mip.Model(sense=mip.MINIMIZE, solver_name=mip.CBC)
        model.max_mip_gap = 0.05  # Relaxed to 5% gap for performance
        model.threads = -1

        start_time = time.time()

        # --- Decision Variables ---

        # x[k, i, j] = 1 if drone k travels from i to j
        # We only create variables for valid arcs to reduce model size
        x = {}
        for k in range(self.k_drones):
            for i, j in self.arcs:
                x[k, i, j] = model.add_var(var_type=mip.BINARY, name=f"x_{k}_{i}_{j}")
        print(f"Variables x_ijk created: {len(x)}")

        # Z = Max time (makespan)
        # This is the variable we want to minimize
        Z = model.add_var(var_type=mip.CONTINUOUS, name="Z", lb=0.0)
        print("Variable Z created.")

        # u_i = auxiliary variables for MTZ subtour elimination (Removed)
        # u = {}
        # for i in range(1, self.num_nodes):
        #     u[i] = model.add_var(
        #         var_type=mip.CONTINUOUS, name=f"u_{i}", lb=1.0, ub=self.num_nodes - 1
        #     )
        # print(f"Auxiliary variables u_i created: {len(u)}")

        # Obj. function: Minimize Z
        model.objective = mip.minimize(Z)

        # --- Constraints ---

        # 1. Visit Each Point At Least Once (Relaxed to allow revisits)
        # We use "At Least Once" because the graph contains dead ends (nodes with degree 1),
        # which require entering and leaving via the same edge (revisit).
        for j in range(1, self.num_nodes):
            model += (
                mip.xsum(
                    x[k, i, j]
                    for k in range(self.k_drones)
                    for i in range(self.num_nodes)
                    if (i, j) in self.arcs
                )
                >= 1,
                f"visit_{j}",
            )
        print("Visit at least once constraints added.")

        # 2. Flow Conservation
        # Constraint: sum(incoming) = sum(outgoing) for each drone k and point j
        for k in range(self.k_drones):
            for j in range(self.num_nodes):
                incoming = [
                    x[k, i, j] for i in range(self.num_nodes) if (i, j) in self.arcs
                ]
                outgoing = [
                    x[k, j, l] for l in range(self.num_nodes) if (j, l) in self.arcs
                ]
                model += mip.xsum(incoming) == mip.xsum(outgoing), f"flow_{k}_{j}"
        print("Flow conservation constraints added.")

        # 3. Depot Constraints
        # Each drone leaves base exactly once and returns exactly once
        # Base is node 0
        for k in range(self.k_drones):
            depart_vars = [
                x[k, 0, j]
                for j in range(1, self.num_nodes)
                if (0, j) in self.arcs and j in self.entry_points_idx
            ]
            return_vars = [
                x[k, j, 0]
                for j in range(1, self.num_nodes)
                if (j, 0) in self.arcs and j in self.entry_points_idx
            ]
            model += mip.xsum(depart_vars) == 1, f"depart_{k}"
            model += mip.xsum(return_vars) == 1, f"return_{k}"
        print("Depot constraints added.")

        # 4. Minimax Time Constraint
        # Z >= Total time for drone k
        # This links the objective variable Z to the actual travel times
        for k in range(self.k_drones):
            travel_time = mip.xsum(
                self.costs[i, j] * x[k, i, j] for (i, j) in self.arcs
            )
            model += travel_time <= Z, f"makespan_{k}"
        print("Minimax time constraints added.")

        # 5. Subtour Elimination (Iterative / Lazy)
        # We will handle subtour elimination iteratively in the solve loop.
        # This avoids the heavy MTZ constraints and allows for revisits.
        print("Subtour elimination will be handled iteratively.")

        # 6. Entry/Exit Point Constraint (maybe is redundant)
        # for k in range(self.k_drones):
        # for j in range(1, self.num_nodes):
        # if j not in self.entry_points_idx:
        # if (0, j) in self.arcs:
        # model += x[k, 0, j] == 0, f"no_entry_{k}_{j}"
        # if (j, 0) in self.arcs:
        # model += x[k, j, 0] == 0, f"no_exit_{k}_{j}"
        # print("Entry/exit point constraints added.")

        # --- Optimization Loop (Iterative Subtour Elimination) ---
        print("Starting optimization loop...")

        iteration = 0
        while True:
            iteration += 1

            # Check remaining time
            elapsed = time.time() - start_time
            remaining = max_seconds - elapsed
            if remaining <= 0:
                print("Global time limit reached.")
                break

            model.max_seconds = remaining
            print(f"--- Iteration {iteration} (Time left: {remaining:.1f}s) ---")

            status = model.optimize()
            print(f"Optimization status: {status}")

            if status not in [
                mip.OptimizationStatus.OPTIMAL,
                mip.OptimizationStatus.FEASIBLE,
            ]:
                print("No solution found.")
                return None

            # Check for subtours
            subtours = self._find_subtours(x)
            if not subtours:
                print("No subtours found. Solution is valid.")
                print(
                    f"Objective value (Max Time): {model.objective_value:.4f} seconds"
                )
                return self._extract_solution(x)

            print(f"Found {len(subtours)} subtours. Adding constraints...")

            # Add cut-set constraints for each subtour
            # Limit the number of cuts added per iteration to avoid bloating the model too fast
            # We prioritize smaller subtours as they are tighter cuts
            subtours.sort(key=len)

            cuts_added = 0
            for component in subtours:
                # Constraint: sum(x_ij) for i in S, j not in S >= 1
                # We sum over all drones
                cut_edges = []
                for k in range(self.k_drones):
                    for i in component:
                        for j in range(self.num_nodes):
                            if j not in component and (i, j) in self.arcs:
                                cut_edges.append(x[k, i, j])

                if cut_edges:
                    model += (
                        mip.xsum(cut_edges) >= 1,
                        f"subtour_cut_{iteration}_{list(component)[0]}",
                    )
                    cuts_added += 1

                # Safety break if too many cuts (optional, but helps performance)
                if cuts_added > 50:
                    print(f"Added 50 cuts (out of {len(subtours)}). Re-optimizing...")
                    break

            if cuts_added == 0:
                print(
                    "Warning: Subtours found but no valid cuts could be generated (no outgoing edges?)."
                )
                break

        print("Loop finished without valid solution.")
        return None

    def _find_subtours(self, x):
        """
        Finds connected components in the solution graph using NetworkX.
        Returns a list of sets, where each set is a component that DOES NOT contain the base (node 0).
        """
        # Build a graph from active edges
        G_sol = nx.Graph()
        G_sol.add_nodes_from(range(self.num_nodes))

        for k in range(self.k_drones):
            for i, j in self.arcs:
                if x[k, i, j].x >= 0.5:
                    G_sol.add_edge(i, j)

        # Find connected components
        components = list(nx.connected_components(G_sol))

        # Filter out the component containing the base (node 0)
        subtours = []
        for comp in components:
            if 0 not in comp:
                subtours.append(comp)

        return subtours

    def _extract_solution(self, x):
        paths = []
        for k in range(self.k_drones):
            path = [0]
            current_node = 0
            steps = 0
            max_steps = self.num_nodes * 2  # Safety limit

            while steps < max_steps:
                next_node = None
                # Find the next node in the path
                # We look for j such that x[k, current_node, j] == 1
                for j in range(self.num_nodes):
                    if (k, current_node, j) in x and x[k, current_node, j].x >= 0.99:
                        next_node = j
                        break

                if next_node is None:
                    break

                path.append(next_node)
                current_node = next_node
                steps += 1

                if current_node == 0:
                    break

            if steps >= max_steps:
                print(
                    f"Warning: Drone {k+1} path extraction hit safety limit. Possible cycle."
                )

            # Format the path as a string "0-node1-node2-...-0"
            path_str = "-".join(map(str, path))
            print(f"Drone {k+1}: {path_str}")
            paths.append(path)
        return paths
