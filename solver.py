from typing import List, Tuple, Dict, Set
import mip
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
    ):
        self.points = [base_point] + points
        self.entry_threshold = entry_threshold
        self.k_drones = k_drones

        # Constants from problem description
        self.speed_up = speed_up
        self.speed_down = speed_down
        self.speed_horizontal = speed_horizontal

        self.num_nodes = len(points)

        # Precompute valid arcs and costs
        self.arcs: Set[Tuple[int, int]] = set()
        self.entry_points_idx: Set[int] = set()
        self.costs: Dict[Tuple[int, int], float] = {}
        self._build_graph()

    def _build_graph(self):
        """Builds the graph nodes, arcs and calculates travel times using vectorized operations."""
        print("Building graph...")

        for i in range(1, self.num_nodes):
            for j in range(1, self.num_nodes):
                if i == j:
                    continue
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
                    cost = calc_time_between_points(
                        p_i, p_j, self.speed_horizontal, self.speed_up, self.speed_down
                    )
                    self.arcs.add((i, j))
                    self.costs[(i, j)] = cost
        print(f"Total arcs between points: {len(self.arcs)}")

        # Define set of entry points
        for i in range(1, self.num_nodes):
            if self.points[i].y <= self.entry_threshold:
                self.entry_points_idx.add(i)
        print(f"Total entry points: {len(self.entry_points_idx)}")

        # Add arcs from base (0) to entry points and back
        for j in self.entry_points_idx:
            p_j = self.points[j]
            cost_to = calc_time_between_points(
                self.points[0],
                p_j,
                self.speed_horizontal,
                self.speed_up,
                self.speed_down,
            )
            cost_from = calc_time_between_points(
                p_j,
                self.points[0],
                self.speed_horizontal,
                self.speed_up,
                self.speed_down,
            )
            self.arcs.add((0, j))
            self.costs[(0, j)] = cost_to
            self.arcs.add((j, 0))
            self.costs[(j, 0)] = cost_from
        print(f"Total arcs including base: {len(self.arcs)}")

    def solve(self, max_seconds: int = 300):
        """
        Builds and solves the MIP model for the Drone Routing Problem.

        Args:
            max_seconds (int): Maximum time allowed for the solver in seconds.
        """
        model = mip.Model(sense=mip.MINIMIZE, solver_name=mip.CBC)
        model.threads = -1  # Use all available threads

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

        # u_i = auxiliary variables for MTZ subtour elimination
        u = {}
        for i in range(1, self.num_nodes):
            u[i] = model.add_var(
                var_type=mip.CONTINUOUS, name=f"u_{i}", lb=1.0, ub=self.num_nodes - 1
            )
        print(f"Auxiliary variables u_i created: {len(u)}")

        # Obj. function: Minimize Z
        model.objective = mip.minimize(Z)

        # --- Constraints ---

        # 1. Visit Every Point Exactly Once (excluding base)
        # Constraint: sum(x_ijk) over all k, i = 1 for each point j
        for j in range(1, self.num_nodes):
            visit_vars = [
                x[k, i, j]
                for k in range(self.k_drones)
                for i in range(1, self.num_nodes)
                if (i, j) in self.arcs
            ]
            model += mip.xsum(visit_vars) == 1, f"visit_{j}"
        print("Visit constraints added.")

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

        # 5. Subtour Elimination (MTZ)
        for i in range(1, self.num_nodes):
            for j in range(1, self.num_nodes):
                if i != j and (i, j) in self.arcs:
                    for k in range(self.k_drones):
                        model += (
                            u[i] - u[j] + (self.num_nodes - 1) * x[k, i, j]
                            <= self.num_nodes - 2,
                            f"subtour_{k}_{i}_{j}",
                        )
        print("Subtour elimination constraints added.")

        # 6. Entry/Exit Point Constraint
        for k in range(self.k_drones):
            for j in range(1, self.num_nodes):
                if j not in self.entry_points_idx:
                    if (0, j) in self.arcs:
                        model += x[k, 0, j] == 0, f"no_entry_{k}_{j}"
                    if (j, 0) in self.arcs:
                        model += x[k, j, 0] == 0, f"no_exit_{k}_{j}"
        print("Entry/exit point constraints added.")

        # --- Optimization ---
        model.max_seconds = max_seconds
        print("Starting optimization...")
        status = model.optimize()

        print(f"Optimization status: {status}")
        if status in [mip.OptimizationStatus.OPTIMAL, mip.OptimizationStatus.FEASIBLE]:
            print(f"Objective value (Max Time): {model.objective_value:.4f} seconds")
            self._print_solution(x)
        else:
            print("No solution found.")

    def _print_solution(self, x):
        for k in range(self.k_drones):
            path = [0]
            current_node = 0
            while True:
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

                if current_node == 0:
                    break

            # Format the path as a string "0-node1-node2-...-0"
            path_str = "-".join(map(str, path))
            print(f"Drone {k+1}: {path_str}")
