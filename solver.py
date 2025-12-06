import mip
import math
from typing import List, Tuple, Dict, Set
from itertools import product
from utils import Point3D, calc_time_between_points

class SubtourElimination(mip.ConstrsGenerator):
    """
    Lazy constraint callback for Subtour Elimination.
    Identifies disconnected components in the solution and adds cuts to eliminate them.
    """
    def __init__(self, model: mip.Model, x_vars: Dict[Tuple[int, int, int], mip.Var], num_nodes: int, num_drones: int):
        self.model = model
        self.x_vars = x_vars
        self.num_nodes = num_nodes # Includes base at index 0
        self.num_drones = num_drones

    def generate_constrs(self, model: mip.Model, depth: int = 0, npass: int = 0):
        # Extract active arcs from the current solution
        # We only care if an arc is used by ANY drone
        active_arcs = []
        
        # x_vars keys are (k, i, j)
        # We want to build an adjacency list for the graph of visited nodes
        adj = {i: [] for i in range(self.num_nodes)}
        
        # Check values of variables in the current relaxation/solution
        for (k, i, j), var in self.x_vars.items():
            val = var.x
            if val is not None and float(val) > 0.5: # If variable is effectively 1
                adj[i].append(j)

        # Find connected components using BFS/DFS
        visited = [False] * self.num_nodes
        components = []

        for i in range(self.num_nodes):
            if not visited[i] and adj[i]: # Only consider nodes that are part of some path
                component = set()
                stack = [i]
                visited[i] = True
                while stack:
                    u = stack.pop()
                    component.add(u)
                    for v in adj[u]:
                        # We treat the graph as undirected for component finding to catch isolated clusters
                        # But strictly speaking, we are looking for cycles. 
                        # However, for VRP, finding components not connected to the depot (node 0) is the standard way.
                        if not visited[v]:
                            visited[v] = True
                            stack.append(v)
                components.append(component)

        # For each component that does NOT contain the depot (node 0), add a cut
        for comp in components:
            if 0 not in comp:
                # This component is a subtour disconnected from the base
                # Cut: sum(x_ijk for i in comp, j not in comp, all k) >= 1
                # This forces at least one drone to leave this set of nodes
                
                # Identify outgoing arcs from this component
                outgoing_vars = []
                for i in comp:
                    for j in range(self.num_nodes):
                        if j not in comp:
                            # Check if arc (i, j) exists for any drone
                            for k in range(self.num_drones):
                                if (k, i, j) in self.x_vars:
                                    outgoing_vars.append(self.x_vars[k, i, j])
                
                if outgoing_vars:
                    model += mip.xsum(outgoing_vars) >= 1


class DroneRoutingSolver:
    def __init__(self, points: List[Point3D], base_point: Point3D, entry_threshold: float, k_drones: int = 4):
        self.points = points
        self.base_point = base_point
        self.entry_threshold = entry_threshold
        self.k_drones = k_drones
        
        # Constants from problem description
        self.SPEED_UP = 1.0
        self.SPEED_DOWN = 2.0
        self.SPEED_HORIZONTAL = 1.5
        
        # Nodes: 0 is Base, 1..N are points
        self.num_points = len(points)
        self.num_nodes = self.num_points + 1
        
        # Precompute valid arcs and costs
        self.arcs: List[Tuple[int, int]] = []
        self.costs: Dict[Tuple[int, int], float] = {}
        self.incoming_arcs: Dict[int, List[int]] = {i: [] for i in range(self.num_nodes)}
        self.outgoing_arcs: Dict[int, List[int]] = {i: [] for i in range(self.num_nodes)}
        self._build_graph()

    def _is_connected(self, p1: Point3D, p2: Point3D) -> bool:
        """Check if two points are connected based on problem rules."""
        dist = p1.distance_to(p2)
        
        # Rule 1: Euclidean distance <= 4m
        if dist <= 4.0:
            return True
            
        # Rule 2: Euclidean distance <= 11m AND 2 coords differ by <= 0.5m
        if dist <= 11.0:
            diffs = [abs(p1.x - p2.x), abs(p1.y - p2.y), abs(p1.z - p2.z)]
            count_small_diffs = sum(1 for d in diffs if d <= 0.5)
            if count_small_diffs >= 2:
                return True
                
        return False

    def _build_graph(self):
        """Builds the graph nodes, arcs and calculates travel times."""
        # Node 0 is Base. Nodes 1..N correspond to points[0..N-1]
        
        # 1. Arcs between Target Points (1..N)
        for i in range(self.num_points):
            for j in range(self.num_points):
                if i == j:
                    continue
                
                u, v = i + 1, j + 1
                p1, p2 = self.points[i], self.points[j]
                
                if self._is_connected(p1, p2):
                    self.arcs.append((u, v))
                    self.costs[(u, v)] = calc_time_between_points(
                        p1, p2, self.SPEED_HORIZONTAL, self.SPEED_UP, self.SPEED_DOWN
                    )
                    self.outgoing_arcs[u].append(v)
                    self.incoming_arcs[v].append(u)

        # 2. Arcs between Base (0) and Entry Points
        # Entry points are those with y <= threshold
        entry_indices = [i for i, p in enumerate(self.points) if p.y <= self.entry_threshold]
        
        for idx in entry_indices:
            u_node = idx + 1 # Convert to 1-based index
            p = self.points[idx]
            
            # Base -> Entry Point
            self.arcs.append((0, u_node))
            self.costs[(0, u_node)] = calc_time_between_points(
                self.base_point, p, self.SPEED_HORIZONTAL, self.SPEED_UP, self.SPEED_DOWN
            )
            self.outgoing_arcs[0].append(u_node)
            self.incoming_arcs[u_node].append(0)
            
            # Entry Point -> Base
            self.arcs.append((u_node, 0))
            self.costs[(u_node, 0)] = calc_time_between_points(
                p, self.base_point, self.SPEED_HORIZONTAL, self.SPEED_UP, self.SPEED_DOWN
            )
            self.outgoing_arcs[u_node].append(0)
            self.incoming_arcs[0].append(u_node)

    def solve(self, max_seconds: int = 300):
        """
        Builds and solves the MIP model for the Drone Routing Problem.
        
        Args:
            max_seconds (int): Maximum time allowed for the solver in seconds.
        """
        model = mip.Model(sense=mip.MINIMIZE, solver_name=mip.CBC)
        # --- Decision Variables ---
        
        # x[k, i, j] = 1 if drone k travels from i to j
        # We only create variables for valid arcs to reduce model size
        x = {}
        for k in range(self.k_drones):
            for (i, j) in self.arcs:
                x[k, i, j] = model.add_var(var_type=mip.BINARY, name=f"x_{k}_{i}_{j}")
        print(f"Variables x_ijk created: {len(x)}")
        
        # Z = Max time (makespan)
        # This is the variable we want to minimize
        Z = model.add_var(var_type=mip.CONTINUOUS, name="Z")
        print("Variable Z created.")
        
        # --- Constraints ---
        
        # 1. Visit Every Point Exactly Once (excluding base)
        for j in range(1, self.num_nodes):
            # Sum over all drones and all incoming arcs to j
            incoming_vars = [x[k, i, j] for k in range(self.k_drones) for i in self.incoming_arcs[j]]
            model += mip.xsum(incoming_vars) == 1, f"visit_{j}"
            if j % 20 == 0: print(f"Visit constraint added up to node {j}.") 
        print("Visit constraints added.")

        # 2. Flow Conservation
        # Constraint: sum(incoming) = sum(outgoing) for each drone k and point j
        for k in range(self.k_drones):
            for j in range(1, self.num_nodes):
                incoming = [x[k, i, j] for i in self.incoming_arcs[j]]
                outgoing = [x[k, j, m] for m in self.outgoing_arcs[j]]
                model += mip.xsum(incoming) == mip.xsum(outgoing), f"flow_{k}_{j}"
            print(f"Flow conservation constraints added for drone {k}.")
        print("Flow conservation constraints added.")
        
        # 3. Depot Constraints
        # Each drone leaves base exactly once and returns exactly once
        # Base is node 0
        for k in range(self.k_drones):
            outgoing_base = [x[k, 0, j] for j in self.outgoing_arcs[0]]
            incoming_base = [x[k, i, 0] for i in self.incoming_arcs[0]]
            
            model += mip.xsum(outgoing_base) == 1, f"depot_out_{k}"
            model += mip.xsum(incoming_base) == 1, f"depot_in_{k}"
        print("Depot constraints added.")
        
        # 4. Minimax Time Constraint
        # Z >= Total time for drone k
        for k in range(self.k_drones):
            travel_time = mip.xsum(self.costs[i, j] * x[k, i, j] for (i, j) in self.arcs)
            model += travel_time <= Z, f"makespan_{k}"
        print("Minimax time constraints added.")
        
        # 6. Symmetry Breaking (Optional but recommended)
        # Force drone k to have >= travel time than drone k+1
        for k in range(self.k_drones - 1):
             time_k = mip.xsum(self.costs[i, j] * x[k, i, j] for (i, j) in self.arcs)
             time_next = mip.xsum(self.costs[i, j] * x[k+1, i, j] for (i, j) in self.arcs)
             model += time_k >= time_next, f"symmetry_{k}"
        print("Symmetry breaking constraints added.")
        
        # 5. Subtour Elimination (Lazy Constraints)
        model.cuts_generator = SubtourElimination(model, x, self.num_nodes, self.k_drones)
        model.lazy_constrs_generator = SubtourElimination(model, x, self.num_nodes, self.k_drones)
        print("Subtour elimination constraints generator added.")
        
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
        print("\n--- Drone Routes ---")
        for k in range(self.k_drones):
            print(f"Drone {k+1}:")
            
            # Reconstruct path
            # Start at 0
            curr = 0
            route = [0]
            total_time = 0.0
            
            while True:
                next_node = None
                for j in range(self.num_nodes):
                    if (curr, j) in self.arcs and x[k, curr, j].x > 0.5:
                        next_node = j
                        break
                
                if next_node is None:
                    print("  Error: Broken path")
                    break
                
                time = self.costs[curr, next_node]
                total_time += time
                route.append(next_node)
                curr = next_node
                
                if curr == 0:
                    break
            
            # Map back to point indices (0-based for user, but internal 1..N are points 0..N-1)
            # Route indices: 0 -> Base, i -> Point i-1
            route_str = "Base"
            for node in route[1:-1]:
                route_str += f" -> P{node-1}"
            route_str += " -> Base"
            
            print(f"  Path: {route_str}")
            print(f"  Total Time: {total_time:.2f} s")
