import mip
import math
import numpy as np
from typing import List, Tuple, Dict, Set
from itertools import product
from utils import Point3D, calc_time_between_points


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

    def _build_graph(self):
        """Builds the graph nodes, arcs and calculates travel times using vectorized operations."""
        print("Building graph with numpy...")
        
        # Convert points to numpy array (N, 3)
        # points[i] corresponds to node i+1
        coords = np.array([[p.x, p.y, p.z] for p in self.points])
        N = self.num_points
        
        # --- 1. Arcs between Target Points (1..N) ---
        
        # Compute pairwise differences: diffs[i, j, :] = coords[i] - coords[j]
        # This creates an (N, N, 3) array. For N=2800, this is ~188MB, which is fine.
        # Using broadcasting: (N, 1, 3) - (1, N, 3)
        diffs = coords[:, np.newaxis, :] - coords[np.newaxis, :, :]
        
        # Squared Euclidean distances: (N, N)
        dists_sq = np.sum(diffs**2, axis=2)
        dists = np.sqrt(dists_sq)
        
        # Connectivity Rules:
        # Rule 1: dist <= 4
        connected_mask = dists <= 4.0
        
        # Rule 2: dist <= 11 AND at least 2 coords differ by <= 0.5
        # abs_diffs: (N, N, 3)
        abs_diffs = np.abs(diffs)
        small_diffs_count = np.sum(abs_diffs <= 0.5, axis=2) # (N, N)
        rule2_mask = (dists <= 11.0) & (small_diffs_count >= 2)
        
        # Combine rules
        connected_mask = connected_mask | rule2_mask
        
        # Remove self-loops
        np.fill_diagonal(connected_mask, False)
        
        # Get indices of connected pairs
        # indices are 0-based relative to 'points' list, so they map to nodes 1..N
        src_indices, dst_indices = np.where(connected_mask)
        
        print(f"Found {len(src_indices)} valid arcs between points.")
        
        # Calculate travel times for these arcs using utils.py
        for k in range(len(src_indices)):
            idx_u = src_indices[k]
            idx_v = dst_indices[k]
            
            p_u = self.points[idx_u]
            p_v = self.points[idx_v]
            
            cost = calc_time_between_points(
                p_u, p_v, 
                self.SPEED_HORIZONTAL, 
                self.SPEED_UP, 
                self.SPEED_DOWN
            )
            
            u = int(idx_u + 1)
            v = int(idx_v + 1)
            
            self.arcs.append((u, v))
            self.costs[(u, v)] = cost
            self.outgoing_arcs[u].append(v)
            self.incoming_arcs[v].append(u)
            
        # --- 2. Arcs between Base (0) and Entry Points ---
        # Entry points: y <= threshold
        # We can use numpy to find these indices
        entry_mask = coords[:, 1] <= self.entry_threshold
        entry_indices = np.where(entry_mask)[0]
        
        print(f"Found {len(entry_indices)} entry points.")
        
        for idx in entry_indices:
            u_node = int(idx + 1)
            p_point = self.points[idx]
            
            # Base -> Entry Point
            time_out = calc_time_between_points(
                self.base_point, p_point,
                self.SPEED_HORIZONTAL,
                self.SPEED_UP,
                self.SPEED_DOWN
            )
            
            self.arcs.append((0, u_node))
            self.costs[(0, u_node)] = time_out
            self.outgoing_arcs[0].append(u_node)
            self.incoming_arcs[u_node].append(0)
            
            # Entry Point -> Base
            time_in = calc_time_between_points(
                p_point, self.base_point,
                self.SPEED_HORIZONTAL,
                self.SPEED_UP,
                self.SPEED_DOWN
            )
            
            self.arcs.append((u_node, 0))
            self.costs[(u_node, 0)] = time_in
            self.outgoing_arcs[u_node].append(0)
            self.incoming_arcs[0].append(u_node)

    def solve(self, max_seconds: int = 300):
        """
        Builds and solves the MIP model for the Drone Routing Problem.
        
        Args:
            max_seconds (int): Maximum time allowed for the solver in seconds.
        """
        model = mip.Model(sense=mip.MINIMIZE, solver_name=mip.CBC)
        model.threads = -1 # Use all available threads
        
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
        model.objective = mip.minimize(Z)
        print("Variable Z created.")
        
        # --- Constraints ---
        
        # 1. Visit Every Point Exactly Once (excluding base)
        # Constraint: sum(x_ijk) over all k, i = 1 for each point j
        for j in range(1, self.num_nodes):
            # Sum over all drones and all incoming arcs to j
            incoming_vars = [x[k, i, j] for k in range(self.k_drones) for i in self.incoming_arcs[j]]
            model += mip.xsum(incoming_vars) == 1, f"visit_{j}"
        print("Visit constraints added.")

        # 2. Flow Conservation
        # Constraint: sum(incoming) = sum(outgoing) for each drone k and point j
        for k in range(self.k_drones):
            for j in range(1, self.num_nodes):
                incoming = [x[k, i, j] for i in self.incoming_arcs[j]]
                outgoing = [x[k, j, m] for m in self.outgoing_arcs[j]]
                model += mip.xsum(incoming) == mip.xsum(outgoing), f"flow_{k}_{j}"
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
        # This links the objective variable Z to the actual travel times
        for k in range(self.k_drones):
            travel_time = mip.xsum(self.costs[i, j] * x[k, i, j] for (i, j) in self.arcs)
            model += travel_time <= Z, f"makespan_{k}"
        print("Minimax time constraints added.")
        
        
        
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
        pass
        
