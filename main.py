from typing import Final, List, Dict, Tuple
import sys
from pathlib import Path
import mip
import numpy as np
from utils import Point3D, read_points_from_csv, calc_time_between_points, are_points_connected


# Number of drones
K = 4
# Speed constants (meters per second)
SPEED_UP = 1.0
SPEED_DOWN = 2.0
SPEED_HORIZONTAL = 1.5
# Initial points for each building
INITIAL_POINT_B1: Final[Point3D] = Point3D(0.0, -16.0, 0.0)
INITIAL_POINT_B2: Final[Point3D] = Point3D(0.0, -40.0, 0.0)


def solve_drone_routing(
    points: List[Point3D],
    base_point: Point3D,
    entry_points: List[Point3D],
    num_drones: int,
    speed_horizontal: float,
    speed_up: float,
    speed_down: float,
) -> Dict[int, List[int]]:
    """
    Solve the drone routing optimization problem.
    
    Args:
        points: List of points to visit (grid points)
        base_point: Starting/ending point for all drones
        entry_points: Points that can be directly accessed from base
        num_drones: Number of available drones (k)
        speed_horizontal: Horizontal movement speed
        speed_up: Upward movement speed
        speed_down: Downward movement speed
    
    Returns:
        Dictionary mapping drone_id to route (list of point indices, starting and ending with 0)
    """
    n = len(points)
    
    # Build list of all nodes: index 0 = base, indices 1..n = grid points
    all_nodes = [base_point] + points
    
    # Determine which grid points are entry points (indices in all_nodes)
    entry_indices = set()
    for i, p in enumerate(points):
        if p in entry_points:
            entry_indices.add(i + 1)  # +1 because index 0 is base
    
    print(f"\nBuilding connectivity graph for {n} points...")
    
    # Build adjacency matrix and travel time matrix
    # arcs[i][j] = True if there's an arc from node i to node j
    arcs = [[False] * (n + 1) for _ in range(n + 1)]
    travel_time = [[float('inf')] * (n + 1) for _ in range(n + 1)]
    
    # Base point (index 0) can connect to any entry point and vice versa
    for entry_idx in entry_indices:
        time_to_entry = calc_time_between_points(
            all_nodes[0], all_nodes[entry_idx],
            speed_horizontal, speed_up, speed_down
        )
        arcs[0][entry_idx] = True
        travel_time[0][entry_idx] = time_to_entry
        arcs[entry_idx][0] = True
        travel_time[entry_idx][0] = time_to_entry
    
    # Grid points can connect to each other based on connectivity rules
    for i in range(1, n + 1):
        for j in range(1, n + 1):
            if i != j and are_points_connected(all_nodes[i], all_nodes[j]):
                arcs[i][j] = True
                travel_time[i][j] = calc_time_between_points(
                    all_nodes[i], all_nodes[j],
                    speed_horizontal, speed_up, speed_down
                )
    
    print(f"Graph built. Creating MIP model...")
    
    # Create MIP model
    model = mip.Model(sense=mip.MINIMIZE)
    
    # Decision variables
    # x[i][j][k] = 1 if drone k travels from node i to node j
    x = {}
    for i in range(n + 1):
        for j in range(n + 1):
            if i != j and arcs[i][j]:
                for k in range(num_drones):
                    x[i, j, k] = model.add_var(var_type=mip.BINARY, name=f"x_{i}_{j}_{k}")
    
    # y[i][k] = 1 if node i is visited by drone k
    y = {}
    for i in range(1, n + 1):  # Only grid points, not base
        for k in range(num_drones):
            y[i, k] = model.add_var(var_type=mip.BINARY, name=f"y_{i}_{k}")
    
    # T[k] = total time for drone k
    T = [model.add_var(lb=0, name=f"T_{k}") for k in range(num_drones)]
    
    # Makespan variable (maximum time among all drones)
    makespan = model.add_var(lb=0, name="makespan")
    
    # Objective: minimize makespan
    model.objective = makespan
    
    print("Adding constraints...")
    
    # Constraint 1: Each grid point must be visited by exactly one drone
    for i in range(1, n + 1):
        model.add_constr(
            mip.xsum(y[i, k] for k in range(num_drones)) == 1,
            name=f"visit_{i}"
        )
    
    # Constraint 2: If a point is visited by a drone, there must be incoming and outgoing arcs
    for k in range(num_drones):
        # For each grid point
        for i in range(1, n + 1):
            # Flow in = flow out
            in_flow = mip.xsum(x[j, i, k] for j in range(n + 1) if j != i and arcs[j][i])
            out_flow = mip.xsum(x[i, j, k] for j in range(n + 1) if j != i and arcs[i][j])
            model.add_constr(in_flow == y[i, k], name=f"flow_in_{i}_{k}")
            model.add_constr(out_flow == y[i, k], name=f"flow_out_{i}_{k}")
    
    # Constraint 3: Each drone starts and ends at base (node 0)
    for k in range(num_drones):
        # Outgoing from base
        out_base = mip.xsum(x[0, j, k] for j in range(1, n + 1) if arcs[0][j])
        # Incoming to base
        in_base = mip.xsum(x[i, 0, k] for i in range(1, n + 1) if arcs[i][0])
        # If drone k visits any point, it must leave and return to base exactly once
        # Otherwise, it doesn't move at all
        model.add_constr(out_base == in_base, name=f"base_balance_{k}")
        model.add_constr(out_base <= 1, name=f"base_out_{k}")
    
    # Constraint 4: Calculate total time for each drone
    for k in range(num_drones):
        total_time = mip.xsum(
            travel_time[i][j] * x[i, j, k]
            for i in range(n + 1)
            for j in range(n + 1)
            if i != j and arcs[i][j]
        )
        model.add_constr(T[k] == total_time, name=f"time_{k}")
    
    # Constraint 5: Makespan is the maximum drone time
    for k in range(num_drones):
        model.add_constr(makespan >= T[k], name=f"makespan_{k}")
    
    # Subtour elimination (MTZ formulation)
    # u[i] represents the position in the tour
    u = {}
    for i in range(1, n + 1):
        u[i] = model.add_var(lb=1, ub=n, name=f"u_{i}")
    
    for k in range(num_drones):
        for i in range(1, n + 1):
            for j in range(1, n + 1):
                if i != j and arcs[i][j]:
                    # If arc (i,j) is used by drone k, then u[j] >= u[i] + 1
                    model.add_constr(
                        u[j] >= u[i] + 1 - n * (1 - x[i, j, k]),
                        name=f"mtz_{i}_{j}_{k}"
                    )
    
    print(f"Model created with {model.num_cols} variables and {model.num_rows} constraints")
    print("Solving...")
    
    # Set a time limit (optional, for large instances)
    model.max_seconds = 300  # 5 minutes
    
    # Solve the model
    status = model.optimize()
    
    if status == mip.OptimizationStatus.OPTIMAL or status == mip.OptimizationStatus.FEASIBLE:
        print(f"\nSolution found!")
        print(f"Objective value (makespan): {model.objective_value:.2f} seconds")
        
        # Extract routes for each drone
        routes = {}
        for k in range(num_drones):
            route = [0]  # Start at base
            current = 0
            visited = {0}
            
            # Follow the path for this drone
            while True:
                next_node = None
                for j in range(n + 1):
                    if j != current and arcs[current][j]:
                        if (current, j, k) in x and x[current, j, k].x > 0.5:
                            next_node = j
                            break
                
                if next_node is None or next_node in visited:
                    break
                
                route.append(next_node)
                visited.add(next_node)
                current = next_node
                
                if current == 0:  # Returned to base
                    break
            
            # Only include drones that actually visited points
            if len(route) > 1:
                routes[k + 1] = route
        
        return routes
    else:
        print(f"No optimal solution found. Status: {status}")
        return None


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python ./main.py <csv_file>", file=sys.stderr)
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    if not csv_path.exists() or not csv_path.is_file():
        print(f"Error: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    try:
        points = read_points_from_csv(str(csv_path))
    except Exception as e:
        print(f"Error reading points from '{csv_path}': {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Loaded {len(points)} points from {csv_path}")

    # Heuristic to pick building thresholds based on filename
    name = csv_path.name.lower()
    if "1" in name or "edificio1" in name or "building1" in name:
        ENTRY_THRESHOLD = -12.5
        initial_points = INITIAL_POINT_B1
    else:
        ENTRY_THRESHOLD = -20.0
        initial_points = INITIAL_POINT_B2

    entry_points = [p for p in points if p.y <= ENTRY_THRESHOLD]
    print(f"Initial point chosen: {initial_points}")
    print(f"Entry points (y <= {ENTRY_THRESHOLD}): {len(entry_points)}")
    
    # Solve the drone routing problem
    solution = solve_drone_routing(
        points=points,
        base_point=initial_points,
        entry_points=entry_points,
        num_drones=K,
        speed_horizontal=SPEED_HORIZONTAL,
        speed_up=SPEED_UP,
        speed_down=SPEED_DOWN,
    )
    
    # Print solution in required format
    if solution:
        print("\n=== Solution ===")
        for drone_id, route in solution.items():
            route_str = "-".join(str(idx) for idx in route)
            print(f"Drone {drone_id}: {route_str}")
    else:
        print("No solution found!")
        sys.exit(1)
