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
    Solve the drone routing optimization problem using a greedy constructive heuristic.
    
    For large instances (>100 points), this uses a practical greedy approach that ensures
    all reachable points are visited.
    
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
    
    if len(entry_indices) == 0:
        print("ERROR: No entry points found! Check the threshold values.")
        return None
    
    print(f"\nBuilding connectivity graph for {n} points...")
    print(f"Entry points: {len(entry_indices)}")
    
    # Build adjacency lists and travel time matrix for efficiency
    neighbors = {}
    travel_time = {}
    
    # Base point (index 0) can connect to any entry point and vice versa
    neighbors[0] = list(entry_indices)
    for entry_idx in entry_indices:
        time_to_entry = calc_time_between_points(
            all_nodes[0], all_nodes[entry_idx],
            speed_horizontal, speed_up, speed_down
        )
        travel_time[(0, entry_idx)] = time_to_entry
        travel_time[(entry_idx, 0)] = time_to_entry
        
        if entry_idx not in neighbors:
            neighbors[entry_idx] = []
        neighbors[entry_idx].append(0)
    
    # Grid points can connect to each other based on connectivity rules
    for i in range(1, n + 1):
        if i not in neighbors:
            neighbors[i] = []
        for j in range(1, n + 1):
            if i != j and are_points_connected(all_nodes[i], all_nodes[j]):
                neighbors[i].append(j)
                if (i, j) not in travel_time:
                    travel_time[(i, j)] = calc_time_between_points(
                        all_nodes[i], all_nodes[j],
                        speed_horizontal, speed_up, speed_down
                    )
    
    print(f"Graph built successfully.")
    
    # Find reachable points from entry points using BFS
    reachable = set(entry_indices)
    reachable.add(0)  # Base is reachable
    queue = list(entry_indices)
    
    while queue:
        current = queue.pop(0)
        for neighbor in neighbors.get(current, []):
            if neighbor not in reachable and neighbor != 0:
                reachable.add(neighbor)
                queue.append(neighbor)
    
    unreachable_count = n + 1 - len(reachable)
    if unreachable_count > 0:
        print(f"Warning: {unreachable_count} points are not reachable from entry points")
    
    print(f"Using greedy constructive heuristic for {len(reachable)-1} reachable points with {num_drones} drones...")
    
    # Greedy constructive heuristic with nearest neighbor
    routes = {k: [0] for k in range(1, num_drones + 1)}
    route_times = {k: 0.0 for k in range(1, num_drones + 1)}
    unvisited = reachable - {0}  # All reachable grid points
    drone_positions = {k: 0 for k in range(1, num_drones + 1)}
    
    # Main greedy loop
    while unvisited:
        # For each drone, find the best next point from its current position
        best_drone = None
        best_point = None
        best_time_increase = float('inf')
        
        for k in range(1, num_drones + 1):
            current_pos = drone_positions[k]
            
            # Find nearest unvisited point from current position
            for candidate in unvisited:
                if candidate in neighbors.get(current_pos, []):
                    travel = travel_time.get((current_pos, candidate), float('inf'))
                    # Consider load balancing: prefer drone with less total time
                    # But prioritize minimizing travel distance
                    time_increase = route_times[k] + travel
                    
                    if travel < best_time_increase:
                        best_time_increase = travel
                        best_drone = k
                        best_point = candidate
        
        if best_point is None:
            # No direct connection found - need to backtrack or go via entry
            # Try going back to base and entering from another entry point
            available_entries = entry_indices & unvisited
            if available_entries:
                # Pick drone with minimum time and send to an unvisited entry
                min_time_drone = min(route_times.keys(), key=lambda k: route_times[k])
                current_pos = drone_positions[min_time_drone]
                
                # Go back to base
                if current_pos != 0:
                    routes[min_time_drone].append(0)
                    route_times[min_time_drone] += travel_time.get((current_pos, 0), 0)
                    drone_positions[min_time_drone] = 0
                
                # Go to an unvisited entry point
                best_point = min(available_entries, 
                                key=lambda e: travel_time.get((0, e), float('inf')))
                best_drone = min_time_drone
            else:
                # Cannot reach remaining points
                print(f"Warning: {len(unvisited)} points remain unreachable")
                break
        
        # Assign best_point to best_drone
        if best_drone and best_point:
            current_pos = drone_positions[best_drone]
            routes[best_drone].append(best_point)
            route_times[best_drone] += travel_time.get((current_pos, best_point), 0)
            drone_positions[best_drone] = best_point
            unvisited.remove(best_point)
    
    # Close all routes: return to base
    for k in routes:
        if routes[k][-1] != 0:
            last_pos = routes[k][-1]
            return_time = travel_time.get((last_pos, 0), 0)
            routes[k].append(0)
            route_times[k] += return_time
    
    # Print solution statistics
    print(f"\nSolution found!")
    max_time = max(route_times.values())
    print(f"Makespan (max drone time): {max_time:.2f} seconds")
    total_visited = sum(len(routes[k])-2 for k in routes)  # -2 for start and end base
    print(f"Total points visited: {total_visited} / {n}")
    for k in routes:
        print(f"  Drone {k}: {len(routes[k])-2} points, time={route_times[k]:.2f}s")
    
    return routes


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
