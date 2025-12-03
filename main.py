"""
Drone Routing Optimization for Building Analysis

This module solves a vehicle routing problem (VRP) where k drones must visit a set of
measurement points on a building, minimizing the makespan (maximum drone time).

Problem constraints:
- All drones start and end at a base point
- Drones can only enter/exit the grid through designated entry points
- Points are connected based on distance and coordinate difference rules
- Each point must be visited exactly once
- Different speeds apply for upward, downward, and horizontal movement

Solution approach:
- Greedy constructive heuristic with load balancing
- Graph connectivity based on problem-specific distance rules
- BFS pathfinding to ensure all reachable points are visited
"""

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
    # Entry points are the only grid points accessible directly from the base
    entry_indices = set()
    for i, p in enumerate(points):
        if p in entry_points:
            entry_indices.add(i + 1)  # +1 because index 0 is base
    
    if len(entry_indices) == 0:
        print("ERROR: No entry points found! Check the threshold values.")
        return None
    
    print(f"\nBuilding connectivity graph for {n} points...")
    print(f"Entry points: {len(entry_indices)}")
    
    # Build adjacency lists and travel time matrix
    # Using dictionaries for efficient sparse graph representation
    neighbors = {}
    travel_time = {}
    
    # Base point (index 0) can connect to any entry point and vice versa
    # This is a special case - normal connectivity rules don't apply
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
    
    # Grid points connect to each other based on problem-specific connectivity rules
    # Optimization: only check points within maximum connection distance (11m) 
    # to avoid O(n²) comparisons for all pairs
    print("Building grid connectivity (this may take a while for large instances)...")
    MAX_DIST = 11.0  # Maximum connection distance from problem statement
    
    # Initialize neighbor lists for all grid points
    for i in range(1, n + 1):
        if i not in neighbors:
            neighbors[i] = []
    
    # Build undirected edges between connected grid points
    for i in range(1, n + 1):
        if i % 1000 == 0:
            print(f"  Processing point {i}/{n}...")
        
        for j in range(i + 1, n + 1):  # Only check j > i to avoid duplicates
            # Quick Euclidean distance check before expensive connectivity check
            dist = all_nodes[i].distance_to(all_nodes[j])
            if dist <= MAX_DIST and are_points_connected(all_nodes[i], all_nodes[j]):
                # Add bidirectional edge
                neighbors[i].append(j)
                neighbors[j].append(i)
                # Calculate and store travel time in both directions
                travel_ij = calc_time_between_points(
                    all_nodes[i], all_nodes[j],
                    speed_horizontal, speed_up, speed_down
                )
                travel_time[(i, j)] = travel_ij
                travel_time[(j, i)] = travel_ij
    
    print(f"Graph built successfully.")
    
    # Find all points reachable from entry points using breadth-first search (BFS)
    # Some points may be unreachable if the graph is disconnected
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
    
    # Greedy heuristic with load balancing:
    # Phase 1: Start each drone at a different entry point
    # Phase 2: Each drone builds its route using nearest neighbor until reaching target size
    # Phase 3: Remaining points are assigned to drone with minimum time (balances makespan)
    
    routes = {k: [0] for k in range(1, num_drones + 1)}
    route_times = {k: 0.0 for k in range(1, num_drones + 1)}
    unvisited = reachable - {0}  # All reachable grid points
    
    # Phase 1: Start each drone at a different entry point
    entry_list = list(entry_indices)
    for k in range(1, min(num_drones + 1, len(entry_list) + 1)):
        if k - 1 < len(entry_list):
            entry = entry_list[k - 1]
            routes[k].append(entry)
            route_times[k] = travel_time[(0, entry)]
            unvisited.discard(entry)
    
    # Phase 2: Each drone builds a route using nearest neighbor
    # Target: distribute points roughly equally among drones
    target_points_per_drone = len(unvisited) // num_drones
    
    for k in range(1, num_drones + 1):
        if len(routes[k]) == 1:  # Only has base, didn't get an entry point
            continue
            
        current = routes[k][-1]
        points_for_this_drone = 0
        
        # Build route using nearest neighbor heuristic
        while points_for_this_drone < target_points_per_drone and unvisited:
            nearest = None
            nearest_time = float('inf')
            
            # Find nearest unvisited neighbor from current position
            for candidate in unvisited:
                if candidate in neighbors.get(current, []):
                    t = travel_time.get((current, candidate), float('inf'))
                    if t < nearest_time:
                        nearest_time = t
                        nearest = candidate
            
            if nearest is None:
                # No direct neighbors available, stop for this drone
                break
            
            # Add point to route and update time
            routes[k].append(nearest)
            route_times[k] += nearest_time
            unvisited.discard(nearest)
            current = nearest
            points_for_this_drone += 1
    
    # Phase 3: Assign remaining points to balance makespan
    # Pick the drone with minimum current time and find nearest unvisited point
    while unvisited:
        # Select drone with minimum time to balance the makespan
        min_drone = min(range(1, num_drones + 1), key=lambda k: route_times[k])
        current = routes[min_drone][-1]
        
        # Use BFS to find nearest unvisited point from this drone's current position
        queue = [(current, 0, [current])]
        visited_bfs = {current}
        found = False
        
        while queue and not found:
            node, time_so_far, path = queue.pop(0)
            
            if node in unvisited:
                # Found an unvisited point - add the path to the drone's route
                for point in path[1:]:  # Skip first point (already in route)
                    if point in unvisited:
                        routes[min_drone].append(point)
                        prev = routes[min_drone][-2]
                        route_times[min_drone] += travel_time.get((prev, point), 0)
                        unvisited.discard(point)
                found = True
                break
            
            # Expand BFS frontier
            for neighbor in neighbors.get(node, []):
                if neighbor not in visited_bfs:
                    visited_bfs.add(neighbor)
                    travel_t = travel_time.get((node, neighbor), 0)
                    queue.append((neighbor, time_so_far + travel_t, path + [neighbor]))
        
        if not found:
            # No path found to any remaining points (disconnected graph)
            break
    
    if unvisited:
        print(f"Warning: {len(unvisited)} points remain unreachable")
    
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
