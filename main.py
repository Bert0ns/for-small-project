"""
Drone Routing Optimization (Strict Physical Constraints + Revisits Allowed)

Constraints from Mathematical Model v6 respected:
1. PHYSICAL CONNECTIVITY: Dist <= 4 OR (Dist <= 11 & Coords align).
2. BASE ACCESS: Only via Entry Points.
3. MINIMAX OBJECTIVE: Balances travel time.

CHANGE:
- REVISITS ALLOWED: Drones can traverse previously visited nodes to reach
  unvisited areas or to return to base.
"""

from typing import Final, List, Dict, Set, Tuple, Optional
import sys
from pathlib import Path
import heapq  # For Dijkstra priority queue
import math
from utils import Point3D, read_points_from_csv, calc_time_between_points

# --- Constants ---
K = 4
SPEED_UP = 1.0
SPEED_DOWN = 2.0
SPEED_HORIZONTAL = 1.5
INITIAL_POINT_B1: Final[Point3D] = Point3D(0.0, -16.0, 0.0)
INITIAL_POINT_B2: Final[Point3D] = Point3D(0.0, -40.0, 0.0)

def is_connected_strict(p1: Point3D, p2: Point3D) -> bool:
    """
    Validates connection strictly according to Mathematical Model v6.
    """
    dist = p1.distance_to(p2)
    
    if dist <= 4.0:
        return True
        
    if dist <= 11.0:
        dx = abs(p1.x - p2.x)
        dy = abs(p1.y - p2.y)
        dz = abs(p1.z - p2.z)
        close_coords = (1 if dx <= 0.5 else 0) + \
                       (1 if dy <= 0.5 else 0) + \
                       (1 if dz <= 0.5 else 0)
        if close_coords >= 2:
            return True
            
    return False

def find_path_to_nearest_target(
    start_node: int, 
    targets: Set[int], 
    adj: Dict[int, List[int]], 
    cost_matrix: Dict[Tuple[int, int], float]
) -> Tuple[Optional[List[int]], float]:
    """
    Uses Dijkstra's algorithm to find the shortest time path from start_node
    to the NEAREST node present in the 'targets' set.
    Allows traversing any node in the graph (revisits).
    """
    # Priority Queue: (accumulated_time, current_node, path_list)
    pq = [(0.0, start_node, [start_node])]
    visited_in_search = set()
    
    while pq:
        time_so_far, current, path = heapq.heappop(pq)
        
        # If we hit a target (that isn't the start node itself, unless required), return
        if current in targets and current != start_node:
            return path[1:], time_so_far  # Return path excluding start, and total time
            
        if current in visited_in_search:
            continue
        visited_in_search.add(current)
        
        for neighbor in adj[current]:
            if neighbor not in visited_in_search:
                edge_cost = cost_matrix[(current, neighbor)]
                heapq.heappush(pq, (time_so_far + edge_cost, neighbor, path + [neighbor]))
                
    return None, 0.0

def solve_drone_routing(
    points: List[Point3D],
    base_point: Point3D,
    entry_points: List[Point3D],
    num_drones: int,
    speed_horizontal: float,
    speed_up: float,
    speed_down: float,
) -> Dict[int, List[int]]:
    
    n = len(points)
    all_nodes = [base_point] + points
    
    # Identify Entry Points
    entry_indices = set()
    for i, p in enumerate(points):
        if p in entry_points:
            entry_indices.add(i + 1)
            
    if not entry_indices:
        print("ERROR: No entry points found.")
        return None

    print(f"Building Connectivity Graph ({n} nodes)...")
    
    # Adjacency & Cost Matrix
    adj = {i: [] for i in range(n + 1)}
    cost_matrix = {}
    
    # 1. Base Connections
    adj[0] = list(entry_indices)
    for idx in entry_indices:
        t_out = calc_time_between_points(all_nodes[0], all_nodes[idx], speed_horizontal, speed_up, speed_down)
        t_in = calc_time_between_points(all_nodes[idx], all_nodes[0], speed_horizontal, speed_up, speed_down)
        cost_matrix[(0, idx)] = t_out
        cost_matrix[(idx, 0)] = t_in
        # Base is connected to entry, entry is connected to base
        adj[idx].append(0)

    # 2. Grid Connections (O(N^2) check)
    for i in range(1, n + 1):
        for j in range(i + 1, n + 1):
            if is_connected_strict(all_nodes[i], all_nodes[j]):
                adj[i].append(j)
                adj[j].append(i)
                
                t_ij = calc_time_between_points(all_nodes[i], all_nodes[j], speed_horizontal, speed_up, speed_down)
                t_ji = calc_time_between_points(all_nodes[j], all_nodes[i], speed_horizontal, speed_up, speed_down)
                cost_matrix[(i, j)] = t_ij
                cost_matrix[(j, i)] = t_ji

    print("Graph built. Starting Solver with Revisits Allowed...")

    # --- STATE ---
    unvisited = set(range(1, n + 1))
    routes = {k: [0] for k in range(1, num_drones + 1)}
    drone_times = {k: 0.0 for k in range(1, num_drones + 1)}
    
    # --- MAIN LOOP ---
    while unvisited:
        # 1. Pick drone with minimum time (Minimax heuristic)
        current_drone = min(routes.keys(), key=lambda k: drone_times[k])
        current_node = routes[current_drone][-1]
        
        # 2. Try simple Greedy step first (Is there a direct unvisited neighbor?)
        # This saves computation time avoiding Dijkstra if not needed.
        best_neighbor = None
        min_dist_time = float('inf')
        
        direct_neighbors = adj[current_node]
        for neighbor in direct_neighbors:
            if neighbor in unvisited:
                t = cost_matrix[(current_node, neighbor)]
                if t < min_dist_time:
                    min_dist_time = t
                    best_neighbor = neighbor
        
        if best_neighbor is not None:
            # DIRECT HIT
            routes[current_drone].append(best_neighbor)
            drone_times[current_drone] += min_dist_time
            unvisited.remove(best_neighbor)
            
        else:
            # 3. TRANSIT MODE (Revisits)
            # No direct unvisited neighbors. Find the nearest unvisited node anywhere in the graph.
            # We use Dijkstra to find the path through visited nodes.
            path_segment, added_time = find_path_to_nearest_target(
                current_node, unvisited, adj, cost_matrix
            )
            
            if path_segment:
                # Append the whole path (which may include revisited nodes)
                routes[current_drone].extend(path_segment)
                drone_times[current_drone] += added_time
                # The last node in the path is the target (unvisited)
                target_node = path_segment[-1]
                if target_node in unvisited:
                    unvisited.remove(target_node)
            else:
                # Graph disconnected or finished for this drone
                # Temporarily make this drone time infinite so we pick others
                # unless all are stuck
                if all(drone_times[d] == float('inf') for d in routes):
                    print("All drones stuck/disconnected.")
                    break
                drone_times[current_drone] = float('inf')

    # --- RETURN TO BASE ---
    print("All reachable nodes visited. Returning to base...")
    
    for k in routes:
        # Reset infinite times if any
        if drone_times[k] == float('inf'):
             # Recalculate real time
             drone_times[k] = sum(cost_matrix.get((u,v), 0) for u,v in zip(routes[k], routes[k][1:]))

        current_node = routes[k][-1]
        if current_node != 0:
            # Use Dijkstra to find path back to Base (0)
            # Target is set containing just {0}
            path_segment, added_time = find_path_to_nearest_target(
                current_node, {0}, adj, cost_matrix
            )
            
            if path_segment:
                routes[k].extend(path_segment)
                drone_times[k] += added_time
            else:
                print(f"Drone {k} CANNOT return to base from {current_node}!")

    # --- STATS ---
    print("\n=== Solution (Revisits Allowed) ===")
    max_time = max(drone_times.values())
    visited_total = n - len(unvisited)
    
    print(f"Makespan: {max_time:.2f} s")
    print(f"Coverage: {visited_total}/{n} points")
    
    return routes

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python ./main.py <csv_file>", file=sys.stderr)
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    try:
        points = read_points_from_csv(str(csv_path))
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # Dedup logic
    seen = set()
    deduped = []
    for p in points:
        key = (p.x, p.y, p.z)
        if key not in seen:
            seen.add(key)
            deduped.append(p)
    points = deduped

    # Config
    name = csv_path.name.lower()
    if "1" in name or "edificio1" in name:
        ENTRY_THRESHOLD = -12.5
        base = INITIAL_POINT_B1
    else:
        ENTRY_THRESHOLD = -20.0
        base = INITIAL_POINT_B2

    entry_points = [p for p in points if p.y <= ENTRY_THRESHOLD]

    solution = solve_drone_routing(
        points=points,
        base_point=base,
        entry_points=entry_points,
        num_drones=K,
        speed_horizontal=SPEED_HORIZONTAL,
        speed_up=SPEED_UP,
        speed_down=SPEED_DOWN,
    )

    if solution:
        for k, route in solution.items():
            route_str = "-".join(map(str, route))
            print(f"Drone {k} ({len(route)} steps): {route_str}")
