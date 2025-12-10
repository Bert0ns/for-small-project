"""
Drone Routing Optimization - Main Execution Script
Confronta Euristica vs MIP e genera doppi grafici.
"""

from typing import Final, List, Dict, Set, Tuple, Optional
import sys
from pathlib import Path
import heapq
import collections
import copy

# Import moduli custom
from utils import Point3D, read_points_from_csv, calc_time_between_points
# Assicurati che questi file esistano nella cartella:
try:
    from visualizer import generate_html_visualization
except ImportError:
    generate_html_visualization = None

try:
    from mip_solver import DroneRoutingSolverMIP
except ImportError:
    DroneRoutingSolverMIP = None
    print("ATTENZIONE: 'mip_solver.py' non trovato. La parte MIP verrà saltata.")

# --- Costanti ---
K = 4
SPEED_UP = 1.0
SPEED_DOWN = 2.0
SPEED_HORIZONTAL = 1.5
INITIAL_POINT_B1: Final[Point3D] = Point3D(0.0, -16.0, 0.0)
INITIAL_POINT_B2: Final[Point3D] = Point3D(0.0, -40.0, 0.0)

# --- Funzioni Helper (Logica Euristica & Grafi) ---

def is_connected_strict(p1: Point3D, p2: Point3D) -> bool:
    """Valida la connessione rigorosa (Modello v6)."""
    dist = p1.distance_to(p2)
    if dist <= 4.0: return True
    if dist <= 11.0:
        dx = abs(p1.x - p2.x); dy = abs(p1.y - p2.y); dz = abs(p1.z - p2.z)
        # Conta coordinate simili (diff <= 0.5)
        close_coords = (1 if dx <= 0.5 else 0) + (1 if dy <= 0.5 else 0) + (1 if dz <= 0.5 else 0)
        if close_coords >= 2: return True
    return False

def check_reachability_bfs(start_node: int, adj: Dict[int, List[int]]) -> Set[int]:
    """Trova tutti i nodi fisicamente raggiungibili."""
    visited = {start_node}
    queue = collections.deque([start_node])
    while queue:
        u = queue.popleft()
        for v in adj[u]:
            if v not in visited:
                visited.add(v)
                queue.append(v)
    return visited

def find_path_to_nearest_target(start_node: int, targets: Set[int], adj: Dict[int, List[int]], cost_matrix: Dict) -> Tuple[Optional[List[int]], float]:
    """Dijkstra per trovare il percorso verso il target più vicino."""
    if not targets: return None, 0.0
    pq = [(0.0, start_node, [start_node])]
    visited_min_cost = {}
    
    while pq:
        time_so_far, current, path = heapq.heappop(pq)
        if current in visited_min_cost and visited_min_cost[current] <= time_so_far: continue
        visited_min_cost[current] = time_so_far

        if current in targets:
            if current != start_node or len(targets) == 1:
                return path[1:], time_so_far
        
        for neighbor in adj[current]:
            new_time = time_so_far + cost_matrix[(current, neighbor)]
            if neighbor not in visited_min_cost or new_time < visited_min_cost[neighbor]:
                heapq.heappush(pq, (new_time, neighbor, path + [neighbor]))
    return None, 0.0

def solve_drone_routing_heuristic(points: List[Point3D], base_point: Point3D, entry_points: List[Point3D], num_drones: int) -> Dict[int, List[int]]:
    """Logica Greedy Euristica (quella robusta con Revisits)."""
    n = len(points)
    all_nodes = [base_point] + points
    entry_indices = {i+1 for i, p in enumerate(points) if p in entry_points}
    if not entry_indices: return None

    # 1. Build Graph
    adj = {i: [] for i in range(n + 1)}
    cost_matrix = {}
    
    # Base links
    adj[0] = list(entry_indices)
    for idx in entry_indices:
        t = calc_time_between_points(all_nodes[0], all_nodes[idx], SPEED_HORIZONTAL, SPEED_UP, SPEED_DOWN)
        cost_matrix[(0, idx)] = t; cost_matrix[(idx, 0)] = t; adj[idx].append(0)

    # Grid links
    for i in range(1, n + 1):
        for j in range(i + 1, n + 1):
            if is_connected_strict(all_nodes[i], all_nodes[j]):
                adj[i].append(j); adj[j].append(i)
                t = calc_time_between_points(all_nodes[i], all_nodes[j], SPEED_HORIZONTAL, SPEED_UP, SPEED_DOWN)
                cost_matrix[(i, j)] = t; cost_matrix[(j, i)] = t

    # 2. Reachability & Solve
    target_nodes = check_reachability_bfs(0, adj) - {0}
    routes = {k: [0] for k in range(1, num_drones + 1)}
    drone_times = {k: 0.0 for k in range(1, num_drones + 1)}
    unvisited = target_nodes.copy()

    # Loop Greedy
    while unvisited:
        current_drone = min(routes.keys(), key=lambda k: drone_times[k])
        current_node = routes[current_drone][-1]
        
        # Try direct
        best_n, min_c = None, float('inf')
        for nbr in adj[current_node]:
            if nbr in unvisited:
                c = cost_matrix[(current_node, nbr)]
                if c < min_c: min_c = c; best_n = nbr
        
        if best_n:
            routes[current_drone].append(best_n)
            drone_times[current_drone] += min_c
            unvisited.remove(best_n)
        else:
            path, t_add = find_path_to_nearest_target(current_node, unvisited, adj, cost_matrix)
            if path:
                routes[current_drone].extend(path)
                drone_times[current_drone] += t_add
                if path[-1] in unvisited: unvisited.remove(path[-1])
            else:
                drone_times[current_drone] = float('inf') # Stuck
                if all(drone_times[k] == float('inf') for k in routes): break

    # Return to Base
    for k in routes:
        if drone_times[k] == float('inf'): # Reset time for calculation
            drone_times[k] = 0 # (simplification for clean log)
        curr = routes[k][-1]
        if curr != 0:
            path, t_add = find_path_to_nearest_target(curr, {0}, adj, cost_matrix)
            if path: routes[k].extend(path)
    
    return routes

def reconstruct_path_from_arcs(arcs_list: List[Tuple[int, int]], start_node: int = 0) -> List[int]:
    """
    Converte una lista non ordinata di archi [(0,1), (2,3), (1,2)...] 
    in un percorso ordinato di nodi [0, 1, 2, 3...].
    Necessario perché MIP restituisce archi sparsi.
    """
    if not arcs_list:
        return [start_node]
        
    path = [start_node]
    # Copia per non modificare l'originale
    available_arcs = list(arcs_list)
    
    current = start_node
    while available_arcs:
        found = False
        for i, (u, v) in enumerate(available_arcs):
            if u == current:
                available_arcs.pop(i)
                path.append(v)
                current = v
                found = True
                break
            # Gestione archi bidirezionali se il MIP non è diretto
            # (Ma il nostro MIP usa indici diretti per x, quindi ok)
        
        if not found:
            # Se siamo bloccati ma ci sono ancora archi, il percorso è discontinuo
            # o c'è un errore nella soluzione MIP
            print(f"Warning: Discontinuous path reconstruction at node {current}. Remaining arcs: {len(available_arcs)}")
            break
            
    return path

# --- MAIN ---

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python ./main.py <csv_file>", file=sys.stderr); sys.exit(1)

    csv_path = Path(sys.argv[1])
    points_raw = read_points_from_csv(str(csv_path))
    
    # Dedup e Setup
    seen = set(); points = []
    for p in points_raw:
        if (p.x, p.y, p.z) not in seen:
            seen.add((p.x, p.y, p.z)); points.append(p)
            
    name = csv_path.name.lower()
    if "1" in name or "edificio1" in name:
        THRESH, base = -12.5, INITIAL_POINT_B1
    else:
        THRESH, base = -20.0, INITIAL_POINT_B2
    
    entry_points = [p for p in points if p.y <= THRESH]
    all_nodes_ordered = [base] + points
    
    # ---------------------------------------------------------
    # FASE 1: EURISTICA
    # ---------------------------------------------------------
    print(f"\n{'='*50}\nFASE 1: SOLUZIONE EURISTICA (Greedy)\n{'='*50}")
    
    heuristic_routes = solve_drone_routing_heuristic(points, base, entry_points, K)
    
    if heuristic_routes:
        print("Rotte Euristica Calcolate:")
        for k, r in heuristic_routes.items():
            print(f"Drone {k}: {r}")

        # Generazione Grafo 1 (Euristica)
        if generate_html_visualization:
            # Ricostruiamo adiacenza per sfondo
            entry_idxs = {i+1 for i, p in enumerate(points) if p.y <= THRESH}
            adj_viz = {i: [] for i in range(len(all_nodes_ordered))}
            adj_viz[0] = list(entry_idxs)
            for idx in entry_idxs: adj_viz[idx].append(0)
            for i in range(1, len(all_nodes_ordered)):
                for j in range(i+1, len(all_nodes_ordered)):
                    if is_connected_strict(all_nodes_ordered[i], all_nodes_ordered[j]):
                        adj_viz[i].append(j); adj_viz[j].append(i)

            out_heur = f"routing_result_{csv_path.stem}_HEURISTIC.html"
            generate_html_visualization(
                all_points=all_nodes_ordered,
                routes=heuristic_routes,
                entry_indices=entry_idxs,
                adj=adj_viz,
                output_filename=out_heur
            )
    else:
        print("Fallimento Euristica. Stop.")
        sys.exit(1)

    # ---------------------------------------------------------
    # FASE 2: MIP (Ottimizzazione)
    # ---------------------------------------------------------
    if DroneRoutingSolverMIP:
        print(f"\n{'='*50}\nFASE 2: SOLUZIONE MIP (Ottimizzazione)\n{'='*50}")
        
        solver = DroneRoutingSolverMIP(
            points=points, base_point=base, entry_points=entry_points,
            k_drones=K, speed_horizontal=SPEED_HORIZONTAL, 
            speed_up=SPEED_UP, speed_down=SPEED_DOWN, verbose=True
        )
        
        # Esegue MIP con Warm Start (Euristica)
        # Timeout 300s (5 min)
        mip_raw_arcs = solver.solve(max_seconds=300, heuristic_routes=heuristic_routes)
        
        if mip_raw_arcs:
            print("\nSoluzione MIP trovata (Archi grezzi convertiti in rotte):")
            
            # Conversione: Archi [(0,5), (5,10)] -> Nodi [0, 5, 10]
            mip_routes_nodes = {}
            for k, arcs in mip_raw_arcs.items():
                mip_routes_nodes[k] = reconstruct_path_from_arcs(arcs, start_node=0)
                print(f"Drone {k}: {mip_routes_nodes[k]}")
            
            # Generazione Grafo 2 (MIP)
            if generate_html_visualization:
                out_mip = f"routing_result_{csv_path.stem}_MIP_FINAL.html"
                generate_html_visualization(
                    all_points=all_nodes_ordered,
                    routes=mip_routes_nodes,
                    entry_indices=entry_idxs,
                    adj=adj_viz, # Usa lo stesso grafo di sfondo
                    output_filename=out_mip
                )
        else:
            print("MIP non ha trovato una soluzione fattibile o non ha migliorato il cutoff.")
    else:
        print("Modulo MIP non disponibile. Salto Fase 2.")
