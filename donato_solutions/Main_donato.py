import math
import csv
import sys
import os
import numpy as np
from sklearn.cluster import KMeans

# [cite_start]Importiamo mip come richiesto dalla specifica [cite: 36]
try:
    import mip
except ImportError:
    print("Errore: Libreria 'mip' non installata. Esegui: pip install mip")
    sys.exit(1)

# --- 1. CLASSI E FUNZIONI DI UTILITÀ ---

class Point3D:
    def __init__(self, x, y, z, original_id):
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.id = original_id 
    
    def distance_to(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)

def calc_travel_time(p1, p2):
    """
    Calcola il tempo di viaggio tra due punti basandosi sulle specifiche.
    [cite_start]Orizzontale: 1.5 m/s, Salita: 1 m/s, Discesa: 2 m/s [cite: 11-17]
    """
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dz = p2.z - p1.z
    dist_h = math.sqrt(dx**2 + dy**2)
    dist_v = abs(dz)
    
    time_h = dist_h / 1.5
    if dz > 0: time_v = dist_v / 1.0
    else:      time_v = dist_v / 2.0
    
    # Il tempo è il massimo tra le componenti (movimento simultaneo)
    return max(time_h, time_v)

def read_points(filename):
    points = []
    print(f"Lettura file {filename}...", flush=True)
    if not os.path.exists(filename):
        print(f"Errore: File {filename} non trovato.")
        sys.exit(1)

    with open(filename, 'r') as f:
        # Analisi euristica del formato CSV (gestione header e separatori)
        sample = f.read(1024)
        f.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample)
            has_header = csv.Sniffer().has_header(sample)
        except:
            dialect = csv.excel
            has_header = False

        reader = csv.reader(f, dialect)
        if has_header: next(reader, None)

        idx = 1
        for row in reader:
            if not row or len(row) < 3: continue
            try:
                # Gestione caso separatore errato (es. ; in unica colonna)
                if len(row) == 1 and ';' in row[0]:
                    row = row[0].split(';')
                
                points.append(Point3D(row[0], row[1], row[2], idx))
                idx += 1
            except ValueError: continue
            
    print(f"Letti {len(points)} punti.", flush=True)
    return points

# --- 2. MODELLO MIP PER CLUSTERING BILANCIATO ---

def assign_points_mip(points, k_drones=4, time_limit=300):
    """
    Usa MIP per assegnare punti ai droni minimizzando la distanza dai centroidi,
    con vincoli rigidi sul bilanciamento del carico.
    """
    n = len(points)
    print("Inizializzazione K-Means per centroidi di riferimento...", flush=True)
    
    # Usiamo K-Means solo per ottenere centroidi iniziali validi geometricamente
    coords = np.array([[p.x, p.y, p.z] for p in points])
    kmeans = KMeans(n_clusters=k_drones, random_state=42, n_init=10).fit(coords)
    centers = kmeans.cluster_centers_

    print("Costruzione Modello MIP...", flush=True)
    # solver_name='CBC' è lo standard open source incluso in mip
    m = mip.Model(sense=mip.MINIMIZE, solver_name=mip.CBC)
    
    # --- FIX CRASH WINDOWS ---
    # Impostiamo threads a 1 per evitare conflitti di memoria su Windows
    m.threads = 1 
    m.verbose = 1 # Vediamo l'output del solver per capire se lavora

    # Variabili Binarie x[i][k] = 1 se il punto i va al drone k
    # Creazione efficiente delle variabili
    x = [[m.add_var(var_type=mip.BINARY) for k in range(k_drones)] for i in range(n)]

    # Obiettivo: Minimizzare distanza totale assegnata
    # Pre-calcoliamo le distanze punto-centroide per l'obiettivo
    # (Questo approssima il clustering spaziale)
    print("Calcolo matrice costi...", flush=True)
    obj_expr = 0
    for i in range(n):
        for k in range(k_drones):
            dist = math.sqrt(sum((coords[i] - centers[k])**2))
            obj_expr += x[i][k] * dist
    m.objective = obj_expr

    # Vincolo 1: Ogni punto deve essere assegnato a un solo drone
    for i in range(n):
        m += mip.xsum(x[i][k] for k in range(k_drones)) == 1

    # Vincolo 2: Bilanciamento del carico
    # Calcoliamo il range accettabile di punti per drone
    avg = n / k_drones
    tolerance = 0.20 # 20% di flessibilità
    min_pts = int(avg * (1 - tolerance))
    max_pts = int(avg * (1 + tolerance))
    
    print(f"Vincolo Bilanciamento: ogni drone deve avere tra {min_pts} e {max_pts} punti.", flush=True)
    
    for k in range(k_drones):
        drone_load = mip.xsum(x[i][k] for i in range(n))
        m += drone_load >= min_pts
        m += drone_load <= max_pts

    print(f"Avvio Solver MIP (Timeout impostato a {time_limit}s)...", flush=True)
    status = m.optimize(max_seconds=time_limit)

    assignments = [0] * n
    
    # Se il MIP trova una soluzione (Ottima o Feasible)
    if status in [mip.OptimizationStatus.OPTIMAL, mip.OptimizationStatus.FEASIBLE]:
        print("Soluzione MIP trovata!", flush=True)
        for i in range(n):
            for k in range(k_drones):
                if x[i][k].x >= 0.99:
                    assignments[i] = k
                    break
    else:
        print("MIP fallito o timeout. Fallback su K-Means standard.", flush=True)
        return kmeans.labels_ # Fallback di sicurezza

    return assignments

# --- 3. ROUTING (TSP EURISTICO) ---

def solve_greedy_route(base_node, unvisited_nodes, entry_threshold):
    """Nearest Neighbor con gestione Entry Points"""
    path = [base_node]
    current = base_node
    candidates = list(unvisited_nodes) 

    while candidates:
        best_next = None
        min_cost = float('inf')
        
        # Cerca il candidato raggiungibile più vicino
        for cand in candidates:
            connected = False
            
            # [cite_start]Regole di connettività [cite: 23-26]
            # 1. Base <-> Entry Points (y <= threshold)
            is_base_move = (current.id == 0 or cand.id == 0)
            if is_base_move:
                target_y = cand.y if current.id == 0 else current.y
                if target_y <= entry_threshold:
                    connected = True
            
            # 2. Griglia <-> Griglia
            else:
                d = current.distance_to(cand)
                if d <= 4.0: 
                    connected = True
                elif d <= 11.0:
                    # Distanza <= 11 e due coordinate differiscono di max 0.5
                    diffs = [abs(current.x - cand.x), abs(current.y - cand.y), abs(current.z - cand.z)]
                    if sum(1 for df in diffs if df <= 0.5) >= 2: 
                        connected = True
            
            if connected:
                cost = calc_travel_time(current, cand)
                if cost < min_cost:
                    min_cost = cost
                    best_next = cand
        
        if best_next:
            path.append(best_next)
            candidates.remove(best_next)
            current = best_next
        else:
            # Vicolo cieco: torna alla base
            path.append(base_node)
            current = base_node
            
            # Se siamo alla base e non troviamo nessuno, forza il prossimo (teletrasporto penalizzato)
            # Questo serve a garantire che tutti i punti vengano visitati anche in casi limite
            if current.id == 0 and candidates:
                forced = candidates.pop(0)
                path.append(forced)
                current = forced

    path.append(base_node)
    return path

def optimize_2opt(path, max_iter=30):
    """Migliora il percorso scambiando archi incrociati"""
    best_path = path[:]
    improved = True
    it = 0
    while improved and it < max_iter:
        improved = False
        it += 1
        for i in range(1, len(best_path) - 2):
            for j in range(i + 1, len(best_path) - 1):
                if j - i == 1: continue
                
                p_prev, p_i = best_path[i-1], best_path[i]
                p_j, p_next = best_path[j], best_path[j+1]
                
                # Ottimizzazione geometrica: non provare se lontani
                if p_prev.distance_to(p_j) > 15.0: continue
                
                current_cost = calc_travel_time(p_prev, p_i) + calc_travel_time(p_j, p_next)
                new_cost = calc_travel_time(p_prev, p_j) + calc_travel_time(p_i, p_next)
                
                if new_cost < current_cost - 0.001:
                    best_path = best_path[:i] + best_path[i:j+1][::-1] + best_path[j+1:]
                    improved = True
                    break 
            if improved: break
    return best_path

def format_output_string(path_objs):
    """Pulisce la stringa di output (es. rimuove 0-0-0)"""
    if not path_objs: return "0-0"
    ids = [p.id for p in path_objs]
    
    # Rimuovi duplicati consecutivi
    clean_ids = [ids[0]]
    for x in ids[1:]:
        if x != clean_ids[-1]:
            clean_ids.append(x)
            
    # Assicura start/end a 0
    if clean_ids[0] != 0: clean_ids.insert(0, 0)
    if clean_ids[-1] != 0: clean_ids.append(0)
    
    return "-".join(map(str, clean_ids))

# --- 4. MAIN ---

# --- 4. MAIN ---

def main():
    if len(sys.argv) < 2:
        print("Uso: python main.py <file.csv>")
        sys.exit(1)

    input_file = sys.argv[1]

    # Logica standard richiesta
    if "1" in input_file:
        base_coords = (0, -16, 0); entry_thresh = -12.5
    else:
        base_coords = (0, -40, 0); entry_thresh = -20.0

    points = read_points(input_file)
    if not points: return

    # MIP con gestione intelligente
    assignments = assign_points_mip(points, k_drones=4, time_limit=240)

    clusters = {0:[], 1:[], 2:[], 3:[]}
    for i, label in enumerate(assignments):
        clusters[label].append(points[i])

    # Routing e Stampa
    for k in range(4):
        target = clusters[k]
        base_node = Point3D(*base_coords, 0)

        if not target:
            print(f"Drone {k+1}: 0-0")
        else:
            path = solve_greedy_route(base_node, target, entry_thresh)
            if len(path) < 1500:
                path = optimize_2opt(path, max_iter=25)
            print(f"Drone {k+1}: {format_output_string(path)}")

if __name__ == "__main__":
    main()