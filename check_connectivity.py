import sys
from pathlib import Path
from utils import Point3D, read_points_from_csv


def check_connectivity(csv_path, base_point, entry_threshold):
    print(f"Checking connectivity for {csv_path}...")
    points = read_points_from_csv(str(csv_path))
    all_points = [base_point] + points
    num_nodes = len(all_points)

    adj = {i: [] for i in range(num_nodes)}

    # Build graph
    # 1. Edges between target points
    for i in range(1, num_nodes):
        for j in range(1, num_nodes):
            if i == j:
                continue
            p_i = all_points[i]
            p_j = all_points[j]

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
                adj[i].append(j)

    # 2. Edges from base to entry points
    entry_points = []
    for i in range(1, num_nodes):
        if all_points[i].y <= entry_threshold:
            entry_points.append(i)
            adj[0].append(i)
            adj[i].append(0)

    print(f"Total nodes: {num_nodes} (1 base + {num_nodes-1} targets)")
    print(f"Entry points: {len(entry_points)}")

    # BFS from 0
    visited = set()
    queue = [0]
    visited.add(0)

    while queue:
        u = queue.pop(0)
        for v in adj[u]:
            if v not in visited:
                visited.add(v)
                queue.append(v)

    unvisited = []
    for i in range(1, num_nodes):
        if i not in visited:
            unvisited.append(i)

    if unvisited:
        print(
            f"FAIL: The graph is NOT connected. {len(unvisited)} nodes are unreachable from base."
        )
        print(f"Unreachable nodes indices: {unvisited}")
        # Check if they are connected to each other (subtours)
        subtours = []
        unvisited_set = set(unvisited)
        while unvisited_set:
            start = next(iter(unvisited_set))
            q = [start]
            component = {start}
            unvisited_set.remove(start)
            while q:
                u = q.pop(0)
                for v in adj[u]:
                    if v in unvisited_set:  # Only care about unvisited nodes
                        unvisited_set.remove(v)
                        component.add(v)
                        q.append(v)
            subtours.append(component)
        print(f"Found {len(subtours)} isolated components.")
        for idx, comp in enumerate(subtours):
            print(f"Component {idx+1}: {len(comp)} nodes. Sample: {list(comp)[:5]}")
            # Check min y in component
            min_y = min(all_points[n].y for n in comp)
            print(f"  Min Y in component: {min_y} (Threshold: {entry_threshold})")

    # Check degrees
    degrees = {i: len(adj[i]) for i in range(num_nodes)}
    leaf_nodes = [i for i, d in degrees.items() if d == 1 and i != 0]
    print(f"Nodes with degree 1 (dead ends): {len(leaf_nodes)}")
    if leaf_nodes:
        print(f"Sample dead ends: {leaf_nodes[:10]}")
        print(
            "WARNING: 'Exactly Once' constraint is IMPOSSIBLE if there are dead ends (unless they are start/end of path, but here we have loops)."
        )

    else:
        print("SUCCESS: The graph is fully connected.")


if __name__ == "__main__":
    BASE_POINT_B1 = Point3D(0.0, -16.0, 0.0)
    BASE_POINT_B2 = Point3D(0.0, -40.0, 0.0)

    check_connectivity("data/Building1.csv", BASE_POINT_B1, -12.5)
    print("-" * 20)
    check_connectivity("data/Building2.csv", BASE_POINT_B2, -20.0)
