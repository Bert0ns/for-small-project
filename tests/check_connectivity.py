import sys
from pathlib import Path
import networkx as nx

# Add parent directory to path to allow importing modules from there
sys.path.append(str(Path(__file__).resolve().parent.parent))

from utils import Point3D, read_points_from_csv
from solver import DroneRoutingSolver


def check_connectivity(csv_path, base_point, entry_threshold):
    print(f"Checking connectivity for {csv_path}...")
    points = read_points_from_csv(str(csv_path))

    # Initialize solver to build the graph
    # We don't care about speeds or k_drones for connectivity check
    solver = DroneRoutingSolver(
        points=points, base_point=base_point, entry_threshold=entry_threshold
    )

    G = solver.graph
    all_points = solver.points
    num_nodes = solver.num_nodes

    print(f"Total nodes: {num_nodes} (1 base + {num_nodes-1} targets)")
    print(f"Entry points: {len(solver.entry_points_idx)}")

    if nx.is_strongly_connected(G):
        print("SUCCESS: The graph is fully connected.")
    else:
        # Find connected components
        components = list(nx.strongly_connected_components(G))
        # Find the component containing the base
        base_component = set()
        for comp in components:
            if 0 in comp:
                base_component = comp
                break

        unreachable_nodes = set(range(num_nodes)) - base_component
        print(
            f"FAIL: The graph is NOT connected. {len(unreachable_nodes)} nodes are unreachable from base."
        )
        print(f"Unreachable nodes indices: {list(unreachable_nodes)}")

        # Analyze isolated components
        isolated_components = [c for c in components if 0 not in c]
        print(f"Found {len(isolated_components)} isolated components.")
        for idx, comp in enumerate(isolated_components):
            print(f"Component {idx+1}: {len(comp)} nodes. Sample: {list(comp)[:5]}")
            min_y = min(all_points[n].y for n in comp)
            print(f"  Min Y in component: {min_y} (Threshold: {entry_threshold})")
    # Check degrees
    degrees = dict(G.degree())
    leaf_nodes = [i for i, d in degrees.items() if d == 1 and i != 0]
    print(f"Nodes with degree 1 (dead ends): {len(leaf_nodes)}")
    if leaf_nodes:
        print(f"Sample dead ends: {leaf_nodes[:10]}")
        print(
            "WARNING: 'Exactly Once' constraint is IMPOSSIBLE if there are dead ends (unless they are start/end of path, but here we have loops)."
        )


if __name__ == "__main__":
    BASE_POINT_B1 = Point3D(0.0, -16.0, 0.0)
    BASE_POINT_B2 = Point3D(0.0, -40.0, 0.0)

    check_connectivity("data/Building1.csv", BASE_POINT_B1, -12.5)
    print("-" * 20)
    check_connectivity("data/Building2.csv", BASE_POINT_B2, -20.0)
