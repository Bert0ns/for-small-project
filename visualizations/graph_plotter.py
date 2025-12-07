from typing import Final
import sys
from pathlib import Path

# Add parent directory to path to allow importing modules from there
sys.path.append(str(Path(__file__).resolve().parent.parent))

from utils import Point3D, read_points_from_csv


# Number of drones
K = 4
# Speed constants (meters per second)
SPEED_UP = 1.0
SPEED_DOWN = 2.0
SPEED_HORIZONTAL = 1.5
# Initial points for each building
BASE_POINT_B1: Final[Point3D] = Point3D(0.0, -16.0, 0.0)
BASE_POINT_B2: Final[Point3D] = Point3D(0.0, -40.0, 0.0)


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
        base_point = BASE_POINT_B1
    else:
        ENTRY_THRESHOLD = -20.0
        base_point = BASE_POINT_B2

    print(f"Initial point chosen: {base_point}")

    from solver import DroneRoutingSolver

    print("Initializing solver...")
    solver = DroneRoutingSolver(
        points,
        base_point,
        ENTRY_THRESHOLD,
        k_drones=K,
        speed_up=SPEED_UP,
        speed_down=SPEED_DOWN,
        speed_horizontal=SPEED_HORIZONTAL,
    )

    points, arcs, costs, entry_points_idx = solver.get_graph()

    print(
        f"Graph has {len(points)} points and {len(arcs)} arcs. number of costs: {len(costs)}"
    )

    import plotly.graph_objects as go

    print("Generating 3D plot with Plotly...")

    # Separate points into normal and entry points
    normal_x, normal_y, normal_z = [], [], []
    entry_x, entry_y, entry_z = [], [], []

    for i, p in enumerate(points):
        if i == 0:
            continue  # Base point handled separately
        if i in entry_points_idx:
            entry_x.append(p.x)
            entry_y.append(p.y)
            entry_z.append(p.z)
        else:
            normal_x.append(p.x)
            normal_y.append(p.y)
            normal_z.append(p.z)

    # Create normal node trace
    normal_node_trace = go.Scatter3d(
        x=normal_x,
        y=normal_y,
        z=normal_z,
        mode="markers",
        marker=dict(size=3, color="red", opacity=0.8),
        name="Points",
    )

    # Create entry point trace
    entry_node_trace = go.Scatter3d(
        x=entry_x,
        y=entry_y,
        z=entry_z,
        mode="markers",
        marker=dict(size=5, color="orange", opacity=0.9),
        name="Entry Points",
    )

    # Create edge trace
    # For performance, we use a single trace with None to separate lines
    edge_x = []
    edge_y = []
    edge_z = []

    for u, v in arcs:
        p1 = points[u]
        p2 = points[v]
        edge_x.extend([p1.x, p2.x, None])
        edge_y.extend([p1.y, p2.y, None])
        edge_z.extend([p1.z, p2.z, None])

    edge_trace = go.Scatter3d(
        x=edge_x,
        y=edge_y,
        z=edge_z,
        mode="lines",
        line=dict(color="blue", width=1),
        opacity=0.3,
        name="Arcs",
    )

    # Base point trace
    base = points[0]
    base_trace = go.Scatter3d(
        x=[base.x],
        y=[base.y],
        z=[base.z],
        mode="markers",
        marker=dict(size=10, color="green", symbol="diamond"),
        name="Base",
    )

    fig = go.Figure(data=[edge_trace, normal_node_trace, entry_node_trace, base_trace])

    fig.update_layout(
        scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z"),
        margin=dict(l=0, r=0, b=0, t=0),
        title=f"3D Graph Visualization - {csv_path.name}",
    )

    output_file = "visualizations/graph_visualization.html"
    fig.write_html(output_file)
    print(f"3D visualization saved to {output_file}")
