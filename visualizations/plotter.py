import sys
from pathlib import Path

# Add parent directory to path to allow importing modules from there
sys.path.append(str(Path(__file__).resolve().parent.parent))

from utils import Point3D


def html_plot_generator(
    points: list[Point3D],
    arcs,
    entry_points_idx,
    name: str,
    output_file="visualizations/graph_visualization.html",
    paths=None,
):
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
        opacity=0.1,  # Reduced opacity for background arcs
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

    data = [edge_trace, normal_node_trace, entry_node_trace, base_trace]

    # Add paths if provided
    if paths:
        colors = ["cyan", "magenta", "yellow", "lime", "purple", "orange"]
        for k, path in enumerate(paths):
            path_x = []
            path_y = []
            path_z = []
            for node_idx in path:
                p = points[node_idx]
                path_x.append(p.x)
                path_y.append(p.y)
                path_z.append(p.z)

            path_trace = go.Scatter3d(
                x=path_x,
                y=path_y,
                z=path_z,
                mode="lines+markers",
                line=dict(color=colors[k % len(colors)], width=4),
                marker=dict(size=4, color=colors[k % len(colors)]),
                name=f"Drone {k+1}",
            )
            data.append(path_trace)

    fig = go.Figure(data=data)

    fig.update_layout(
        scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z"),
        margin=dict(l=0, r=0, b=0, t=0),
        title=f"3D Graph Visualization - {name}",
    )

    fig.write_html(output_file)
    print(f"3D visualization saved to {output_file}")
