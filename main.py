from typing import Final
import sys
from pathlib import Path
from utils import Point3D, read_points_from_csv
from visualizations.plotter import html_plot_generator


# Number of drones
K = 4
# Speed constants (meters per second)
SPEED_UP = 1.0
SPEED_DOWN = 2.0
SPEED_HORIZONTAL = 1.5
SOLVER_TIME_LIMIT = 9000  # seconds
SOLVER_MIP_GAP = 0.5  # relative gap for faster solves
WARM_START = True  # whether to use warm start or not

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

    # Check if there are duplicates while preserving input order
    seen = set()
    deduped = []
    for p in points:
        key = (p.x, p.y, p.z)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(p)

    if len(deduped) != len(points):
        print(
            "WARNING: Duplicate points found in the input data. Keeping first occurrence of each."
        )
        points = deduped
        print(f"Total unique points after removing duplicates: {len(points)}")
    else:
        print(f"Loaded {len(points)} points from {csv_path}")

    # Heuristic to pick building thresholds based on filename
    name = csv_path.name.lower()
    if "1" in name or "edificio1" in name or "building1" in name:
        ENTRY_THRESHOLD = -12.5
        base_point = BASE_POINT_B1
        solution_plot_name = (
            "visualizations/after_solution/Building1_visualization_solution.html"
        )
        before_solution_plot_name = "visualizations/before_solution/Building1_visualization_before_solution.html"
    else:
        ENTRY_THRESHOLD = -20.0
        base_point = BASE_POINT_B2
        solution_plot_name = (
            "visualizations/after_solution/Building2_visualization_solution.html"
        )
        before_solution_plot_name = "visualizations/before_solution/Building2_visualization_before_solution.html"
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
        verbose=True,
    )

    points, arcs, costs, entry_points_idx = solver.get_graph()

    print(
        f"Graph has {len(points)} points and {len(arcs)} arcs. number of costs: {len(costs)}"
    )

    print("Generating pre-solution plot...")
    html_plot_generator(
        points,
        arcs,
        entry_points_idx,
        str(csv_path.name) + "Before solution",
        output_file=before_solution_plot_name,
    )

    print("Solving routing problem...")
    paths = solver.solve(
        max_seconds=SOLVER_TIME_LIMIT, mip_gap=SOLVER_MIP_GAP, warm_start=WARM_START
    )

    if paths:
        print("Routing problem solved. Generating solution plot...")
        html_plot_generator(
            points,
            arcs,
            entry_points_idx,
            str(csv_path.name) + " - Solution",
            output_file=solution_plot_name,
            paths=paths,
        )
    else:
        print("Could not print solution paths.")
