from typing import Final
import sys
from pathlib import Path

# Add parent directory to path to allow importing modules from there
sys.path.append(str(Path(__file__).resolve().parent.parent))

from utils import Point3D, read_points_from_csv
from visualizations.plotter import htmpl_plot_generator
from solver import DroneRoutingSolver


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

    htmpl_plot_generator(
        points,
        arcs,
        entry_points_idx,
        str(csv_path.name) + "Before solution",
        output_file="visualizations/graph_visualization_before_solution.html",
    )

    print("Solving routing problem...")
    # paths = solver.solve(max_seconds=300000)
    print("Skipping actual solving for testing purposes.")

    paths_dummy_string = """
    Drone 1: 0-51-50-17-18-49-43-42-46-47-48-44-41-42-43-49-18-19-1-5-1-20-17-50-51-0
    Drone 2: 0-33-37-25-26-22-55-34-38-59-60-56-57-56-23-19-1-4-8-11-10-9-13-16-15-11-5-1-20-17-50-51-0
    Drone 3: 0-36-40-27-35-21-58-39-53-61-62-57-24-20-1-2-3-7-6-9-10-14-10-11-5-1-20-17-50-51-0
    Drone 4: 0-54-28-32-29-30-31-27-58-52-47-46-45-48-33-25-26-55-57-24-20-1-5-12-5-1-20-17-50-51-0
    """

    paths = []
    for line in paths_dummy_string.strip().split('\n'):
        if "Drone" in line:
            try:
                path_str = line.split(':')[1].strip()
                path = [int(node) for node in path_str.split('-')]
                paths.append(path)
            except Exception as e:
                print(f"Error parsing line: {line}. Error: {e}")
    print(paths)
    print(points)
    print(len(points))

    if paths:
        print("Routing problem solved. Generating solution plot...")
        htmpl_plot_generator(
            points,
            arcs,
            entry_points_idx,
            str(csv_path.name) + " - Solution",
            output_file="visualizations/graph_visualization_solution.html",
            paths=paths,
        )
    else:
        print("Could not print solution paths.")
