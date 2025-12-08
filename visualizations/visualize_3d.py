import sys
import argparse
from pathlib import Path

# Add parent directory to path to allow importing modules from there
sys.path.append(str(Path(__file__).resolve().parent.parent))

from utils import Point3D, read_points_from_csv
from solver import DroneRoutingSolver
from visualizations.plotter import htmpl_plot_generator

# Initial points for each building (copied from main.py)
BASE_POINT_B1 = Point3D(0.0, -16.0, 0.0)
BASE_POINT_B2 = Point3D(0.0, -40.0, 0.0)


def main():
    parser = argparse.ArgumentParser(
        description="Visualize 3D points and graph structure."
    )
    parser.add_argument(
        "csv_file", type=str, help="Path to the CSV file containing points."
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="visualizations/before_solution",
        help="Directory to save the output HTML file.",
    )

    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"Error: File '{csv_path}' not found.")
        sys.exit(1)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load points
    try:
        points = read_points_from_csv(str(csv_path))
        print(f"Loaded {len(points)} points from {csv_path}")
    except Exception as e:
        print(f"Error reading points: {e}")
        sys.exit(1)

    # Determine base point and threshold based on filename (heuristic from main.py)
    name = csv_path.name.lower()
    if "1" in name or "edificio1" in name or "building1" in name:
        entry_threshold = -12.5
        base_point = BASE_POINT_B1
    else:
        entry_threshold = -20.0
        base_point = BASE_POINT_B2

    print(f"Using base point: {base_point}")
    print(f"Using entry threshold: {entry_threshold}")

    # Initialize solver to build the graph
    print("Initializing solver to build graph...")
    solver = DroneRoutingSolver(
        points=points, base_point=base_point, entry_threshold=entry_threshold
    )

    # The solver builds the graph in __init__.
    # solver.points includes the base point at index 0.
    # solver.arcs and solver.entry_points_idx are populated.

    output_filename = f"{csv_path.stem}_visualization.html"
    output_path = output_dir / output_filename

    print(f"Generating visualization to {output_path}...")

    htmpl_plot_generator(
        points=solver.points,
        arcs=solver.arcs,
        entry_points_idx=solver.entry_points_idx,
        name=csv_path.stem,
        output_file=str(output_path),
    )

    print("Done.")


if __name__ == "__main__":
    main()
