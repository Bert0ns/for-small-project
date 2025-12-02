from typing import Final
from utils import Point3D, read_points_from_csv
import sys
from pathlib import Path

# Path to building data files
path_building1 = "data/Building1.csv"
path_building2 = "data/Building2.csv"
# Number of drones
K = 4
# Speed constants (meters per second)
SPEED_UP = 1.0
SPEED_DOWN = 2.0
SPEED_HORIZONTAL = 1.5
# Initial points for each building
INITIAL_POINT_B1: Final[Point3D] = Point3D(0.0, -16.0, 0.0)
INITIAL_POINT_B2: Final[Point3D] = Point3D(0.0, -40.0, 0.0)

"""points_b1 = read_points_from_csv(path_building1)
    print("Points in Building 1:")
    print(points_b1)

    points_b2 = read_points_from_csv(path_building2)
    print("Points in Building 2:")
    print(points_b2)

    print("Initial Point Building 1:", INITIAL_POINT_B1)
    print("Initial Point Building 2:", INITIAL_POINT_B2)

    ENTRY_POINTS_B1: Final[list[Point3D]] = [p for p in points_b1 if p.y <= 12.5]
    ENTRY_POINTS_B2: Final[list[Point3D]] = [p for p in points_b2 if p.y <= -20.0]"""

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
        entry_threshold = 12.5
        initial_points = INITIAL_POINT_B1
    else:
        entry_threshold = -20.0
        initial_points = INITIAL_POINT_B2

    entry_points = [p for p in points if p.y <= entry_threshold]
    print(f"Initial point chosen: {initial_points}")
    print(f"Entry points (y <= {entry_threshold}): {len(entry_points)}")
    
