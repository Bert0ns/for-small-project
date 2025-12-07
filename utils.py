from dataclasses import dataclass
from typing import Tuple
import math
import csv
from typing import Iterable, Iterator


@dataclass
class Point3D:
    """
    Class that represents a point in 3D space.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_tuple(self) -> Tuple[float, float, float]:
        """Convert the point to a tuple.

        Returns:
            Tuple[float, float, float]: the point as a tuple (x, y, z)
        """
        return (self.x, self.y, self.z)

    def distance_to(self, other: "Point3D") -> float:
        """Euclidean distance between points in space

        Args:
            other (Point3D): the other point
        Returns:
            float: the Euclidean distance between the two points
        """
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def __add__(self, other: "Point3D") -> "Point3D":
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Point3D") -> "Point3D":
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)


def read_points_from_csv(path: str, delimiter: str = ",", skip_header: bool = True):
    """Read a CSV file and return a list of Point3D objects.

    Expected CSV layout: x,y,z (floats). Empty lines are ignored.
    If skip_header is True the first row will be skipped (useful for a header).
    """
    points = []

    def to_float(s: str) -> float:
        try:
            return float(s)
        except Exception as exc:
            raise ValueError(f"Cannot convert '{s}' to float") from exc

    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh, delimiter=delimiter)

        # Filter out comments and empty lines before processing
        rows = (
            row
            for row in reader
            if row
            and any(cell.strip() for cell in row)
            and not row[0].strip().startswith("#")
        )

        if skip_header:
            # consume header if present
            try:
                next(rows)
            except StopIteration:
                return points

        for row in rows:
            # tolerate rows with >=3 columns; missing columns -> error
            if len(row) < 3:
                raise ValueError(f"CSV row has fewer than 3 columns: {row}")
            x = to_float(row[0])
            y = to_float(row[1])
            z = to_float(row[2])
            # Point3D is defined later in the file; constructing here is fine when called after class definition
            points.append(Point3D(x, y, z))
    return points


def calc_time_for_movement(
    speed_lateral: float,
    speed_vertical: float,
    length_horizontal: float,
    length_vertical: float,
) -> float:
    """Compute the time to travel a cerain distance using the function provided by the exercise text.

    Args:
        speed_lateral (float): lateral speed
        speed_vertical (float): vertical speed
        length_horizontal (float): horizontal travelled distance
        length_vertical (float): vertical travelled distance

    Returns:
        float: time to travel the distance in seconds
    """
    time_horizontal = length_horizontal / speed_lateral
    time_vertical = length_vertical / speed_vertical
    return max(time_horizontal, time_vertical)


def calc_time_between_points(
    point_a: Point3D,
    point_b: Point3D,
    speed_lateral: float,
    speed_up: float,
    speed_down: float,
) -> float:
    """Compute the time to travel between 2 points, using the method provided by the exercise text.

    Args:
        point_a (Point3D): starting point
        point_b (Point3D): ending point
        speed_lateral (float): lateral speed
        speed_up (float): speed when moving up
        speed_down (float): speed when moving down
    Returns:
        float: time to travel between the two points in seconds
    """
    delta = point_b - point_a
    length_horizontal = math.sqrt(delta.x * delta.x + delta.y * delta.y)
    length_vertical = abs(delta.z)

    if delta.z > 0:
        speed_vertical = speed_up
    else:
        speed_vertical = speed_down

    return calc_time_for_movement(
        speed_lateral, speed_vertical, length_horizontal, length_vertical
    )
