"""Grid-based collision checking for an in-place footprint rotation."""

from dataclasses import dataclass
import math

from .corner_geometry import normalize_angle


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: list


@dataclass(frozen=True)
class SweepResult:
    safe: bool
    blocking_cell: tuple = None
    reason: str = ""


def _transform_polygon(footprint, x, y, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (
            x + point[0] * cosine - point[1] * sine,
            y + point[0] * sine + point[1] * cosine,
        )
        for point in footprint
    ]


def _point_in_polygon(point, polygon):
    inside = False
    x, y = point
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) < 1.0e-10 and min(x1, x2) <= x <= max(
            x1, x2
        ) and min(y1, y2) <= y <= max(y1, y2):
            return True
        if (y1 > y) != (y2 > y):
            intersection_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < intersection_x:
                inside = not inside
        previous = current
    return inside


def _polygon_cells(grid, polygon):
    minimum_x = min(point[0] for point in polygon)
    maximum_x = max(point[0] for point in polygon)
    minimum_y = min(point[1] for point in polygon)
    maximum_y = max(point[1] for point in polygon)
    first_column = int(math.floor((minimum_x - grid.origin_x) / grid.resolution))
    last_column = int(math.floor((maximum_x - grid.origin_x) / grid.resolution))
    first_row = int(math.floor((minimum_y - grid.origin_y) / grid.resolution))
    last_row = int(math.floor((maximum_y - grid.origin_y) / grid.resolution))
    if (
        first_column < 0
        or first_row < 0
        or last_column >= grid.width
        or last_row >= grid.height
    ):
        return None
    cells = []
    for row in range(first_row, last_row + 1):
        center_y = grid.origin_y + (row + 0.5) * grid.resolution
        for column in range(first_column, last_column + 1):
            center_x = grid.origin_x + (column + 0.5) * grid.resolution
            if _point_in_polygon((center_x, center_y), polygon):
                cells.append((column, row))
    return cells


def check_rotation_sweep(
    grid,
    pose,
    target_yaw,
    footprint,
    angle_step,
    lethal_threshold,
):
    if angle_step <= 0.0:
        raise ValueError("angle_step must be positive")
    x, y, start_yaw = pose
    delta = normalize_angle(target_yaw - start_yaw)
    sample_count = max(1, int(math.ceil(abs(delta) / angle_step)))
    for sample in range(sample_count + 1):
        yaw = start_yaw + delta * sample / sample_count
        polygon = _transform_polygon(footprint, x, y, yaw)
        cells = _polygon_cells(grid, polygon)
        if cells is None:
            return SweepResult(False, reason="footprint_outside_grid")
        for column, row in cells:
            cost = grid.data[row * grid.width + column]
            if cost < 0:
                return SweepResult(
                    False, (column, row), "unknown_cost"
                )
            if cost >= lethal_threshold:
                return SweepResult(
                    False, (column, row), "lethal_cost"
                )
    return SweepResult(True)
