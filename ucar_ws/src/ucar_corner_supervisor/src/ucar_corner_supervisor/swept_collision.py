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


def _orientation(first, second, third):
    return (second[0] - first[0]) * (third[1] - first[1]) - (
        second[1] - first[1]
    ) * (third[0] - first[0])


def _segments_intersect(first_start, first_end, second_start, second_end):
    first_side_a = _orientation(first_start, first_end, second_start)
    first_side_b = _orientation(first_start, first_end, second_end)
    second_side_a = _orientation(second_start, second_end, first_start)
    second_side_b = _orientation(second_start, second_end, first_end)
    epsilon = 1.0e-10
    if (
        first_side_a * first_side_b < -epsilon
        and second_side_a * second_side_b < -epsilon
    ):
        return True
    for point, value, start, end in (
        (second_start, first_side_a, first_start, first_end),
        (second_end, first_side_b, first_start, first_end),
        (first_start, second_side_a, second_start, second_end),
        (first_end, second_side_b, second_start, second_end),
    ):
        if (
            abs(value) <= epsilon
            and min(start[0], end[0]) - epsilon
            <= point[0]
            <= max(start[0], end[0]) + epsilon
            and min(start[1], end[1]) - epsilon
            <= point[1]
            <= max(start[1], end[1]) + epsilon
        ):
            return True
    return False


def _polygon_intersects_cell(polygon, minimum_x, minimum_y, size):
    maximum_x = minimum_x + size
    maximum_y = minimum_y + size
    cell = [
        (minimum_x, minimum_y),
        (maximum_x, minimum_y),
        (maximum_x, maximum_y),
        (minimum_x, maximum_y),
    ]
    if any(
        minimum_x <= point[0] <= maximum_x
        and minimum_y <= point[1] <= maximum_y
        for point in polygon
    ):
        return True
    if any(_point_in_polygon(point, polygon) for point in cell):
        return True
    polygon_edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    cell_edges = list(zip(cell, cell[1:] + cell[:1]))
    return any(
        _segments_intersect(first, second, third, fourth)
        for first, second in polygon_edges
        for third, fourth in cell_edges
    )


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
        for column in range(first_column, last_column + 1):
            minimum_cell_x = grid.origin_x + column * grid.resolution
            minimum_cell_y = grid.origin_y + row * grid.resolution
            if _polygon_intersects_cell(
                polygon,
                minimum_cell_x,
                minimum_cell_y,
                grid.resolution,
            ):
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
