"""Derive sparse topological pass-through waypoints from a grid path."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DerivedWaypoint:
    column: int
    row: int
    path_index: int
    yaw: float
    clearance: float
    turn_angle: float
    kind: str = "pass_through"


def _normalize(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _heading(first, second):
    return math.atan2(second[1] - first[1], second[0] - first[0])


def _cluster_candidates(candidates):
    if not candidates:
        return []
    clusters = [[candidates[0]]]
    for candidate in candidates[1:]:
        if candidate[0] <= clusters[-1][-1][0] + 1:
            clusters[-1].append(candidate)
        else:
            clusters.append([candidate])
    return clusters


def _evenly_limit(points, maximum_count):
    if len(points) <= maximum_count:
        return points
    if maximum_count <= 1:
        return [max(points, key=lambda point: abs(point.turn_angle))]
    selected = []
    for position in range(maximum_count):
        source_index = int(
            round(position * (len(points) - 1) / (maximum_count - 1))
        )
        point = points[source_index]
        if point not in selected:
            selected.append(point)
    return selected


def derive_sparse_waypoints(
    path,
    clearance,
    resolution,
    direction_window,
    minimum_turn,
    minimum_spacing,
    maximum_count,
    terminal_spacing=0.60,
):
    if len(path) < 3 or maximum_count <= 0:
        return []
    window = max(1, int(round(direction_window / resolution)))
    candidates = []
    for index in range(window, len(path) - window):
        incoming = _heading(path[index - window], path[index])
        outgoing = _heading(path[index], path[index + window])
        turn = _normalize(outgoing - incoming)
        if abs(turn) >= minimum_turn:
            candidates.append((index, turn, outgoing))

    points = []
    search_start = max(1, int(round(0.20 / resolution)))
    search_end = max(search_start, int(round(0.50 / resolution)))
    for cluster in _cluster_candidates(candidates):
        apex_index, turn, outgoing = max(
            cluster, key=lambda candidate: abs(candidate[1])
        )
        first = min(len(path) - 2, apex_index + search_start)
        last = min(len(path) - 2, apex_index + search_end)
        if first > last:
            continue
        best_index = max(
            range(first, last + 1),
            key=lambda index: clearance[path[index][1]][path[index][0]],
        )
        column, row = path[best_index]
        point = DerivedWaypoint(
            column=column,
            row=row,
            path_index=best_index,
            yaw=outgoing,
            clearance=clearance[row][column] * resolution,
            turn_angle=turn,
        )
        if points:
            previous = points[-1]
            distance = math.hypot(
                point.column - previous.column,
                point.row - previous.row,
            ) * resolution
            if distance < minimum_spacing:
                if point.clearance > previous.clearance:
                    points[-1] = point
                continue
        points.append(point)

    points.sort(key=lambda point: point.path_index)
    remaining = [0.0] * len(path)
    for index in range(len(path) - 2, -1, -1):
        remaining[index] = remaining[index + 1] + math.hypot(
            path[index + 1][0] - path[index][0],
            path[index + 1][1] - path[index][1],
        ) * resolution
    points = [
        point
        for point in points
        if remaining[point.path_index] >= terminal_spacing
    ]
    return _evenly_limit(points, maximum_count)


def build_waypoint_document(
    map_sha256,
    derived,
    pixel_to_world,
    terminal,
):
    waypoints = []
    for index, point in enumerate(derived, start=1):
        x, y = pixel_to_world(point.column, point.row)
        waypoints.append(
            {
                "name": "pass_{}".format(index),
                "kind": "pass_through",
                "x": round(x, 6),
                "y": round(y, 6),
                "yaw": round(point.yaw, 6),
                "switch_radius": 0.35,
                "exit_radius": 0.45,
                "heading_tolerance": round(math.radians(40.0), 6),
                "minimum_pass_speed": 0.08,
                "clearance": round(point.clearance, 3),
            }
        )
    waypoints.append(
        {
            "name": "pickup_observation",
            "kind": "terminal",
            "x": float(terminal[0]),
            "y": float(terminal[1]),
            "yaw": float(terminal[2]),
            "position_tolerance": 0.15,
            "yaw_tolerance": 0.15,
            "settle_time": 0.5,
        }
    )
    return {
        "version": 1,
        "frame_id": "map",
        "map_sha256": str(map_sha256),
        "waypoints": waypoints,
    }


def write_preview_ppm(
    output_path,
    pixels,
    path,
    waypoints,
    start,
    goal,
):
    height = len(pixels)
    width = len(pixels[0]) if height else 0
    image = [
        [[value, value, value] for value in row]
        for row in pixels
    ]

    def mark(point, color, radius):
        column, row = point
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                target_column = column + dx
                target_row = row + dy
                if (
                    0 <= target_column < width
                    and 0 <= target_row < height
                    and dx * dx + dy * dy <= radius * radius
                ):
                    image[target_row][target_column] = list(color)

    for point in path:
        mark(point, (50, 120, 255), 0)
    for point in waypoints:
        mark(point, (255, 165, 0), 2)
    mark(start, (0, 200, 0), 2)
    mark(goal, (255, 0, 0), 2)

    payload = bytearray()
    for row in image:
        for red, green, blue in row:
            payload.extend((red, green, blue))
    header = "P6\n{} {}\n255\n".format(width, height).encode("ascii")
    from pathlib import Path

    Path(output_path).write_bytes(header + bytes(payload))
