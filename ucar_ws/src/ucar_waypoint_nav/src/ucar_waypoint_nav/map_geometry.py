"""Occupancy-map geometry used by offline sparse-waypoint derivation."""

from dataclasses import dataclass
import heapq
import math
import pathlib

import yaml


@dataclass(frozen=True)
class MapGeometry:
    width: int
    height: int
    resolution: float
    origin: tuple

    def world_to_pixel(self, x, y):
        column = int(round((float(x) - self.origin[0]) / self.resolution))
        row = self.height - 1 - int(
            round((float(y) - self.origin[1]) / self.resolution)
        )
        return column, row

    def pixel_to_world(self, column, row):
        return (
            self.origin[0] + int(column) * self.resolution,
            self.origin[1]
            + (self.height - 1 - int(row)) * self.resolution,
        )


@dataclass(frozen=True)
class MapData:
    geometry: MapGeometry
    occupied: list
    source_image: pathlib.Path


def _pgm_tokens(data, start=0):
    index = start
    length = len(data)
    while index < length:
        while index < length and data[index] in b" \t\r\n":
            index += 1
        if index < length and data[index] == ord("#"):
            while index < length and data[index] not in b"\r\n":
                index += 1
            continue
        if index >= length:
            return
        end = index
        while end < length and data[end] not in b" \t\r\n#":
            end += 1
        yield data[index:end], end
        index = end


def read_pgm(path):
    data = pathlib.Path(path).read_bytes()
    tokens = _pgm_tokens(data)
    magic, _ = next(tokens)
    width_token, _ = next(tokens)
    height_token, _ = next(tokens)
    maximum_token, header_end = next(tokens)
    magic = magic.decode("ascii")
    width = int(width_token)
    height = int(height_token)
    maximum = int(maximum_token)
    count = width * height

    if magic == "P2":
        values = [int(token) for token, _ in tokens]
    elif magic == "P5":
        payload_start = header_end
        if data[payload_start : payload_start + 2] == b"\r\n":
            payload_start += 2
        elif (
            payload_start < len(data)
            and data[payload_start] in b" \t\r\n"
        ):
            payload_start += 1
        else:
            raise ValueError("P5 header is missing its binary delimiter")
        if maximum > 255:
            payload = data[payload_start : payload_start + count * 2]
            values = [
                (payload[index] << 8) | payload[index + 1]
                for index in range(0, len(payload), 2)
            ]
        else:
            values = list(data[payload_start : payload_start + count])
    else:
        raise ValueError("unsupported PGM format: {}".format(magic))

    if len(values) != count:
        raise ValueError(
            "PGM pixel count mismatch: expected {}, got {}".format(
                count, len(values)
            )
        )
    rows = [
        values[index : index + width]
        for index in range(0, count, width)
    ]
    return width, height, maximum, rows


def load_map(yaml_path):
    yaml_path = pathlib.Path(yaml_path)
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = yaml_path.parent / metadata["image"]
    width, height, maximum, pixels = read_pgm(image_path)
    negate = bool(int(metadata.get("negate", 0)))
    occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    free_threshold = float(metadata.get("free_thresh", 0.196))
    occupied = []
    for row in pixels:
        output_row = []
        for value in row:
            normalized = float(value) / maximum
            occupancy = normalized if negate else 1.0 - normalized
            output_row.append(
                occupancy >= occupied_threshold
                or free_threshold < occupancy < occupied_threshold
            )
        occupied.append(output_row)
    geometry = MapGeometry(
        width=width,
        height=height,
        resolution=float(metadata["resolution"]),
        origin=(
            float(metadata["origin"][0]),
            float(metadata["origin"][1]),
        ),
    )
    return MapData(geometry, occupied, image_path.resolve())


def inflate_obstacles(occupied, radius_cells):
    height = len(occupied)
    width = len(occupied[0]) if height else 0
    radius = max(0, int(radius_cells))
    offsets = [
        (dx, dy)
        for dy in range(-radius, radius + 1)
        for dx in range(-radius, radius + 1)
        if dx * dx + dy * dy <= radius * radius
    ]
    inflated = [row[:] for row in occupied]
    for row in range(height):
        for column in range(width):
            if not occupied[row][column]:
                continue
            for dx, dy in offsets:
                target_column = column + dx
                target_row = row + dy
                if 0 <= target_column < width and 0 <= target_row < height:
                    inflated[target_row][target_column] = True
    return inflated


def _neighbors(occupied, point):
    column, row = point
    height = len(occupied)
    width = len(occupied[0]) if height else 0
    for dx, dy in (
        (-1, -1),
        (0, -1),
        (1, -1),
        (-1, 0),
        (1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
    ):
        next_column = column + dx
        next_row = row + dy
        if not (0 <= next_column < width and 0 <= next_row < height):
            continue
        if occupied[next_row][next_column]:
            continue
        if dx and dy:
            if (
                occupied[row][next_column]
                or occupied[next_row][column]
            ):
                continue
        yield (next_column, next_row), math.hypot(dx, dy)


def astar(occupied, start, goal):
    width = len(occupied[0]) if occupied else 0
    height = len(occupied)
    for name, point in (("start", start), ("goal", goal)):
        column, row = point
        if not (0 <= column < width and 0 <= row < height):
            raise ValueError("{} is outside the map".format(name))
        if occupied[row][column]:
            raise ValueError("{} is occupied".format(name))

    queue = [(0.0, start)]
    costs = {start: 0.0}
    parents = {}
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            path = [current]
            while current in parents:
                current = parents[current]
                path.append(current)
            return list(reversed(path))
        for neighbor, step_cost in _neighbors(occupied, current):
            candidate = costs[current] + step_cost
            if candidate >= costs.get(neighbor, float("inf")):
                continue
            costs[neighbor] = candidate
            parents[neighbor] = current
            heuristic = math.hypot(
                goal[0] - neighbor[0], goal[1] - neighbor[1]
            )
            heapq.heappush(queue, (candidate + heuristic, neighbor))
    raise ValueError("no free path between start and goal")


def clearance_map(occupied):
    height = len(occupied)
    width = len(occupied[0]) if height else 0
    distances = [[float("inf")] * width for _ in range(height)]
    queue = []
    for row in range(height):
        for column in range(width):
            if occupied[row][column]:
                distances[row][column] = 0.0
                heapq.heappush(queue, (0.0, column, row))
    if not queue:
        return [[float("inf")] * width for _ in range(height)]
    while queue:
        distance, column, row = heapq.heappop(queue)
        if distance > distances[row][column]:
            continue
        for dx, dy in (
            (-1, -1),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ):
            next_column = column + dx
            next_row = row + dy
            if not (
                0 <= next_column < width and 0 <= next_row < height
            ):
                continue
            candidate = distance + math.hypot(dx, dy)
            if candidate >= distances[next_row][next_column]:
                continue
            distances[next_row][next_column] = candidate
            heapq.heappush(
                queue, (candidate, next_column, next_row)
            )
    return distances
