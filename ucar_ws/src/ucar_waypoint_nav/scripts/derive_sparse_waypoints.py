#!/usr/bin/env python3

"""Generate sparse waypoint configuration from an occupancy map."""

import argparse
import hashlib
import math
import pathlib
import sys

import yaml

from ucar_waypoint_nav.map_geometry import (
    astar,
    clearance_map,
    inflate_obstacles,
    load_map,
    read_pgm,
)
from ucar_waypoint_nav.waypoint_derivation import (
    build_waypoint_document,
    derive_sparse_waypoints,
    write_preview_ppm,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument(
        "--start", nargs=3, required=True, type=float, metavar=("X", "Y", "YAW")
    )
    parser.add_argument(
        "--goal", nargs=3, required=True, type=float, metavar=("X", "Y", "YAW")
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--preview", required=True)
    parser.add_argument("--inflation-radius", type=float, default=0.15)
    parser.add_argument("--maximum-count", type=int, default=3)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    map_data = load_map(args.map_path)
    geometry = map_data.geometry
    inflation_cells = int(
        math.ceil(args.inflation_radius / geometry.resolution)
    )
    planning_grid = inflate_obstacles(
        map_data.occupied, inflation_cells
    )
    start_pixel = geometry.world_to_pixel(args.start[0], args.start[1])
    goal_pixel = geometry.world_to_pixel(args.goal[0], args.goal[1])
    path = astar(planning_grid, start_pixel, goal_pixel)
    clearance = clearance_map(map_data.occupied)
    derived = derive_sparse_waypoints(
        path=path,
        clearance=clearance,
        resolution=geometry.resolution,
        direction_window=0.40,
        minimum_turn=math.radians(35.0),
        minimum_spacing=0.45,
        maximum_count=args.maximum_count,
    )
    digest = hashlib.sha256(
        pathlib.Path(map_data.source_image).read_bytes()
    ).hexdigest()
    document = build_waypoint_document(
        map_sha256=digest,
        derived=derived,
        pixel_to_world=geometry.pixel_to_world,
        terminal=tuple(args.goal),
    )
    output_path = pathlib.Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        yaml.safe_dump(
            document,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    _, _, _, pixels = read_pgm(map_data.source_image)
    write_preview_ppm(
        args.preview,
        pixels,
        path,
        [(point.column, point.row) for point in derived],
        start_pixel,
        goal_pixel,
    )
    print(
        "generated {} pass-through waypoints, {} path cells".format(
            len(derived), len(path)
        )
    )
    for waypoint in document["waypoints"]:
        print(
            "{name}: kind={kind} x={x:.3f} y={y:.3f} yaw={yaw:.3f}".format(
                **waypoint
            )
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
