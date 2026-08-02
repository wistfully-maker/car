#!/usr/bin/env python3
"""Extract likely wall-contact intervals from a ROS navigation bag."""

import argparse
import math

import rosbag


def sector(angle):
    degrees = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
    if -35.0 <= degrees <= 35.0:
        return "front"
    if 55.0 <= degrees <= 125.0:
        return "left"
    if -125.0 <= degrees <= -55.0:
        return "right"
    if degrees >= 145.0 or degrees <= -145.0:
        return "back"
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--bin", type=float, default=0.5)
    parser.add_argument("--near", type=float, default=0.24)
    args = parser.parse_args()

    bins = {}
    start = None
    path_frames = {}

    with rosbag.Bag(args.bag) as bag:
        for topic, msg, stamp in bag.read_messages():
            seconds = stamp.to_sec()
            if start is None:
                start = seconds
            index = int((seconds - start) / args.bin)
            row = bins.setdefault(
                index,
                {
                    "front": float("inf"),
                    "left": float("inf"),
                    "right": float("inf"),
                    "back": float("inf"),
                    "vx": 0.0,
                    "vy": 0.0,
                    "wz": 0.0,
                },
            )

            if topic == "/scan":
                angle = msg.angle_min
                for distance in msg.ranges:
                    target = sector(angle)
                    if (
                        target
                        and math.isfinite(distance)
                        and msg.range_min <= distance <= msg.range_max
                    ):
                        row[target] = min(row[target], distance)
                    angle += msg.angle_increment
            elif topic == "/cmd_vel":
                row["vx"] = msg.linear.x
                row["vy"] = msg.linear.y
                row["wz"] = msg.angular.z
            elif topic.endswith("plan"):
                path_frames.setdefault(topic, set()).add(msg.header.frame_id)

    print("start={:.6f}".format(start))
    print("path_frames:")
    for topic in sorted(path_frames):
        print("  {}: {}".format(topic, sorted(path_frames[topic])))
    print("candidate_near_wall_bins:")
    candidates = 0
    for index in sorted(bins):
        row = bins[index]
        moving = math.hypot(row["vx"], row["vy"]) > 0.015 or abs(row["wz"]) > 0.08
        nearest = min(row[name] for name in ("front", "left", "right", "back"))
        if moving and nearest < args.near:
            candidates += 1
            print(
                "  t={:5.1f}-{:5.1f}s front={:.3f} left={:.3f} right={:.3f} "
                "back={:.3f} cmd=({:+.3f},{:+.3f},{:+.3f})".format(
                    index * args.bin,
                    (index + 1) * args.bin,
                    row["front"],
                    row["left"],
                    row["right"],
                    row["back"],
                    row["vx"],
                    row["vy"],
                    row["wz"],
                )
            )
    print("candidate_count={}".format(candidates))


if __name__ == "__main__":
    main()
