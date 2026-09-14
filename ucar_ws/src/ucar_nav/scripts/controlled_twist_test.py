#!/usr/bin/env python3
"""Publish one short, single-axis Twist command and always finish with zero."""

import argparse
import math


MAX_TRANSLATION = 0.10
MAX_ROTATION = 0.25
MAX_DURATION = 2.0


def validate_command(x, y, yaw, duration):
    values = (x, y, yaw, duration)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("all values must be finite")
    if not 0.0 < duration <= MAX_DURATION:
        raise ValueError("duration must be in (0, 2.0] seconds")
    if abs(x) > MAX_TRANSLATION or abs(y) > MAX_TRANSLATION:
        raise ValueError("translation is limited to 0.10 m/s")
    if abs(yaw) > MAX_ROTATION:
        raise ValueError("rotation is limited to 0.25 rad/s")
    if sum(abs(value) > 1e-9 for value in (x, y, yaw)) != 1:
        raise ValueError("exactly one motion axis must be non-zero")
    return x, y, yaw, duration


def publish_cycles(duration, rate_hz):
    return max(1, int(round(duration * rate_hz)))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    x, y, yaw, duration = validate_command(
        args.x, args.y, args.yaw, args.duration
    )

    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node("controlled_twist_test", anonymous=True)
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    command = Twist()
    command.linear.x = x
    command.linear.y = y
    command.angular.z = yaw
    zero = Twist()

    try:
        rospy.sleep(0.25)
        for _ in range(publish_cycles(duration, rate_hz)):
            if rospy.is_shutdown():
                break
            publisher.publish(command)
            rate.sleep()
    finally:
        for _ in range(5):
            publisher.publish(zero)
            rospy.sleep(0.05)


if __name__ == "__main__":
    main()
