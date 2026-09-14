#!/usr/bin/env python3
"""Send one short forward move_base goal with timeout and forced stop."""

import argparse
import math
import xml.etree.ElementTree as ET


MAX_DISTANCE = 0.50


def validate_distance(distance):
    if not math.isfinite(distance) or not 0.0 < distance <= MAX_DISTANCE:
        raise ValueError("distance must be in (0, 0.50] metres")
    return distance


def compute_relative_goal(x, y, yaw, distance):
    distance = validate_distance(distance)
    return (
        x + distance * math.cos(yaw),
        y + distance * math.sin(yaw),
        yaw,
    )


def parse_waypoint_xml(xml_text, waypoint_name):
    root = ET.fromstring(xml_text)
    fields = (
        "Pos_x", "Pos_y", "Pos_z", "Ori_x", "Ori_y", "Ori_z", "Ori_w"
    )
    for waypoint in root.findall("Waypoint"):
        if waypoint.findtext("Name") == waypoint_name:
            try:
                return tuple(float(waypoint.findtext(field)) for field in fields)
            except (TypeError, ValueError):
                raise ValueError("waypoint {} has invalid pose".format(waypoint_name))
    raise ValueError("waypoint {} was not found".format(waypoint_name))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--distance", type=float)
    target.add_argument("--waypoint-file")
    parser.add_argument("--waypoint-name", default="1")
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if not 5.0 <= args.timeout <= 60.0:
        raise ValueError("timeout must be in [5, 60] seconds")

    import actionlib
    import rospy
    import tf
    from actionlib_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

    rospy.init_node("controlled_nav_goal", anonymous=True)
    stop_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    listener = tf.TransformListener()
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        raise RuntimeError("move_base action server is unavailable")

    listener.waitForTransform(
        "map", "base_link", rospy.Time(0), rospy.Duration(5.0)
    )
    translation, rotation = listener.lookupTransform(
        "map", "base_link", rospy.Time(0)
    )
    yaw = tf.transformations.euler_from_quaternion(rotation)[2]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    if args.waypoint_file:
        with open(args.waypoint_file, "r") as waypoint_file:
            pose = parse_waypoint_xml(
                waypoint_file.read(), args.waypoint_name
            )
        goal_x, goal_y = pose[0], pose[1]
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        goal.target_pose.pose.position.z = pose[2]
        quaternion = pose[3:]
        goal_yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
    else:
        goal_x, goal_y, goal_yaw = compute_relative_goal(
            translation[0], translation[1], yaw, args.distance
        )
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        quaternion = tf.transformations.quaternion_from_euler(
            0.0, 0.0, goal_yaw
        )
    (
        goal.target_pose.pose.orientation.x,
        goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z,
        goal.target_pose.pose.orientation.w,
    ) = quaternion

    rospy.loginfo(
        "controlled goal start=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f)",
        translation[0], translation[1], yaw, goal_x, goal_y, goal_yaw
    )
    finished = False
    try:
        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(args.timeout))
        if not finished:
            client.cancel_goal()
            raise RuntimeError("controlled navigation goal timed out")
        if client.get_state() != GoalStatus.SUCCEEDED:
            raise RuntimeError(
                "move_base ended with state {}: {}".format(
                    client.get_state(), client.get_goal_status_text()
                )
            )
    finally:
        if not finished:
            client.cancel_goal()
        for _ in range(5):
            stop_publisher.publish(Twist())
            rospy.sleep(0.05)


if __name__ == "__main__":
    main()
