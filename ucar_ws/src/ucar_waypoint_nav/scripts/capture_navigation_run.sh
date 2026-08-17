#!/usr/bin/env bash

set -eu

OUTPUT_ROOT="${1:-/home/ucar/navigation_runs}"
STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="${OUTPUT_ROOT}/${STAMP}"
PACKAGE_DIR="$(rospack find ucar_waypoint_nav)"
MAP_IMAGE="${2:-/home/ucar/ucar_ws/src/ucar_nav/maps/map.pgm}"

mkdir -p "${RUN_DIR}"
rosparam dump "${RUN_DIR}/rosparams.yaml"
rosnode list > "${RUN_DIR}/nodes.txt"
rostopic list > "${RUN_DIR}/topics.txt"
rostopic info /cmd_vel > "${RUN_DIR}/cmd_vel_info.txt" 2>&1 || true
rosrun tf tf_echo map base_link > "${RUN_DIR}/tf_map_base_link.txt" 2>&1 &
TF_PID=$!
sleep 2
kill "${TF_PID}" 2>/dev/null || true

git -C "${PACKAGE_DIR}" rev-parse HEAD > "${RUN_DIR}/git_commit.txt" 2>&1 || \
  git -C /home/ucar/ucar_ws rev-parse HEAD > "${RUN_DIR}/git_commit.txt" 2>&1 || true
sha256sum "${MAP_IMAGE}" > "${RUN_DIR}/map_sha256.txt"

rosbag record -O "${RUN_DIR}/navigation.bag" \
  /scan \
  /tf \
  /tf_static \
  /amcl_pose \
  /odom \
  /cmd_vel \
  /move_base/GlobalPlanner/plan \
  /move_base/NavfnROS/plan \
  /move_base/TebLocalPlannerROS/local_plan \
  /move_base/local_costmap/costmap \
  /move_base/local_costmap/costmap_updates \
  /ucar_waypoint_nav/state \
  /ucar_waypoint_nav/diagnostic &

echo "$!" > "${RUN_DIR}/rosbag.pid"
echo "Navigation capture started: ${RUN_DIR}"
echo "Stop capture: kill \$(cat '${RUN_DIR}/rosbag.pid')"
