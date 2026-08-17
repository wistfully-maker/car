#!/usr/bin/env bash
set -u

failures=0

pass() { printf 'PASS: %s\n' "$1"; }
fail() { printf 'FAIL: %s\n' "$1" >&2; failures=$((failures + 1)); }

for package in ucar_fast_nav ucar_controller ydlidar jie_ware move_base global_planner teb_local_planner; do
  if rospack find "$package" >/dev/null 2>&1; then
    pass "package $package"
  else
    fail "package $package not found"
  fi
done

for node in /base_driver /ydlidar_node /map_server /lidar_loc /move_base; do
  if rosnode list 2>/dev/null | grep -Fxq "$node"; then
    pass "node $node"
  else
    fail "node $node not running"
  fi
done

for topic in /scan /odom /map /cmd_vel; do
  if timeout 4 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
    pass "topic $topic has data"
  else
    fail "topic $topic has no data within 4 seconds"
  fi
done

for transform in 'map odom' 'odom base_link' 'base_link laser_frame'; do
  set -- $transform
  if timeout 4 rosrun tf tf_echo "$1" "$2" 2>/dev/null | grep -q 'Translation:'; then
    pass "TF $1 -> $2"
  else
    fail "TF $1 -> $2 unavailable"
  fi
done

global_planner="$(rosparam get /move_base/base_global_planner 2>/dev/null || true)"
local_planner="$(rosparam get /move_base/base_local_planner 2>/dev/null || true)"

if [ "$global_planner" = 'global_planner/GlobalPlanner' ]; then
  pass 'GlobalPlanner selected'
else
  fail "unexpected global planner: ${global_planner:-unset}"
fi

if [ "$local_planner" = 'teb_local_planner/TebLocalPlannerROS' ]; then
  pass 'TEB selected'
else
  fail "unexpected local planner: ${local_planner:-unset}"
fi

printf '\n/cmd_vel connection summary:\n'
rostopic info /cmd_vel 2>/dev/null || true

if [ "$failures" -ne 0 ]; then
  printf '\nFAILED: %d runtime checks failed. Do not send a navigation goal.\n' "$failures" >&2
  exit 1
fi

printf '\nPASS: navigation runtime chain is ready for a stationary goal test.\n'
