#!/usr/bin/env bash
set -euo pipefail

output_root="${1:-$HOME/ucar_nav_diagnostics}"
stamp="$(date +%Y%m%d-%H%M%S)"
output_dir="${output_root}/${stamp}"
mkdir -p "${output_dir}"

rosnode list >"${output_dir}/nodes.txt"
rostopic list >"${output_dir}/topics.txt"
rosparam get /move_base >"${output_dir}/move_base_params.yaml"
rostopic info /scan >"${output_dir}/scan_info.txt" 2>&1 || true
rostopic info /odom >"${output_dir}/odom_info.txt" 2>&1 || true
rosnode info /move_base >"${output_dir}/move_base_info.txt" 2>&1 || true
rosrun tf tf_echo base_link laser_frame \
  >"${output_dir}/base_link_to_laser_frame.txt" 2>&1 &
tf_pid=$!
sleep 2
kill "${tf_pid}" 2>/dev/null || true
wait "${tf_pid}" 2>/dev/null || true

{
  echo "git_commit=$(git -C "$HOME/ucar_ws/src/ucar_nav" rev-parse HEAD 2>/dev/null || echo untracked)"
  echo "map_sha256=$(sha256sum "$HOME/ucar_ws/src/ucar_nav/maps/map.yaml" | awk '{print $1}')"
  echo "image_sha256=$(sha256sum "$HOME/ucar_ws/src/ucar_nav/maps/map.pgm" | awk '{print $1}')"
} >"${output_dir}/manifest.txt"

printf '%s\n' "${output_dir}"
