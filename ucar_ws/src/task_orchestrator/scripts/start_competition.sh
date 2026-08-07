#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-/home/ucar/ucar_ws/devel/setup.bash}"
SPARK_SECRET_FILE="${SPARK_SECRET_FILE:-${HOME}/.config/ucar/spark_api_password}"
BASE_DEVICE="/dev/ttyS0"
LIDAR_DEVICE="/dev/ttyS4"
CAMERA_DEVICE="/dev/video0"
SPEECH_DEVICE="/dev/ttyS3"

die() { echo "ERROR: $*" >&2; exit 2; }
warn() { echo "WARNING: $*" >&2; }

[[ -r "$ROS_SETUP" ]] || die "ROS setup is not readable: $ROS_SETUP"
[[ -r "$WORKSPACE_SETUP" ]] || die "workspace setup is not readable: $WORKSPACE_SETUP"
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WORKSPACE_SETUP"

declare -A flags=(
  [start_fast_nav]=true [start_base]=true [start_lidar]=true
  [start_camera]=true [start_fast_nav_adapter]=true
  [start_readiness_gate]=true [start_speech]=true [start_qr]=true
  [start_llm]=true [start_orchestrator]=true
  [start_velocity_arbiter]=true [start_delivery]=true
)

normalise_bool() {
  case "${2,,}" in
    true|1) flags["$1"]=true ;;
    false|0) flags["$1"]=false ;;
    *) die "invalid boolean for $1: $2" ;;
  esac
}

launch_args=("$@")
for arg in "${launch_args[@]}"; do
  if [[ "$arg" =~ ^([^:]+):=(.*)$ ]]; then
    key="${BASH_REMATCH[1]}"
    value="${BASH_REMATCH[2]}"
    case "$key" in
      start_fast_nav|start_base|start_lidar|start_camera|\
      start_fast_nav_adapter|start_readiness_gate|start_speech|start_qr|\
      start_llm|start_orchestrator|start_velocity_arbiter|start_delivery)
        normalise_bool "$key" "$value"
        ;;
      *) : ;;  # Unknown roslaunch arguments are forwarded unchanged.
    esac
  fi
done

if [[ "${flags[start_llm]}" == true && -z "${SPARK_API_PASSWORD:-}" ]]; then
  [[ ! -L "$SPARK_SECRET_FILE" ]] || die "Spark credential file must not be a symlink"
  [[ -f "$SPARK_SECRET_FILE" ]] || die "Spark credential file must be a regular file"
  [[ -r "$SPARK_SECRET_FILE" ]] || die "Spark credential file is not readable"
  # This is a data file, never shell code: one literal line, optionally prefixed.
  set +x
  if ! secret_stat="$(stat -c '%u %a' -- "$SPARK_SECRET_FILE" 2>/dev/null)"; then
    die "Spark credential file permissions could not be inspected"
  fi
  read -r secret_owner secret_mode extra_stat <<<"$secret_stat"
  [[ -z "${extra_stat:-}" && "$secret_owner" =~ ^[0-9]+$ && "$secret_mode" =~ ^[0-7]+$ ]] ||
    die "Spark credential file permissions are malformed"
  [[ "$secret_owner" == "$(id -u)" ]] || die "Spark credential file must be owned by the current user"
  (( (8#$secret_mode & 0077) == 0 )) || die "Spark credential file must not grant group or other permissions"
  (( (8#$secret_mode & 0400) != 0 )) || die "Spark credential file owner must have read permission"
  if ! secret_hex="$(LC_ALL=C od -An -tx1 -- "$SPARK_SECRET_FILE")"; then
    die "Spark credential file could not be inspected safely"
  fi
  if grep -qw 00 <<<"$secret_hex"; then
    die "Spark credential file contains a NUL byte"
  fi
  if grep -qw 0d <<<"$secret_hex"; then
    die "Spark credential file must not contain carriage returns"
  fi
  secret_lines=()
  mapfile -t secret_lines < "$SPARK_SECRET_FILE"
  [[ "${#secret_lines[@]}" -eq 1 ]] || die "Spark credential file must contain exactly one non-empty single line"
  secret_text="${secret_lines[0]}"
  if [[ "$secret_text" == SPARK_API_PASSWORD=* ]]; then
    secret_text="${secret_text#SPARK_API_PASSWORD=}"
  fi
  [[ -n "$secret_text" ]] || die "Spark credential file must contain exactly one non-empty single line"
  export SPARK_API_PASSWORD="$secret_text"
  unset secret_text secret_lines secret_hex secret_stat secret_owner secret_mode extra_stat
fi

master_available=true
if ! node_list="$(rosnode list 2>/dev/null)"; then
  master_available=false
  node_list=""
  echo "INFO: ROS master is not running; roslaunch will start it"
fi

node_registered() {
  grep -Fxq -- "$1" <<<"$node_list"
}

check_node_conflict() {
  local node="$1"
  node_registered "$node" || return 0
  if rosnode ping -c 1 "$node" >/dev/null 2>&1; then
    die "live node conflict: $node"
  fi
  die "stale ROS master registration: $node"
}

tag_probe_stderr() {
  local line=""
  while IFS= read -r line || [[ -n "$line" ]]; do
    printf '\036%s\n' "$line"
  done
}

check_device() {
  local label="$1" device="$2" probe_tool="" probe_output=""
  local probe_stdout="" probe_stderr="" probe_rc=0 line=""
  local stderr_marker=$'\036'
  if [[ ! -e "$device" ]]; then
    die "$label device is absent: $device"
  fi
  if command -v fuser >/dev/null 2>&1; then
    probe_tool=fuser
    if probe_output="$(fuser "$device" 2> >(tag_probe_stderr))"; then probe_rc=0; else probe_rc=$?; fi
  elif command -v lsof >/dev/null 2>&1; then
    probe_tool=lsof
    if probe_output="$(lsof -t -- "$device" 2> >(tag_probe_stderr))"; then probe_rc=0; else probe_rc=$?; fi
  else
    die "cannot verify $label device ownership: install fuser or lsof"
  fi
  while IFS= read -r line || [[ -n "$line" ]]; do
    [[ -z "$line" ]] && continue
    if [[ "$line" == "$stderr_marker"* ]]; then
      probe_stderr+="${line#"$stderr_marker"}"$'\n'
    else
      probe_stdout+="$line"$'\n'
    fi
  done <<<"$probe_output"
  if [[ "$probe_rc" -eq 0 && "$probe_stdout" =~ ^[[:space:]]*[0-9]+([[:space:]]+[0-9]+)*[[:space:]]*$ ]]; then
    die "$label device is busy: $device (owner PID:${probe_stdout})"
  fi
  if [[ "$probe_rc" -eq 1 && -z "$probe_stderr" && -z "$probe_stdout" ]]; then
    return 0
  fi
  die "$label device probe failure via $probe_tool (rc=$probe_rc): ${probe_stderr:-no owner PID reported}"
}

get_cmd_vel_publishers() {
  local topic_info=""
  if topic_info="$(rostopic info /cmd_vel 2>&1)"; then
    if publisher_count="$(awk '
      /^Publishers:[ \t][ \t]*None[ \t]*$/ {
        publishers++; if (publishers > 1 || subscribers) bad=1
        section="publishers"; publisher_none=1; next
      }
      /^Publishers:[ \t]*$/ {
        publishers++; if (publishers > 1 || subscribers) bad=1
        section="publishers"; next
      }
      /^Subscribers:[ \t][ \t]*None[ \t]*$/ {
        subscribers++; if (publishers != 1 || subscribers > 1) bad=1
        section="subscribers"; subscriber_none=1; next
      }
      /^Subscribers:[ \t]*$/ {
        subscribers++; if (publishers != 1 || subscribers > 1) bad=1
        section="subscribers"; next
      }
      /^[ \t]*$/ { next }
      section == "publishers" && /^None$/ {
        if (publisher_none || count) bad=1
        publisher_none=1; next
      }
      section == "publishers" && /^[ \t][ \t]*\*[ \t][ \t]*\/[-A-Za-z0-9_.\/][-A-Za-z0-9_.\/]*([ \t]|$)/ {
        if (publisher_none) bad=1
        count++; next
      }
      section == "subscribers" && /^None$/ {
        if (subscriber_none || subscriber_count) bad=1
        subscriber_none=1; next
      }
      section == "subscribers" && /^[ \t][ \t]*\*[ \t][ \t]*\/[-A-Za-z0-9_.\/][-A-Za-z0-9_.\/]*([ \t]|$)/ {
        if (subscriber_none) bad=1
        subscriber_count++; next
      }
      section != "" { bad=1 }
      END {
        if (publishers != 1 || subscribers != 1 || bad) exit 2
        print count+0
      }
    ' <<<"$topic_info")"; then
      echo "$publisher_count"
      return 0
    fi
    echo "ERROR: malformed rostopic info /cmd_vel output" >&2
    return 1
  fi
  if grep -Eqi 'unknown topic|not published' <<<"$topic_info"; then
    echo 0
    return 0
  fi
  echo "ERROR: rostopic info /cmd_vel failed: $topic_info" >&2
  return 1
}

if [[ "$master_available" == true ]]; then
  conflicts=()
  [[ "${flags[start_camera]}" == true ]] && conflicts+=(/usb_cam)
  [[ "${flags[start_speech]}" == true ]] && conflicts+=(/speech_command_node)
  [[ "${flags[start_qr]}" == true ]] && conflicts+=(/qr_scanner /item_search_controller)
  [[ "${flags[start_llm]}" == true ]] && conflicts+=(/spark_llm_node)
  if [[ "${flags[start_orchestrator]}" == true ]]; then
    conflicts+=(/task_orchestrator /voice_task_adapter /tts_bridge)
  fi
  [[ "${flags[start_fast_nav_adapter]}" == true ]] && conflicts+=(/fast_nav_adapter)
  [[ "${flags[start_readiness_gate]}" == true ]] && conflicts+=(/readiness_gate)
  [[ "${flags[start_velocity_arbiter]}" == true ]] && conflicts+=(/velocity_arbiter)
  [[ "${flags[start_delivery]}" == true ]] && conflicts+=(/vision_node /racecar_control)
  if [[ "${flags[start_fast_nav]}" == true ]]; then
    # Vendor runtime_check.sh contract; navigation_full.launch owns the includes.
    conflicts+=(/map_server /lidar_loc /move_base)
    [[ "${flags[start_base]}" == true ]] && conflicts+=(/base_driver)
    [[ "${flags[start_lidar]}" == true ]] && conflicts+=(/ydlidar_node)
  fi
  for node in "${conflicts[@]}"; do check_node_conflict "$node"; done

  uses_lidar_loc=false
  if [[ "${flags[start_fast_nav]}" == true ||
        "${flags[start_fast_nav_adapter]}" == true ||
        "${flags[start_readiness_gate]}" == true ]]; then
    uses_lidar_loc=true
  fi
  if [[ "$uses_lidar_loc" == true ]] && node_registered /amcl; then
    if rosnode ping -c 1 /amcl >/dev/null 2>&1; then
      die "/amcl is live while this flow requires /lidar_loc"
    fi
    die "stale ROS master registration for /amcl while this flow requires /lidar_loc"
  fi

  if ! publisher_count="$(get_cmd_vel_publishers)"; then
    die "cannot verify /cmd_vel ownership because ROS topic inspection failed"
  fi
  if [[ "${flags[start_velocity_arbiter]}" == true ]]; then
    [[ "$publisher_count" -eq 0 ]] || die "/cmd_vel already has a direct publisher; arbiter requires exclusive ownership"
  else
    [[ "$publisher_count" -eq 1 ]] || die "external arbiter mode requires exactly one /cmd_vel publisher (found $publisher_count)"
  fi
elif [[ "${flags[start_velocity_arbiter]}" == false ]]; then
  die "external arbiter mode requires a reachable ROS master to verify /cmd_vel ownership"
fi

if [[ "${flags[start_fast_nav]}" == true ]]; then
  if [[ "${flags[start_base]}" == true ]]; then
    check_device base "$BASE_DEVICE"
  fi
  if [[ "${flags[start_lidar]}" == true ]]; then
    check_device lidar "$LIDAR_DEVICE"
  fi
fi
if [[ "${flags[start_camera]}" == true ]]; then
  check_device camera "$CAMERA_DEVICE"
fi
if [[ "${flags[start_speech]}" == true ]]; then
  check_device speech "$SPEECH_DEVICE"
fi

echo "Preflight OK; starting competition_full.launch"
exec roslaunch task_orchestrator competition_full.launch "${launch_args[@]}"
