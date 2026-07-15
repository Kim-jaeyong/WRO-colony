#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
PIDS=()
cleanup() { for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
for robot in robot1 robot2 robot3; do
  ros2 run tf2_ros static_transform_publisher     --x 0.12 --y 0.0 --z 0.20     --roll 0.0 --pitch 0.0 --yaw 0.0     --frame-id "${robot}/base_link"     --child-frame-id "${robot}/laser_frame"     --ros-args -r /tf_static:="/${robot}/tf_static" &
  PIDS+=("$!")
done
wait
