#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
for robot in robot1 robot2 robot3; do
  echo
  echo "===== ${robot} ====="
  ros2 lifecycle get "/${robot}/amcl" || true
  ros2 lifecycle get "/${robot}/controller_server" || true
  ros2 lifecycle get "/${robot}/planner_server" || true
done
