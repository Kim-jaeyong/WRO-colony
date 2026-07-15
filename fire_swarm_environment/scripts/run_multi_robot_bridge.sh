#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

exec ros2 launch ros_gz_bridge ros_gz_bridge.launch.py   bridge_name:=multi_robot_bridge   config_file:="$ROOT/config/multi_robot_bridge.yaml"
