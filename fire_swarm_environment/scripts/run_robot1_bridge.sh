#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

ros2 launch ros_gz_bridge ros_gz_bridge.launch.py   bridge_name:=robot1_bridge   config_file:="$ROOT/config/robot1_bridge.yaml"
