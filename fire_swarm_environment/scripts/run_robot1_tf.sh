#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

exec ros2 run tf2_ros static_transform_publisher \
  --x 0.12 \
  --y 0.0 \
  --z 0.20 \
  --roll 0.0 \
  --pitch 0.0 \
  --yaw 0.0 \
  --frame-id robot1/base_link \
  --child-frame-id robot1/laser_frame
