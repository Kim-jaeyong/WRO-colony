#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAP="$ROOT/maps/nav2/osco_scenario_map_v2/osco_scenario_map_v2.yaml"
PARAMS="$ROOT/config/robot1_nav2_params.yaml"

exec ros2 launch nav2_bringup bringup_launch.py \
  map:="$MAP" \
  params_file:="$PARAMS" \
  use_sim_time:=true \
  autostart:=true \
  use_composition:=False
