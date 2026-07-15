#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
MAP="$ROOT/maps/nav2/osco_scenario_map_v2/osco_scenario_map_v2.yaml"
PIDS=()
cleanup() { for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
for robot in robot1 robot2 robot3; do
  echo "Starting Nav2 for ${robot}..."
  ros2 launch nav2_bringup bringup_launch.py     namespace:="${robot}"     use_namespace:=true     map:="$MAP"     params_file:="$ROOT/config/${robot}_nav2_params.yaml"     use_sim_time:=true     autostart:=true     use_composition:=True     use_respawn:=False     log_level:=info &
  PIDS+=("$!")
  sleep 3
done
wait
