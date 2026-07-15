#!/usr/bin/env bash

set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/fire_swarm_ws/install/setup.bash"

PROJECT_DIR="$HOME/fire_swarm_ws/src/WRO-colony/fire_swarm_environment"

MAP_YAML=$(grep -RIl \
  --include="*.yaml" \
  --include="*.yml" \
  '^[[:space:]]*image:' \
  "$PROJECT_DIR" | head -n 1)

if [ -z "$MAP_YAML" ]; then
  echo "[ERROR] 지도 YAML 파일을 찾지 못했습니다."
  echo "지도 YAML에는 image: 항목이 있어야 합니다."
  exit 1
fi

echo "사용할 지도:"
echo "$MAP_YAML"

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:="$MAP_YAML" &

MAP_SERVER_PID=$!

sleep 2

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

echo "지도 서버 활성화 완료"

wait "$MAP_SERVER_PID"
