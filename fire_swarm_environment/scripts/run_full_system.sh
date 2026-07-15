#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="$HOME/fire_swarm_ws/src/WRO-colony/fire_swarm_environment"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$HOME/fire_swarm_ws/install/setup.bash"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[ERROR] gnome-terminal을 찾을 수 없습니다."
  exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
  echo "[ERROR] ROS 2 Jazzy 설정 파일이 없습니다."
  exit 1
fi

if [ ! -f "$WS_SETUP" ]; then
  echo "[ERROR] 워크스페이스가 빌드되지 않았습니다."
  echo "먼저 아래 명령을 실행하세요."
  echo
  echo "cd ~/fire_swarm_ws"
  echo "source /opt/ros/jazzy/setup.bash"
  echo "colcon build --packages-select fire_swarm_environment --symlink-install"
  exit 1
fi

open_terminal()
{
  local title="$1"
  local command="$2"

  gnome-terminal \
    --title="$title" \
    -- bash -lc '
      source "$1"
      source "$2"
      cd "$3"

      eval "$4"

      echo
      echo "프로세스가 종료되었습니다."
      exec bash
    ' _ "$ROS_SETUP" "$WS_SETUP" "$PROJECT_DIR" "$command"
}

echo "========================================"
echo " Fire Swarm 전체 시스템 실행"
echo "========================================"

echo "[1/5] Gazebo 실행"
open_terminal \
  "Fire Swarm - Gazebo" \
  "./scripts/run_multi_robot_gazebo.sh"

sleep 5

echo "[2/5] Gazebo-ROS Bridge 실행"
open_terminal \
  "Fire Swarm - Bridge" \
  "./scripts/run_multi_robot_bridge.sh"

sleep 2

echo "[3/5] TF 실행"
open_terminal \
  "Fire Swarm - TF" \
  "./scripts/run_multi_robot_tf.sh"

sleep 2

echo "[4/5] Nav2 실행"
open_terminal \
  "Fire Swarm - Nav2" \
  "./scripts/run_multi_robot_nav2.sh"

sleep 2

echo "[5/5] Nav2 준비 확인 후 Dispatcher 실행"
open_terminal \
  "Fire Swarm - Dispatcher" \
  '
    echo "robot1과 robot2의 Nav2 액션 서버를 기다리는 중..."

    until \
      ros2 action list 2>/dev/null | grep -qx "/robot1/navigate_to_pose" &&
      ros2 action list 2>/dev/null | grep -qx "/robot2/navigate_to_pose"
    do
      sleep 1
    done

    echo "Nav2 액션 서버 확인 완료"
    echo "Nav2 안정화를 위해 5초 기다립니다."

    sleep 5

    echo "C++ 군집 탐색 Dispatcher 실행"
    ros2 run fire_swarm_environment swarm_goal_dispatcher
  '

echo
echo "전체 실행 명령을 완료했습니다."
echo "Gazebo, Bridge, TF, Nav2, Dispatcher 터미널을 종료하지 마세요."
