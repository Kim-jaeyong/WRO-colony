#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

exec gz sim -r "$ROOT/worlds/osco_multi_robot_world.sdf"
