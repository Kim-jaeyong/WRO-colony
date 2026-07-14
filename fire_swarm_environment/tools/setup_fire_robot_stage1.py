#!/usr/bin/env python3
"""
WRO fire swarm - Stage 1 robot setup

Run this script from:
~/fire_swarm_ws/src/WRO-colony/fire_swarm_environment

It will:
1) Create a simple differential-drive robot model.
2) Create a ROS-Gazebo bridge config.
3) Insert robot1 into worlds/osco_scenario_world.sdf.
4) Create run helper scripts.

Default spawn pose is inside the reconstructed lobby:
x=48.5 m, y=3.0 m, yaw=1.5708 rad
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>fire_robot</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>WRO Fire Swarm Team</name>
  </author>
  <description>Simple differential-drive robot for WRO fire swarm simulation.</description>
</model>
"""


MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="fire_robot" canonical_link="base_link">

    <link name="base_link">
      <pose relative_to="__model__">0 0 0.14 0 0 0</pose>

      <inertial>
        <mass>8.0</mass>
        <inertia>
          <ixx>0.0813</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1307</iyy>
          <iyz>0.0</iyz>
          <izz>0.1867</izz>
        </inertia>
      </inertial>

      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.42 0.32 0.16</size>
          </box>
        </geometry>
      </collision>

      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.42 0.32 0.16</size>
          </box>
        </geometry>
        <material>
          <ambient>0.15 0.25 0.75 1</ambient>
          <diffuse>0.15 0.25 0.75 1</diffuse>
        </material>
      </visual>

      <visual name="front_marker">
        <pose>0.20 0 0.03 0 0 0</pose>
        <geometry>
          <box>
            <size>0.04 0.14 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.9 0.15 0.05 1</ambient>
          <diffuse>0.9 0.15 0.05 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="left_wheel">
      <pose relative_to="__model__">0 0.18 0.08 -1.5708 0 0</pose>

      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.00082</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.00082</iyy>
          <iyz>0.0</iyz>
          <izz>0.00160</izz>
        </inertia>
      </inertial>

      <collision name="left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.5</mu>
              <mu2>1.5</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="left_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="right_wheel">
      <pose relative_to="__model__">0 -0.18 0.08 -1.5708 0 0</pose>

      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.00082</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.00082</iyy>
          <iyz>0.0</iyz>
          <izz>0.00160</izz>
        </inertia>
      </inertial>

      <collision name="right_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.5</mu>
              <mu2>1.5</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="right_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="caster">
      <pose relative_to="__model__">-0.15 0 0.04 0 0 0</pose>

      <inertial>
        <mass>0.15</mass>
        <inertia>
          <ixx>0.000096</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.000096</iyy>
          <iyz>0.0</iyz>
          <izz>0.000096</izz>
        </inertia>
      </inertial>

      <collision name="caster_collision">
        <geometry>
          <sphere>
            <radius>0.04</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.02</mu>
              <mu2>0.02</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="caster_visual">
        <geometry>
          <sphere>
            <radius>0.04</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.25 0.25 0.25 1</ambient>
          <diffuse>0.25 0.25 0.25 1</diffuse>
        </material>
      </visual>
    </link>

    <joint name="left_wheel_joint" type="revolute">
      <pose relative_to="left_wheel"/>
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <pose relative_to="right_wheel"/>
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
        </limit>
      </axis>
    </joint>

    <joint name="caster_joint" type="ball">
      <parent>base_link</parent>
      <child>caster</child>
    </joint>

    <plugin
      filename="gz-sim-diff-drive-system"
      name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.36</wheel_separation>
      <wheel_radius>0.08</wheel_radius>
      <odom_publish_frequency>30</odom_publish_frequency>
      <max_linear_velocity>0.8</max_linear_velocity>
      <max_angular_velocity>1.5</max_angular_velocity>
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <max_angular_acceleration>2.0</max_angular_acceleration>
    </plugin>

  </model>
</sdf>
"""


BRIDGE_YAML = """- ros_topic_name: "/robot1/cmd_vel"
  gz_topic_name: "/model/robot1/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/robot1/odom"
  gz_topic_name: "/model/robot1/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/tf"
  gz_topic_name: "/model/robot1/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
"""


RUN_GAZEBO = """#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

gz sim -r "$ROOT/worlds/osco_scenario_world.sdf"
"""


RUN_BRIDGE = """#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

ros2 launch ros_gz_bridge ros_gz_bridge.launch.py \
  bridge_name:=robot1_bridge \
  config_file:="$ROOT/config/robot1_bridge.yaml"
"""


WORLD_BLOCK_TEMPLATE = """
    <!-- FIRE_SWARM_ROBOT1_BEGIN -->
    <include>
      <uri>model://fire_robot</uri>
      <name>robot1</name>
      <pose>{x:.4f} {y:.4f} 0.0 0 0 {yaw:.4f}</pose>
    </include>
    <!-- FIRE_SWARM_ROBOT1_END -->
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--x", type=float, default=48.5)
    parser.add_argument("--y", type=float, default=3.0)
    parser.add_argument("--yaw", type=float, default=1.5708)
    return parser.parse_args()


def write_file(path: Path, content: str, executable: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    if executable:
        path.chmod(0o755)


def patch_world(world_path: Path, x: float, y: float, yaw: float) -> None:
    text = world_path.read_text(encoding="utf-8")

    if "FIRE_SWARM_ROBOT1_BEGIN" in text:
        print("robot1 is already present in the world. World patch skipped.")
        return

    if "</world>" not in text:
        raise ValueError(f"</world> not found in {world_path}")

    backup = world_path.with_suffix(world_path.suffix + ".before_robot1.bak")
    if not backup.exists():
        shutil.copy2(world_path, backup)

    block = WORLD_BLOCK_TEMPLATE.format(x=x, y=y, yaw=yaw)
    text = text.replace("</world>", block + "\n  </world>", 1)
    world_path.write_text(text, encoding="utf-8")
    print(f"Patched world: {world_path}")
    print(f"Backup world:  {backup}")


def main() -> int:
    args = parse_args()
    root = args.root.expanduser().resolve()

    world_path = root / "worlds" / "osco_scenario_world.sdf"
    if not world_path.exists():
        print(
            "Run this script from fire_swarm_environment or pass --root.\n"
            f"World file not found: {world_path}",
            file=sys.stderr,
        )
        return 1

    write_file(root / "models" / "fire_robot" / "model.config", MODEL_CONFIG)
    write_file(root / "models" / "fire_robot" / "model.sdf", MODEL_SDF)
    write_file(root / "config" / "robot1_bridge.yaml", BRIDGE_YAML)
    write_file(root / "scripts" / "run_osco_robot1_gazebo.sh", RUN_GAZEBO, True)
    write_file(root / "scripts" / "run_robot1_bridge.sh", RUN_BRIDGE, True)

    patch_world(world_path, args.x, args.y, args.yaw)

    print()
    print("Stage 1 robot setup complete.")
    print(f"Spawn pose: x={args.x}, y={args.y}, yaw={args.yaw}")
    print()
    print("Run Gazebo:")
    print("  ./scripts/run_osco_robot1_gazebo.sh")
    print()
    print("Run ROS-Gazebo bridge in a second terminal:")
    print("  ./scripts/run_robot1_bridge.sh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
