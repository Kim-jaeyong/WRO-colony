#!/usr/bin/env python3
"""
WRO Fire Swarm - Stage 2: add a 2D LiDAR to robot1.

Run from:
  ~/fire_swarm_ws/src/WRO-colony/fire_swarm_environment

This script:
1. Adds lidar_link + gpu_lidar sensor to models/fire_robot/model.sdf
2. Adds the Gazebo Sensors system to the world
3. Adds /robot1/scan to config/robot1_bridge.yaml
4. Creates backups before modifying files
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path


LIDAR_BLOCK = r'''
    <!-- FIRE_SWARM_LIDAR_BEGIN -->
    <link name="lidar_link">
      <pose relative_to="base_link">0.12 0 0.17 0 0 0</pose>

      <inertial>
        <mass>0.10</mass>
        <inertia>
          <ixx>0.00010</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.00010</iyy>
          <iyz>0.0</iyz>
          <izz>0.00010</izz>
        </inertia>
      </inertial>

      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <radius>0.055</radius>
            <length>0.045</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.055</radius>
            <length>0.045</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
        </material>
      </visual>

      <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0.03 0 0 0</pose>
        <topic>/robot1/scan</topic>
        <update_rate>10</update_rate>
        <always_on>true</always_on>
        <visualize>true</visualize>

        <lidar>
          <scan>
            <horizontal>
              <samples>720</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159265</min_angle>
              <max_angle>3.14159265</max_angle>
            </horizontal>
          </scan>

          <range>
            <min>0.12</min>
            <max>15.0</max>
            <resolution>0.01</resolution>
          </range>

          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.005</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>
    <!-- FIRE_SWARM_LIDAR_END -->

'''


SENSORS_PLUGIN = r'''
    <!-- FIRE_SWARM_SENSORS_SYSTEM_BEGIN -->
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- FIRE_SWARM_SENSORS_SYSTEM_END -->

'''


BRIDGE_BLOCK = r'''
- ros_topic_name: "/robot1/scan"
  gz_topic_name: "/robot1/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
'''


def backup(path: Path, suffix: str) -> Path:
    backup_path = path.with_name(path.name + suffix)
    if not backup_path.exists():
        shutil.copy2(path, backup_path)
    return backup_path


def patch_model(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    if "FIRE_SWARM_LIDAR_BEGIN" in text:
        print("LiDAR already exists in model. Model patch skipped.")
        return

    marker = '    <link name="left_wheel">'
    if marker not in text:
        raise ValueError(f"Could not find insertion point in {path}")

    backup_path = backup(path, ".before_lidar.bak")
    text = text.replace(marker, LIDAR_BLOCK + marker, 1)
    path.write_text(text, encoding="utf-8")

    print(f"Patched model: {path}")
    print(f"Backup model:  {backup_path}")


def patch_world(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    if "FIRE_SWARM_SENSORS_SYSTEM_BEGIN" in text:
        print("Gazebo Sensors system already exists. World patch skipped.")
        return

    marker = "    <scene>"
    if marker not in text:
        raise ValueError(f"Could not find <scene> insertion point in {path}")

    backup_path = backup(path, ".before_lidar.bak")
    text = text.replace(marker, SENSORS_PLUGIN + marker, 1)
    path.write_text(text, encoding="utf-8")

    print(f"Patched world: {path}")
    print(f"Backup world:  {backup_path}")


def patch_bridge(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    if 'ros_topic_name: "/robot1/scan"' in text:
        print("/robot1/scan already exists in bridge config. Bridge patch skipped.")
        return

    backup_path = backup(path, ".before_lidar.bak")
    if text and not text.endswith("\n"):
        text += "\n"

    text += "\n" + BRIDGE_BLOCK.lstrip()
    path.write_text(text, encoding="utf-8")

    print(f"Patched bridge: {path}")
    print(f"Backup bridge:  {backup_path}")


def main() -> int:
    root = Path.cwd().resolve()

    model_path = root / "models" / "fire_robot" / "model.sdf"
    world_path = root / "worlds" / "osco_scenario_world.sdf"
    bridge_path = root / "config" / "robot1_bridge.yaml"

    missing = [
        str(path)
        for path in (model_path, world_path, bridge_path)
        if not path.exists()
    ]

    if missing:
        print(
            "Required files were not found. Run this script from "
            "fire_swarm_environment.\nMissing:\n  "
            + "\n  ".join(missing),
            file=sys.stderr,
        )
        return 1

    try:
        patch_model(model_path)
        patch_world(world_path)
        patch_bridge(bridge_path)
    except (OSError, ValueError) as exc:
        print(f"Stage 2 setup failed: {exc}", file=sys.stderr)
        return 2

    print()
    print("Stage 2 LiDAR setup complete.")
    print("Restart Gazebo and the ROS-Gazebo bridge.")
    print()
    print("Gazebo:")
    print("  ./scripts/run_osco_robot1_gazebo.sh")
    print()
    print("Bridge:")
    print("  ./scripts/run_robot1_bridge.sh")
    print()
    print("Check scan:")
    print("  ros2 topic echo /robot1/scan --once")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
