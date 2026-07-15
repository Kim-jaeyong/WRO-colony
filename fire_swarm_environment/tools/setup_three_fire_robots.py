#!/usr/bin/env python3
"""
WRO Fire Swarm - Stage 3: prepare three isolated Gazebo robots.

Run from:
  ~/fire_swarm_ws/src/WRO-colony/fire_swarm_environment

This preserves the working single-robot setup and creates separate multi-robot files:
- models/fire_robot_1, fire_robot_2, fire_robot_3
- worlds/osco_multi_robot_world.sdf
- config/multi_robot_bridge.yaml
- scripts/run_multi_robot_gazebo.sh
- scripts/run_multi_robot_bridge.sh
- scripts/run_multi_robot_tf.sh
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROBOTS = {
    "robot1": {"x": 47.6, "y": 3.0, "yaw": 1.5708},
    "robot2": {"x": 48.5, "y": 3.0, "yaw": 1.5708},
    "robot3": {"x": 49.4, "y": 3.0, "yaw": 1.5708},
}


def copy_robot_model(root: Path, robot_name: str, number: int) -> None:
    source_dir = root / "models" / "fire_robot"
    target_dir = root / "models" / f"fire_robot_{number}"

    if not source_dir.exists():
        raise FileNotFoundError(f"Base robot model not found: {source_dir}")

    target_dir.mkdir(parents=True, exist_ok=True)

    source_sdf = (source_dir / "model.sdf").read_text(encoding="utf-8")
    source_config = (source_dir / "model.config").read_text(encoding="utf-8")

    model_sdf = source_sdf.replace("robot1", robot_name)
    model_sdf = model_sdf.replace(
        '<model name="fire_robot"',
        f'<model name="fire_robot_{number}"',
        1,
    )

    model_config = source_config.replace(
        "<name>fire_robot</name>",
        f"<name>fire_robot_{number}</name>",
        1,
    )

    (target_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")
    (target_dir / "model.config").write_text(model_config, encoding="utf-8")


def make_multi_world(root: Path) -> Path:
    single_world = root / "worlds" / "osco_scenario_world.sdf"
    multi_world = root / "worlds" / "osco_multi_robot_world.sdf"

    if not single_world.exists():
        raise FileNotFoundError(f"World not found: {single_world}")

    text = single_world.read_text(encoding="utf-8")

    blocks = ["    <!-- FIRE_SWARM_MULTI_ROBOTS_BEGIN -->"]
    for index, (robot_name, pose) in enumerate(ROBOTS.items(), start=1):
        blocks.append(
            f"""    <include>
      <uri>model://fire_robot_{index}</uri>
      <name>{robot_name}</name>
      <pose>{pose['x']:.4f} {pose['y']:.4f} 0.0 0 0 {pose['yaw']:.4f}</pose>
    </include>"""
        )
    blocks.append("    <!-- FIRE_SWARM_MULTI_ROBOTS_END -->")
    multi_block = "\n".join(blocks)

    single_pattern = re.compile(
        r"\s*<!-- FIRE_SWARM_ROBOT1_BEGIN -->.*?"
        r"<!-- FIRE_SWARM_ROBOT1_END -->\s*",
        re.DOTALL,
    )
    text = single_pattern.sub("\n", text)

    multi_pattern = re.compile(
        r"\s*<!-- FIRE_SWARM_MULTI_ROBOTS_BEGIN -->.*?"
        r"<!-- FIRE_SWARM_MULTI_ROBOTS_END -->\s*",
        re.DOTALL,
    )
    text = multi_pattern.sub("\n", text)

    if "</world>" not in text:
        raise ValueError(f"</world> not found in {single_world}")

    text = text.replace("</world>", f"\n{multi_block}\n\n  </world>", 1)
    multi_world.write_text(text, encoding="utf-8")
    return multi_world


def bridge_entry(
    ros_topic: str,
    gz_topic: str,
    ros_type: str,
    gz_type: str,
    direction: str,
) -> str:
    return f"""- ros_topic_name: "{ros_topic}"
  gz_topic_name: "{gz_topic}"
  ros_type_name: "{ros_type}"
  gz_type_name: "{gz_type}"
  direction: {direction}
"""


def make_bridge_config(root: Path) -> Path:
    parts = []

    for robot_name in ROBOTS:
        parts.append(
            bridge_entry(
                f"/{robot_name}/cmd_vel",
                f"/model/{robot_name}/cmd_vel",
                "geometry_msgs/msg/Twist",
                "gz.msgs.Twist",
                "ROS_TO_GZ",
            )
        )
        parts.append(
            bridge_entry(
                f"/{robot_name}/odom",
                f"/model/{robot_name}/odometry",
                "nav_msgs/msg/Odometry",
                "gz.msgs.Odometry",
                "GZ_TO_ROS",
            )
        )
        parts.append(
            bridge_entry(
                "/tf",
                f"/model/{robot_name}/tf",
                "tf2_msgs/msg/TFMessage",
                "gz.msgs.Pose_V",
                "GZ_TO_ROS",
            )
        )
        parts.append(
            bridge_entry(
                f"/{robot_name}/scan",
                f"/{robot_name}/scan",
                "sensor_msgs/msg/LaserScan",
                "gz.msgs.LaserScan",
                "GZ_TO_ROS",
            )
        )

    parts.append(
        bridge_entry(
            "/clock",
            "/clock",
            "rosgraph_msgs/msg/Clock",
            "gz.msgs.Clock",
            "GZ_TO_ROS",
        )
    )

    bridge_path = root / "config" / "multi_robot_bridge.yaml"
    bridge_path.parent.mkdir(parents=True, exist_ok=True)
    bridge_path.write_text("\n".join(parts), encoding="utf-8")
    return bridge_path


def make_scripts(root: Path) -> list[Path]:
    scripts_dir = root / "scripts"
    scripts_dir.mkdir(parents=True, exist_ok=True)

    gazebo_script = scripts_dir / "run_multi_robot_gazebo.sh"
    gazebo_script.write_text(
        """#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

exec gz sim -r "$ROOT/worlds/osco_multi_robot_world.sdf"
""",
        encoding="utf-8",
    )

    bridge_script = scripts_dir / "run_multi_robot_bridge.sh"
    bridge_script.write_text(
        """#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

exec ros2 launch ros_gz_bridge ros_gz_bridge.launch.py \
  bridge_name:=multi_robot_bridge \
  config_file:="$ROOT/config/multi_robot_bridge.yaml"
""",
        encoding="utf-8",
    )

    tf_script = scripts_dir / "run_multi_robot_tf.sh"
    tf_script.write_text(
        """#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

PIDS=()

cleanup() {
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}

trap cleanup EXIT INT TERM

for robot in robot1 robot2 robot3; do
  ros2 run tf2_ros static_transform_publisher \
    --x 0.12 \
    --y 0.0 \
    --z 0.20 \
    --roll 0.0 \
    --pitch 0.0 \
    --yaw 0.0 \
    --frame-id "${robot}/base_link" \
    --child-frame-id "${robot}/laser_frame" &
  PIDS+=("$!")
done

wait
""",
        encoding="utf-8",
    )

    paths = [gazebo_script, bridge_script, tf_script]
    for path in paths:
        path.chmod(0o755)

    return paths


def validate_generated_models(root: Path) -> None:
    for index, robot_name in enumerate(ROBOTS, start=1):
        model_text = (
            root / "models" / f"fire_robot_{index}" / "model.sdf"
        ).read_text(encoding="utf-8")

        required = [
            f"<frame_id>{robot_name}/odom</frame_id>",
            f"<child_frame_id>{robot_name}/base_link</child_frame_id>",
            f"<topic>/{robot_name}/scan</topic>",
            f"<gz_frame_id>{robot_name}/laser_frame</gz_frame_id>",
        ]

        missing = [item for item in required if item not in model_text]
        if missing:
            raise ValueError(
                f"{robot_name} model is missing expected settings: {missing}"
            )


def main() -> int:
    root = Path.cwd().resolve()

    required = [
        root / "models" / "fire_robot" / "model.sdf",
        root / "models" / "fire_robot" / "model.config",
        root / "worlds" / "osco_scenario_world.sdf",
    ]

    missing = [str(path) for path in required if not path.exists()]
    if missing:
        print(
            "Run this script from fire_swarm_environment.\nMissing:\n  "
            + "\n  ".join(missing),
            file=sys.stderr,
        )
        return 1

    try:
        for index, robot_name in enumerate(ROBOTS, start=1):
            copy_robot_model(root, robot_name, index)

        validate_generated_models(root)
        world = make_multi_world(root)
        bridge = make_bridge_config(root)
        scripts = make_scripts(root)

    except (OSError, ValueError) as exc:
        print(f"Multi-robot setup failed: {exc}", file=sys.stderr)
        return 2

    print("Stage 3 multi-robot setup complete.")
    print()
    print(f"World:  {world}")
    print(f"Bridge: {bridge}")
    print("Scripts:")
    for path in scripts:
        print(f"  {path}")
    print()
    print("Spawn poses:")
    for robot_name, pose in ROBOTS.items():
        print(
            f"  {robot_name}: "
            f"x={pose['x']}, y={pose['y']}, yaw={pose['yaw']}"
        )
    print()
    print("Run:")
    print("  ./scripts/run_multi_robot_gazebo.sh")
    print("  ./scripts/run_multi_robot_bridge.sh")
    print("  ./scripts/run_multi_robot_tf.sh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
