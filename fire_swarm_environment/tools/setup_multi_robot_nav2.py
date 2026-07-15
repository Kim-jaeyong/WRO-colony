#!/usr/bin/env python3
from __future__ import annotations

import copy
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print('python3-yaml is required: sudo apt install -y python3-yaml', file=sys.stderr)
    raise

ROBOTS = {
    'robot1': {'x': 47.6, 'y': 3.0, 'yaw': 1.5708},
    'robot2': {'x': 48.5, 'y': 3.0, 'yaw': 1.5708},
    'robot3': {'x': 49.4, 'y': 3.0, 'yaw': 1.5708},
}


def replace_strings(value, old: str, new: str):
    if isinstance(value, str):
        return value.replace(old, new)
    if isinstance(value, list):
        return [replace_strings(item, old, new) for item in value]
    if isinstance(value, dict):
        return {
            replace_strings(key, old, new): replace_strings(item, old, new)
            for key, item in value.items()
        }
    return value


def ros_params(data: dict, node_name: str) -> dict:
    node = data.get(node_name)
    if not isinstance(node, dict) or not isinstance(node.get('ros__parameters'), dict):
        raise KeyError(f'Missing parameters for {node_name}')
    return node['ros__parameters']


def generate_params(root: Path) -> list[Path]:
    source = root / 'config' / 'robot1_nav2_params.yaml'
    if not source.exists():
        raise FileNotFoundError(source)

    base = yaml.safe_load(source.read_text(encoding='utf-8'))
    if not isinstance(base, dict):
        raise ValueError('Invalid Nav2 YAML')

    outputs = []
    for robot, pose in ROBOTS.items():
        data = replace_strings(copy.deepcopy(base), 'robot1', robot)

        amcl = ros_params(data, 'amcl')
        amcl.update({
            'use_sim_time': True,
            'base_frame_id': f'{robot}/base_link',
            'odom_frame_id': f'{robot}/odom',
            'global_frame_id': 'map',
            'scan_topic': f'/{robot}/scan',
            'set_initial_pose': True,
            'initial_pose': {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'z': 0.0,
                'yaw': float(pose['yaw']),
            },
        })

        bt = ros_params(data, 'bt_navigator')
        bt.update({
            'use_sim_time': True,
            'global_frame': 'map',
            'robot_base_frame': f'{robot}/base_link',
            'odom_topic': f'/{robot}/odom',
        })

        ros_params(data, 'controller_server')['use_sim_time'] = True
        planner = ros_params(data, 'planner_server')
        planner['use_sim_time'] = True
        if isinstance(planner.get('GridBased'), dict):
            planner['GridBased']['use_astar'] = True
            planner['GridBased']['allow_unknown'] = False

        local = data['local_costmap']['local_costmap']['ros__parameters']
        local.update({
            'use_sim_time': True,
            'global_frame': f'{robot}/odom',
            'robot_base_frame': f'{robot}/base_link',
        })
        for layer_name in ('voxel_layer', 'obstacle_layer'):
            layer = local.get(layer_name)
            if isinstance(layer, dict) and isinstance(layer.get('scan'), dict):
                layer['scan']['topic'] = f'/{robot}/scan'

        global_cm = data['global_costmap']['global_costmap']['ros__parameters']
        global_cm.update({
            'use_sim_time': True,
            'global_frame': 'map',
            'robot_base_frame': f'{robot}/base_link',
        })
        obstacle = global_cm.get('obstacle_layer')
        if isinstance(obstacle, dict) and isinstance(obstacle.get('scan'), dict):
            obstacle['scan']['topic'] = f'/{robot}/scan'

        behavior = ros_params(data, 'behavior_server')
        behavior.update({
            'use_sim_time': True,
            'local_frame': f'{robot}/odom',
            'global_frame': 'map',
            'robot_base_frame': f'{robot}/base_link',
        })

        if 'velocity_smoother' in data:
            velocity = ros_params(data, 'velocity_smoother')
            velocity['use_sim_time'] = True
            velocity['odom_topic'] = f'/{robot}/odom'

        if 'collision_monitor' in data:
            collision = ros_params(data, 'collision_monitor')
            collision.update({
                'use_sim_time': True,
                'base_frame_id': f'{robot}/base_link',
                'odom_frame_id': f'{robot}/odom',
            })
            if isinstance(collision.get('scan'), dict):
                collision['scan']['topic'] = f'/{robot}/scan'

        destination = root / 'config' / f'{robot}_nav2_params.yaml'
        destination.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
        outputs.append(destination)

    return outputs


def bridge_entry(ros_topic, gz_topic, ros_type, gz_type, direction):
    return f'''- ros_topic_name: "{ros_topic}"
  gz_topic_name: "{gz_topic}"
  ros_type_name: "{ros_type}"
  gz_type_name: "{gz_type}"
  direction: {direction}
'''


def generate_bridge(root: Path) -> Path:
    parts = []
    for robot in ROBOTS:
        parts += [
            bridge_entry(f'/{robot}/cmd_vel', f'/model/{robot}/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
            bridge_entry(f'/{robot}/odom', f'/model/{robot}/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
            bridge_entry(f'/{robot}/tf', f'/model/{robot}/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS'),
            bridge_entry(f'/{robot}/scan', f'/{robot}/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', 'GZ_TO_ROS'),
        ]
    parts.append(bridge_entry('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS'))
    path = root / 'config' / 'multi_robot_bridge.yaml'
    path.write_text('\n'.join(parts), encoding='utf-8')
    return path


def write_scripts(root: Path) -> list[Path]:
    scripts = root / 'scripts'
    scripts.mkdir(parents=True, exist_ok=True)

    tf_path = scripts / 'run_multi_robot_tf.sh'
    tf_path.write_text('''#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
PIDS=()
cleanup() { for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
for robot in robot1 robot2 robot3; do
  ros2 run tf2_ros static_transform_publisher \
    --x 0.12 --y 0.0 --z 0.20 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id "${robot}/base_link" \
    --child-frame-id "${robot}/laser_frame" \
    --ros-args -r /tf_static:="/${robot}/tf_static" &
  PIDS+=("$!")
done
wait
''', encoding='utf-8')

    nav_path = scripts / 'run_multi_robot_nav2.sh'
    nav_path.write_text('''#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
MAP="$ROOT/maps/nav2/osco_scenario_map_v2/osco_scenario_map_v2.yaml"
PIDS=()
cleanup() { for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
for robot in robot1 robot2 robot3; do
  echo "Starting Nav2 for ${robot}..."
  ros2 launch nav2_bringup bringup_launch.py \
    namespace:="${robot}" \
    use_namespace:=true \
    map:="$MAP" \
    params_file:="$ROOT/config/${robot}_nav2_params.yaml" \
    use_sim_time:=true \
    autostart:=true \
    use_composition:=True \
    use_respawn:=False \
    log_level:=info &
  PIDS+=("$!")
  sleep 3
done
wait
''', encoding='utf-8')

    check_path = scripts / 'check_multi_robot_nav2.sh'
    check_path.write_text('''#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
for robot in robot1 robot2 robot3; do
  echo
  echo "===== ${robot} ====="
  ros2 lifecycle get "/${robot}/amcl" || true
  ros2 lifecycle get "/${robot}/controller_server" || true
  ros2 lifecycle get "/${robot}/planner_server" || true
done
''', encoding='utf-8')

    for path in (tf_path, nav_path, check_path):
        path.chmod(0o755)
    return [tf_path, nav_path, check_path]


def main() -> int:
    root = Path.cwd().resolve()
    required = [
        root / 'config' / 'robot1_nav2_params.yaml',
        root / 'config' / 'multi_robot_bridge.yaml',
        root / 'scripts' / 'run_multi_robot_gazebo.sh',
    ]
    missing = [str(p) for p in required if not p.exists()]
    if missing:
        print('Run this from fire_swarm_environment. Missing:\n  ' + '\n  '.join(missing), file=sys.stderr)
        return 1

    try:
        params = generate_params(root)
        bridge = generate_bridge(root)
        scripts = write_scripts(root)
    except (OSError, ValueError, KeyError) as exc:
        print(f'Stage 4 setup failed: {exc}', file=sys.stderr)
        return 2

    print('Stage 4 multi-robot Nav2 setup complete.')
    for path in params:
        print(path)
    print(bridge)
    for path in scripts:
        print(path)
    print('Restart bridge and TF before starting Nav2.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
