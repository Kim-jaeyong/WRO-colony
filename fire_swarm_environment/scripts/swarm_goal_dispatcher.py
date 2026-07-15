#!/usr/bin/env python3
"""Send one simultaneous Nav2 test goal to robot1, robot2, and robot3.

Important:
- Restart the multi-robot Gazebo world before running this test.
- The restart returns all robots to y=3.0.
- This script sends them to the already-tested y=5.5 lobby line.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Dict

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass(frozen=True)
class GoalPose:
    x: float
    y: float
    yaw: float


GOALS: Dict[str, GoalPose] = {
    "robot1": GoalPose(47.6, 5.5, math.pi / 2.0),
    "robot2": GoalPose(48.5, 5.5, math.pi / 2.0),
    "robot3": GoalPose(49.4, 5.5, math.pi / 2.0),
}


class SwarmGoalDispatcher(Node):
    def __init__(self) -> None:
        super().__init__("swarm_goal_dispatcher")

        # Do not use the name `_clients`; rclpy.Node uses it internally.
        self._nav_clients = {
            robot: ActionClient(
                self,
                NavigateToPose,
                f"/{robot}/navigate_to_pose",
            )
            for robot in GOALS
        }

        self._remaining = len(GOALS)
        self._failed = False

    def wait_for_servers(self) -> bool:
        for robot, client in self._nav_clients.items():
            self.get_logger().info(
                f"Waiting for /{robot}/navigate_to_pose ..."
            )
            if not client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(
                    f"{robot}: Nav2 action server not found"
                )
                return False
        return True

    def send_all(self) -> None:
        for robot, target in GOALS.items():
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = target.x
            goal.pose.pose.position.y = target.y

            half_yaw = target.yaw / 2.0
            goal.pose.pose.orientation.z = math.sin(half_yaw)
            goal.pose.pose.orientation.w = math.cos(half_yaw)

            self.get_logger().info(
                f"{robot}: sending goal "
                f"({target.x:.1f}, {target.y:.1f})"
            )

            future = self._nav_clients[robot].send_goal_async(goal)
            future.add_done_callback(
                lambda done, name=robot:
                self._goal_response(name, done)
            )

    def _goal_response(self, robot: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"{robot}: send failed: {exc}"
            )
            self._failed = True
            self._finish_one()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{robot}: goal rejected")
            self._failed = True
            self._finish_one()
            return

        self.get_logger().info(f"{robot}: goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, name=robot:
                self._result(name, done)
        )

    def _result(self, robot: str, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"{robot}: result failed: {exc}"
            )
            self._failed = True
            self._finish_one()
            return

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{robot}: SUCCEEDED")
        else:
            self.get_logger().error(
                f"{robot}: action status={wrapped.status}, "
                f"error_code={wrapped.result.error_code}, "
                f"error_msg={wrapped.result.error_msg!r}"
            )
            self._failed = True

        self._finish_one()

    def _finish_one(self) -> None:
        self._remaining -= 1

        if self._remaining > 0:
            return

        if self._failed:
            self.get_logger().error(
                "One or more robots failed."
            )
        else:
            self.get_logger().info(
                "All three robots reached their goals."
            )

        rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = SwarmGoalDispatcher()

    try:
        if not node.wait_for_servers():
            return 2

        node.send_all()
        rclpy.spin(node)
        return 1 if node._failed else 0

    except KeyboardInterrupt:
        return 130

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
