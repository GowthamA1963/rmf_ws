# RobotClientAPI.py

import math
import threading
from typing import Dict, Optional

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


class RobotAPI:
    def __init__(self, node: Node):
        self._node = node

        self._pose_lock = threading.Lock()
        self._poses: Dict[str, PoseWithCovarianceStamped] = {}
        self._pose_subs: Dict[str, object] = {}

        self._nav_clients: Dict[str, ActionClient] = {}
        self._goal_results: Dict[str, Dict[int, str]] = {}

        self._node.get_logger().info(
            "RobotAPI initialized (AMCL + Nav2 fully namespaced)"
        )

    # ---------------- AMCL — namespaced ---------------- #

    def _ensure_amcl_sub(self, robot_name: str):
        if robot_name not in self._pose_subs:
            topic = f"/{robot_name}/amcl_pose"
            self._pose_subs[robot_name] = self._node.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                lambda msg, rn=robot_name: self._amcl_cb(msg, rn),
                10
            )
            self._node.get_logger().info(f"[RobotAPI] Subscribed: {topic}")

    def _amcl_cb(self, msg, robot_name):
        with self._pose_lock:
            self._poses[robot_name] = msg

    # ---------------- RMF Position API ---------------- #

    def position(self, robot_name: str):
        self._ensure_amcl_sub(robot_name)

        with self._pose_lock:
            msg = self._poses.get(robot_name)
            if msg is None:
                return None

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )
        return [p.x, p.y, yaw]

    def battery_soc(self, robot_name: str):
        return 1.0

    # ---------------- Nav2 — namespaced ---------------- #

    def _get_nav_client(self, robot_name: str):
        if robot_name not in self._nav_clients:
            topic = f"/{robot_name}/navigate_to_pose"
            self._nav_clients[robot_name] = ActionClient(
                self._node, NavigateToPose, topic
            )
            self._goal_results[robot_name] = {}
            self._node.get_logger().info(f"[RobotAPI] Nav2 client: {topic}")

        return self._nav_clients[robot_name]

    def navigate(self, robot_name: str, cmd_id: int, pose, map_name, speed_limit=0.0):
        x, y, yaw = pose[:3]
        client = self._get_nav_client(robot_name)

        if not client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().error(f"Nav2 unavailable for {robot_name}")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)

        send_future = client.send_goal_async(goal)

        def goal_response_cb(fut):
            handle = fut.result()
            if not handle.accepted:
                self._goal_results[robot_name][cmd_id] = 'rejected'
                return
            result_future = handle.get_result_async()

            def result_cb(res_fut):
                res = res_fut.result()
                status = 'succeeded' if res.status == 4 else f"status_{res.status}"
                self._goal_results[robot_name][cmd_id] = status

            result_future.add_done_callback(result_cb)

        send_future.add_done_callback(goal_response_cb)
        return True

    def navigation_completed(self, robot_name: str, cmd_id: int):
        return self._goal_results.get(robot_name, {}).get(cmd_id) == 'succeeded'

    # ---------------- RMF Stubs ---------------- #

    def requires_replan(self, *a, **k): return False
    def start_process(self, *a, **k): return True
    def process_completed(self, *a, **k): return True
    def toggle_action(self, *a, **k): return True
    def stop(self, *a, **k): return True
