# RobotClientAPI.py

import math
import threading
from typing import Dict, Optional

import rclpy
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

        # For navigation we keep one shared ActionClient for the global /navigate_to_pose
        self._global_nav_client: Optional[ActionClient] = None
        self._global_nav_topic = "/navigate_to_pose"

        # still keep per-robot result dictionaries
        self._goal_results: Dict[str, Dict[int, str]] = {}

        self._node.get_logger().info(
            "RobotAPI initialized (AMCL namespaced, Nav2 client on /navigate_to_pose)"
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

    def _amcl_cb(self, msg: PoseWithCovarianceStamped, robot_name: str):
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
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return [p.x, p.y, yaw]

    def battery_soc(self, robot_name: str):
        return 1.0

    # ---------------- Nav2 — shared global client ---------------- #

    def _get_nav_client(self):
        """
        Return a single shared ActionClient for the global /navigate_to_pose topic.
        Create lazily.
        """
        if self._global_nav_client is None:
            self._global_nav_client = ActionClient(self._node, NavigateToPose, self._global_nav_topic)
            self._node.get_logger().info(f"[RobotAPI] Nav2 client created on: {self._global_nav_topic}")
        return self._global_nav_client

    def navigate(self, robot_name: str, cmd_id: int, pose, map_name, speed_limit=0.0):
        """
        Send a NavigateToPose goal using the shared global ActionClient.
        Defensive: wait for server, validate futures/handles, and guard callbacks.
        """
        # ensure per-robot result dict exists
        if robot_name not in self._goal_results:
            self._goal_results[robot_name] = {}

        x, y, yaw = pose[:3]
        client = self._get_nav_client()

        # Wait robustly for server (increase timeout if required)
        try:
            if not client.wait_for_server(timeout_sec=5.0):
                self._node.get_logger().error(f"[RobotAPI] Nav2 unavailable on {self._global_nav_topic}")
                return False
        except Exception as e:
            self._node.get_logger().error(f"[RobotAPI] Exception while waiting for server: {e}")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        try:
            send_future = client.send_goal_async(goal)
        except Exception as e:
            self._node.get_logger().error(f"[RobotAPI] Failed to send goal async for {robot_name}: {e}")
            self._goal_results[robot_name][cmd_id] = 'error'
            return False

        def goal_response_cb(fut):
            try:
                handle = fut.result()
            except Exception as e:
                self._node.get_logger().error(f"[RobotAPI] Exception getting goal handle for {robot_name}: {e}")
                self._goal_results[robot_name][cmd_id] = 'error'
                return

            if handle is None:
                self._node.get_logger().error(f"[RobotAPI] Goal response for {robot_name} returned None handle")
                self._goal_results[robot_name][cmd_id] = 'error'
                return

            # accepted?
            accepted = getattr(handle, "accepted", False)
            if not accepted:
                self._node.get_logger().info(f"[RobotAPI] Goal rejected for {robot_name}")
                self._goal_results[robot_name][cmd_id] = 'rejected'
                return

            # get result future, guard exceptions
            try:
                result_future = handle.get_result_async()
            except Exception as e:
                self._node.get_logger().error(f"[RobotAPI] Failed to get_result_async for {robot_name}: {e}")
                self._goal_results[robot_name][cmd_id] = 'error'
                return

            def result_cb(res_fut):
                try:
                    res = res_fut.result()
                except Exception as e:
                    self._node.get_logger().error(f"[RobotAPI] Exception in result future for {robot_name}: {e}")
                    self._goal_results[robot_name][cmd_id] = 'error'
                    return

                # rclpy statuses: 4 == SUCCEEDED
                status_val = getattr(res, 'status', None)
                status = 'succeeded' if status_val == 4 else f"status_{status_val}"
                self._goal_results[robot_name][cmd_id] = status
                self._node.get_logger().info(f"[RobotAPI] Goal result for {robot_name} cmd {cmd_id}: {status}")

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
