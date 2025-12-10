#!/usr/bin/env python3
import math
import threading
from typing import Dict, Optional, List
import re  # <-- Added for robot name matching

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

# RMF delivery messages
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_ingestor_msgs.msg import IngestorResult


class RobotAPI:
    """
    Cleaned RobotAPI with:
      - Lazy AMCL subscription per robot (only if topic exists)
      - Online robot detection helper
      - Single global Nav2 client
      - Clean RMF workcell interface
    """

    def __init__(self, node: Node):
        self._node = node

        # Pose handling
        self._pose_lock = threading.Lock()
        self._poses: Dict[str, PoseWithCovarianceStamped] = {}
        self._pose_subs: Dict[str, object] = {}

        # Nav2 client (shared)
        self._global_nav_client: Optional[ActionClient] = None
        self._goal_results: Dict[str, Dict[int, str]] = {}

        # Delivery process tracking
        self._process_lock = threading.Lock()
        self._process_cmd_map: Dict[int, str] = {}
        self._dock_cmds: Dict[str, List[int]] = {}
        self._process_status: Dict[int, bool] = {}

        # Subscribe to workcell results
        self._node.create_subscription(
            DispenserResult, "/dispenser_results",
            self._dispenser_result_cb, 10
        )
        self._node.create_subscription(
            IngestorResult, "/ingestor_results",
            self._ingestor_result_cb, 10
        )

        self._node.get_logger().info("RobotAPI initialized")

    # ----------------------------------------------------------------------
    # POSITION
    # ----------------------------------------------------------------------
    def _ensure_amcl_sub(self, robot_name: str) -> bool:
        if robot_name in self._pose_subs:
            return True

        topic = f"/{robot_name}/amcl_pose"
        active_topics = {t[0] for t in self._node.get_topic_names_and_types()}

        if topic not in active_topics:
            self._node.get_logger().debug(
                f"[RobotAPI] No active AMCL topic for {robot_name}"
            )
            return False

        self._pose_subs[robot_name] = self._node.create_subscription(
            PoseWithCovarianceStamped, topic,
            lambda msg, rn=robot_name: self._amcl_cb(msg, rn),
            10
        )
        self._node.get_logger().info(f"[RobotAPI] Subscribed: {topic}")
        return True

    def _amcl_cb(self, msg, robot_name):
        with self._pose_lock:
            self._poses[robot_name] = msg

    def position(self, robot_name: str):
        if not self._ensure_amcl_sub(robot_name):
            return None

        with self._pose_lock:
            msg = self._poses.get(robot_name)

        if msg is None:
            return None

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )
        return [p.x, p.y, yaw]

    def battery_soc(self, robot_name: str) -> float:
        return 1.0  # stub

    # ----------------------------------------------------------------------
    # NAVIGATION
    # ----------------------------------------------------------------------
    def _get_nav_client(self):
        if self._global_nav_client is None:
            self._global_nav_client = ActionClient(
                self._node, NavigateToPose, "/navigate_to_pose"
            )
            self._node.get_logger().info("[RobotAPI] Using /navigate_to_pose")
        return self._global_nav_client

    def navigate(self, robot_name, cmd_id, pose, map_name, speed_limit=0.0):
        if robot_name not in self._goal_results:
            self._goal_results[robot_name] = {}

        client = self._get_nav_client()
        if not client.wait_for_server(5.0):
            self._node.get_logger().error("Nav2 unavailable")
            return False

        x, y, yaw = pose[:3]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)

        future = client.send_goal_async(goal)

        def goal_cb(fut):
            handle = fut.result()
            if not handle.accepted:
                self._goal_results[robot_name][cmd_id] = "rejected"
                return
            result_future = handle.get_result_async()

            def result_cb(res_fut):
                status = "succeeded" if res_fut.result().status == 4 else "failed"
                self._goal_results[robot_name][cmd_id] = status

            result_future.add_done_callback(result_cb)

        future.add_done_callback(goal_cb)
        return True

    def navigation_completed(self, robot_name, cmd_id):
        return self._goal_results.get(robot_name, {}).get(cmd_id) == "succeeded"

    # ----------------------------------------------------------------------
    # DELIVERY
    # ----------------------------------------------------------------------
    def start_process(self, robot_name, cmd_id, dock_name, map_name):
        with self._process_lock:
            self._process_cmd_map[cmd_id] = dock_name
            self._dock_cmds.setdefault(dock_name, []).append(cmd_id)
            self._process_status[cmd_id] = False
        return True

    def process_completed(self, robot_name, cmd_id):
        return self._process_status.get(cmd_id, False)

    def _dispenser_result_cb(self, msg):
        self._mark_completed(msg.source_guid)

    def _ingestor_result_cb(self, msg):
        self._mark_completed(msg.source_guid)

    def _mark_completed(self, dock_name):
        with self._process_lock:
            if dock_name in self._dock_cmds:
                for cid in self._dock_cmds[dock_name]:
                    self._process_status[cid] = True

    # ----------------------------------------------------------------------
    # NO-OPS
    # ----------------------------------------------------------------------
    def requires_replan(self, *a, **k): return False
    def toggle_action(self, *a, **k): return True
    def stop(self, *a, **k): return True
