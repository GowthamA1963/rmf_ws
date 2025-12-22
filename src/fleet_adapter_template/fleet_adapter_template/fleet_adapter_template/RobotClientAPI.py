#!/usr/bin/env python3
import math
import threading
from typing import Dict, Optional, List
import re  # <-- Added for robot name matching

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSReliabilityPolicy as Reliability

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

# RMF delivery messages
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_ingestor_msgs.msg import IngestorResult
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState


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

        # Battery handling
        self._battery_lock = threading.Lock()
        self._battery_levels: Dict[str, float] = {}
        self._battery_subs: Dict[str, object] = {}

        # Nav2 client (shared)
        self._nav_clients: Dict[str, ActionClient] = {}
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

        amcl_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL
        )

        self._pose_subs[robot_name] = self._node.create_subscription(
            PoseWithCovarianceStamped, topic,
            lambda msg, rn=robot_name: self._amcl_cb(msg, rn),
            qos_profile=amcl_qos
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
        if not self._ensure_battery_sub(robot_name):
            return 1.0

        with self._battery_lock:
            return self._battery_levels.get(robot_name, 1.0)

    def _ensure_battery_sub(self, robot_name: str) -> bool:
        if robot_name in self._battery_subs:
            return True

        # Try to find a battery topic for this robot
        # We look for /{robot_name}/battery_percentage (Float32) or /{robot_name}/battery_state (BatteryState)
        target_topics = [
            (f"/{robot_name}/battery_percentage", Float32),
            (f"/{robot_name}/battery_state", BatteryState)
        ]

        active_topics = self._node.get_topic_names_and_types()
        active_topic_map = {t[0]: t[1] for t in active_topics}

        for topic, msg_type in target_topics:
            if topic in active_topic_map:
                self._battery_subs[robot_name] = self._node.create_subscription(
                    msg_type, topic,
                    lambda msg, rn=robot_name, mt=msg_type: self._battery_cb(msg, rn, mt),
                    10
                )
                self._node.get_logger().info(f"[RobotAPI] Subscribed to battery: {topic}")
                return True

        # Optional: Log only once per robot to avoid spam, or debug only
        self._node.get_logger().debug(f"[RobotAPI] No battery topic found for {robot_name}")
        return False

    def _battery_cb(self, msg, robot_name, msg_type):
        level = 1.0
        if msg_type == Float32:
            level = msg.data
        elif msg_type == BatteryState:
            level = msg.percentage

        with self._battery_lock:
            self._battery_levels[robot_name] = level

    # ----------------------------------------------------------------------
    # NAVIGATION
    # ----------------------------------------------------------------------
    def _get_nav_client(self, robot_name: str):
        if robot_name not in self._nav_clients:
            # Try specific namespace first
            topic = f"/{robot_name}/navigate_to_pose"
            # We don't strictly check if action server exists here,
            # we lazily create the client. user can check ready state.

            client = ActionClient(self._node, NavigateToPose, topic)
            self._nav_clients[robot_name] = client
            self._node.get_logger().info(f"[RobotAPI] Created client for {topic}")

        return self._nav_clients[robot_name]

    def navigate(self, robot_name, cmd_id, pose, map_name, speed_limit=0.0):
        if robot_name not in self._goal_results:
            self._goal_results[robot_name] = {}

        client = self._get_nav_client(robot_name)
        if not client.wait_for_server(5.0):
            self._node.get_logger().error(f"Nav2 unavailable for {robot_name}")
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
