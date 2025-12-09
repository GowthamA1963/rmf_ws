#!/usr/bin/env python3
import math
import threading
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

# RMF delivery messages
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_ingestor_msgs.msg import IngestorResult


class RobotAPI:
    def __init__(self, node: Node):
        self._node = node

        # Pose handling
        self._pose_lock = threading.Lock()
        self._poses: Dict[str, PoseWithCovarianceStamped] = {}
        self._pose_subs: Dict[str, object] = {}

        # Nav2 client
        self._global_nav_client: Optional[ActionClient] = None
        self._goal_results: Dict[str, Dict[int, str]] = {}

        # Delivery process tracking
        self._process_lock = threading.Lock()
        self._process_cmd_map: Dict[int, str] = {}   # cmd_id -> dock_name
        self._dock_cmds: Dict[str, List[int]] = {}   # dock_name -> list[cmd_ids]
        self._process_status: Dict[int, bool] = {}   # cmd_id -> completed

        # Subscribe to workcell results
        self._node.create_subscription(
            DispenserResult,
            "/dispenser_results",
            self._dispenser_result_cb,
            10
        )

        self._node.create_subscription(
            IngestorResult,
            "/ingestor_results",
            self._ingestor_result_cb,
            10
        )

        self._node.get_logger().info("RobotAPI initialized (clean mode)")

    # ----------------------------------------------------------------------
    # POSITION
    # ----------------------------------------------------------------------
    def _ensure_amcl_sub(self, robot_name):
        if robot_name not in self._pose_subs:
            topic = f"/{robot_name}/amcl_pose"
            self._pose_subs[robot_name] = self._node.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                lambda msg, rn=robot_name: self._amcl_cb(msg, rn),
                10
            )
            self._node.get_logger().info(f"[RobotAPI] Subscribed to: {topic}")

    def _amcl_cb(self, msg, robot_name):
        with self._pose_lock:
            self._poses[robot_name] = msg

    def position(self, robot_name):
        self._ensure_amcl_sub(robot_name)
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

    def battery_soc(self, robot_name):
        return 1.0  # stub

    # ----------------------------------------------------------------------
    # NAVIGATION
    # ----------------------------------------------------------------------
    def _get_nav_client(self):
        if self._global_nav_client is None:
            self._global_nav_client = ActionClient(
                self._node, NavigateToPose, "/navigate_to_pose")
        return self._global_nav_client

    def navigate(self, robot_name, cmd_id, pose, map_name, speed_limit=0.0):

        if robot_name not in self._goal_results:
            self._goal_results[robot_name] = {}

        client = self._get_nav_client()
        if not client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error("Nav2 server unavailable")
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
                result = res_fut.result()
                status = 'succeeded' if result.status == 4 else f"status_{result.status}"
                self._goal_results[robot_name][cmd_id] = status

            result_future.add_done_callback(result_cb)

        future.add_done_callback(goal_cb)
        return True

    def navigation_completed(self, robot_name, cmd_id):
        return self._goal_results.get(robot_name, {}).get(cmd_id) == "succeeded"

    # ----------------------------------------------------------------------
    # DELIVERY PROCESSES (RMF → Workcell)
    # ----------------------------------------------------------------------
    def start_process(self, robot_name, cmd_id, dock_name, map_name):
        """
        RMF calls this when robot arrives at pickup/dropoff.
        We only record the request.
        """
        with self._process_lock:
            self._process_cmd_map[cmd_id] = dock_name
            self._dock_cmds.setdefault(dock_name, []).append(cmd_id)
            self._process_status[cmd_id] = False

        self._node.get_logger().info(
            f"[RobotAPI] start_process: robot={robot_name}, dock={dock_name}, cmd_id={cmd_id}"
        )
        return True

    def process_completed(self, robot_name, cmd_id):
        with self._process_lock:
            return self._process_status.get(cmd_id, False)

    # ----------------------------------------------------------------------
    # CALLBACKS FROM WORKCELLS
    # ----------------------------------------------------------------------
    def _dispenser_result_cb(self, msg: DispenserResult):
        dock_name = msg.source_guid
        self._mark_completed(dock_name)
        self._node.get_logger().info(f"[DispenserResult] SUCCESS @ {dock_name}")

    def _ingestor_result_cb(self, msg: IngestorResult):
        dock_name = msg.source_guid
        self._mark_completed(dock_name)
        self._node.get_logger().info(f"[IngestorResult] SUCCESS @ {dock_name}")

    def _mark_completed(self, dock_name):
        with self._process_lock:
            if dock_name not in self._dock_cmds:
                return
            for cmd_id in self._dock_cmds[dock_name]:
                self._process_status[cmd_id] = True

    # ----------------------------------------------------------------------
    def requires_replan(self, *a, **k): return False
    def toggle_action(self, *a, **k): return True
    def stop(self, *a, **k): return True
