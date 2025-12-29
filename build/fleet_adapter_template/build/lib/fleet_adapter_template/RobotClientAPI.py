# RobotClientAPI.py
#
# ROS-based Robot API used by RobotCommandHandle.
# - Uses /amcl_pose for position
# - Ignores battery topic for now and returns 100% SOC
# - Uses Nav2 NavigateToPose for navigation goals
# - Minimal stubs for process/toggle_action/etc.

import math
import threading
from typing import Dict, Optional

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

import threading
import math
class RobotAPI:
    def __init__(self, node: Node, robot_names: list):
        """
        Initialize RobotAPI for multiple robots.
        
        Args:
            node: ROS 2 node to use for subscriptions and action clients
            robot_names: List of robot names (e.g., ['robot1', 'robot2'])
        """
        # IMPORTANT: reuse the existing fleet node.
        # DO NOT create a new rclpy node here, and DO NOT call rclpy.init() here.
        self._node = node
        self._robot_names = robot_names

        # Per-robot pose storage
        self._pose_lock = threading.Lock()
        self._poses: Dict[str, Optional[PoseWithCovarianceStamped]] = {}
        self._logged_pose_once: Dict[str, bool] = {}

        # Per-robot Nav2 action clients and goal results
        self._nav_clients: Dict[str, ActionClient] = {}
        self._goal_results: Dict[str, Dict[int, str]] = {}  # robot_name -> {cmd_id: status}

        # QoS profile for AMCL pose - optimized for localization data
        # BEST_EFFORT: Don't wait for retransmissions, accept latest data
        # VOLATILE: Don't store historical data, only current pose matters
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribe to each robot's AMCL pose topic
        for robot_name in robot_names:
            self._poses[robot_name] = None
            self._logged_pose_once[robot_name] = False
            
            topic = f'/{robot_name}/amcl_pose'
            self._node.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                lambda msg, name=robot_name: self._amcl_cb(msg, name),
                amcl_qos
            )
            self._node.get_logger().info(
                f"[RobotAPI] Subscribed to {topic} with QoS profile"
            )

        self._node.get_logger().info(
            f"RobotAPI initialized for {len(robot_names)} robot(s): {robot_names}"
        )

    # ----------------------------------------------------------------------
    # Subscribers
    # ----------------------------------------------------------------------

    def _amcl_cb(self, msg: PoseWithCovarianceStamped, robot_name: str):
        """Callback for AMCL pose updates, stores pose per robot."""
        with self._pose_lock:
            self._poses[robot_name] = msg
    
        if not self._logged_pose_once.get(robot_name, False):
            self._logged_pose_once[robot_name] = True
            self._node.get_logger().info(
                f"[RobotAPI] [{robot_name}] Received first amcl_pose: "
                f"({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})"
            )

    # ----------------------------------------------------------------------
    # Position / battery API used by RobotCommandHandle
    # ----------------------------------------------------------------------

    def position(self, robot_name: str):
        """Return [x, y, yaw] in map frame for the specified robot, or None if not available yet."""
        with self._pose_lock:
            pose = self._poses.get(robot_name)
            if pose is None:
                self._node.get_logger().warn(
                    f"[RobotAPI] position() called for [{robot_name}] but amcl_pose not received yet"
                )
                return None
            
            p = pose.pose.pose.position
            q = pose.pose.pose.orientation

        # quaternion -> yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [p.x, p.y, yaw]

    def battery_soc(self, robot_name: str) -> float:
        """
        Return SOC in [0.0, 1.0].

        We ignore the real battery topic for now because /battery_percentage
        has conflicting message types in your system. To avoid errors and
        let RMF operate, we just report 100% battery.
        """
        return 1.0

    # ----------------------------------------------------------------------
    # Nav2 NavigateToPose integration
    # ----------------------------------------------------------------------

    def _get_nav_client(self, robot_name: str) -> ActionClient:
        """Get or create a Nav2 action client for the specified robot."""
        if robot_name not in self._nav_clients:
            topic = f'/{robot_name}/navigate_to_pose'
            
            # QoS profile for Nav2 action client - reliable command delivery
            # RELIABLE: Ensure navigation commands are delivered
            # VOLATILE: Don't need historical goals, only current one
            nav_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            self._nav_clients[robot_name] = ActionClient(
                self._node, NavigateToPose, topic, goal_service_qos_profile=nav_qos)
            self._goal_results[robot_name] = {}
            self._node.get_logger().info(
                f"[RobotAPI] Created Nav2 client for [{robot_name}] on [{topic}] with QoS profiles"
            )
        return self._nav_clients[robot_name]

    def navigate(self,
                 robot_name: str,
                 cmd_id: int,
                 pose,
                 map_name: str,
                 speed_limit: float = 0.0) -> bool:
        """
        Request robot to navigate to pose [x, y, yaw] in map frame.
        Return True if the request was sent (goal will be accepted or rejected async).
        """
        assert len(pose) >= 3
        x, y, yaw = pose[0], pose[1], pose[2]

        client = self._get_nav_client(robot_name)

        if not client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().error(
                f"[RobotAPI] Nav2 action server not available for [{robot_name}]"
            )
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # yaw -> quaternion (z,w)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._node.get_logger().info(
            f"[RobotAPI] [{robot_name}] Nav2 goal cmd_id={cmd_id} "
            f"({x:.2f}, {y:.2f}, yaw={yaw:.2f}), speed_limit={speed_limit}"
        )

        send_future = client.send_goal_async(goal)

        def goal_response_cb(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self._node.get_logger().error(
                    f"[RobotAPI] Nav2 goal REJECTED for [{robot_name}], cmd_id={cmd_id}"
                )
                self._goal_results[robot_name][cmd_id] = 'rejected'
                return

            self._node.get_logger().info(
                f"[RobotAPI] Nav2 goal accepted for [{robot_name}], cmd_id={cmd_id}"
            )

            result_future = goal_handle.get_result_async()

            def result_cb(res_fut):
                result = res_fut.result()
                status_code = getattr(result, 'status', None)
                if status_code == 4:  # SUCCEEDED
                    status_str = 'succeeded'
                else:
                    status_str = f'status_{status_code}'
                self._goal_results[robot_name][cmd_id] = status_str
                self._node.get_logger().info(
                    f"[RobotAPI] Nav2 result for [{robot_name}], "
                    f"cmd_id={cmd_id}: {status_str}"
                )

            result_future.add_done_callback(result_cb)

        send_future.add_done_callback(goal_response_cb)
        return True

    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        """We don't estimate remaining duration → return None."""
        return None

    def navigation_completed(self, robot_name: str, cmd_id: int) -> bool:
        """
        Return True if Nav2 reported success for this cmd_id.
        RobotCommandHandle polls this.
        """
        robot_results = self._goal_results.get(robot_name, {})
        status = robot_results.get(cmd_id)
        return status == 'succeeded'

    # ----------------------------------------------------------------------
    # Replan / process / toggle stubs
    # ----------------------------------------------------------------------

    def requires_replan(self, robot_name: str) -> bool:
        return False

    def start_process(self,
                      robot_name: str,
                      cmd_id: int,
                      process: str,
                      map_name: str) -> bool:
        self._node.get_logger().info(
            f"[RobotAPI] start_process '{process}' for [{robot_name}], "
            f"cmd_id={cmd_id} (stubbed as immediate success)"
        )
        return True

    def process_completed(self, robot_name: str, cmd_id: int) -> bool:
        return True

    def toggle_action(self, robot_name: str, toggle: bool) -> bool:
        self._node.get_logger().info(
            f"[RobotAPI] toggle_action={toggle} for [{robot_name}] (stub)"
        )
        return True

    def stop(self, robot_name: str, cmd_id: int) -> bool:
        self._node.get_logger().info(
            f"[RobotAPI] stop() requested for [{robot_name}], cmd_id={cmd_id} (stub)"
        )
        return True
