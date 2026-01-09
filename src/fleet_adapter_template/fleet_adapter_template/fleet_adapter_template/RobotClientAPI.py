# RobotClientAPI.py
#
# ROS-based Robot API used by RobotCommandHandle.
# - Uses /amcl_pose for position
# - Ignores battery topic for now and returns 100% SOC
# - Uses Nav2 NavigateToPose for navigation goals
# - Minimal stubs for process/toggle_action/etc.

import math
import threading
import csv
from typing import Dict, Optional
from pathlib import Path

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
class RobotAPI:
    def __init__(self, node: Node, robot_names: list, waypoint_csv_path: str = None):
        """
        Initialize RobotAPI for multiple robots.
        
        Args:
            node: ROS 2 node to use for subscriptions and action clients
            robot_names: List of robot names (e.g., ['robot1', 'robot2'])
            waypoint_csv_path: Optional path to CSV file with waypoint orientations
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
        
        # Load complete waypoint data from CSV (x, y, yaw)
        self._waypoint_data: Dict[str, tuple] = {}  # waypoint_name -> (x, y, yaw)
        self._load_waypoint_data(waypoint_csv_path)

        # QoS profile for AMCL pose - optimized for localization data
        # BEST_EFFORT: Don't wait for retransmissions, accept latest data
        # VOLATILE: Don't store historical data, only current pose matters
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        None
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
        if self._waypoint_data:
            self._node.get_logger().info(
                f"Loaded complete data for {len(self._waypoint_data)} waypoints from CSV"
            )

    # ----------------------------------------------------------------------
    # CSV Orientation Loading
    # ----------------------------------------------------------------------
    
    def _load_waypoint_data(self, csv_path: str = None):
        """Load complete waypoint data (x, y, orientation) from CSV file."""
        if csv_path is None:
            # Try default location
            csv_path = "/home/robot1/rmf_ws/src/fleet_adapter_template/fleet_adapter_template/fleet_adapter_template/final_goals.csv"
        else:
            csv_path = Path(csv_path)
        
        if not csv_path:
            self._node.get_logger().warn(
                f"Waypoint data CSV not found at {csv_path}. "
                "Using coordinates from navigation graph."
            )
            return
        
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                
                if not all(col in reader.fieldnames for col in ['Table', 'x', 'y', 'z', 'w']):
                    self._node.get_logger().error(
                        "CSV must have columns: Table, x, y, z, w"
                    )
                    return
                
                for row in reader:
                    name = row['Table'].strip()
                    
                    if not name or name.lower() in ['table', 'name']:
                        continue
                    
                    try:
                        x = float(row['x'])
                        y = float(row['y'])
                        z = float(row['z'])
                        w = float(row['w'])
                        
                        # Convert quaternion (z, w) to yaw
                        yaw = 2.0 * math.atan2(z, w)
                        
                        # Normalize to [-π, π]
                        while yaw > math.pi:
                            yaw -= 2 * math.pi
                        while yaw < -math.pi:
                            yaw += 2 * math.pi
                        
                        # Store complete waypoint data
                        self._waypoint_data[name] = (x, y, yaw)
                        
                    except (ValueError, KeyError) as e:
                        self._node.get_logger().warn(
                            f"Skipping waypoint '{name}': {e}"
                        )
                        continue
            
            self._node.get_logger().info(
                f"Loaded complete data for {len(self._waypoint_data)} waypoints from {csv_path}"
            )
            
        except Exception as e:
            self._node.get_logger().error(
                f"Error loading waypoint data from CSV: {e}"
            )
    
    def _match_waypoint_by_position(self, x: float, y: float, tolerance: float = 1.0) -> Optional[str]:
        """Try to match a waypoint by position proximity to CSV data."""
        for name, (csv_x, csv_y, csv_yaw) in self._waypoint_data.items():
            distance = math.sqrt((x - csv_x)**2 + (y - csv_y)**2)
            if distance < tolerance:
                self._node.get_logger().info(
                    f"[RobotAPI] 🔍 Position ({x:.2f}, {y:.2f}) matched to '{name}' "
                    f"at ({csv_x:.2f}, {csv_y:.2f}), distance={distance:.3f}m"
                )
                return name
        
        # Log available waypoints if no match found
        self._node.get_logger().debug(
            f"[RobotAPI] No position match found for ({x:.2f}, {y:.2f}). "
            f"Available waypoints: {list(self._waypoint_data.keys())}"
        )
        return None
    
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
                 speed_limit: float = 0.0,
                 waypoint_name: str = None) -> bool:
        """
        Request robot to navigate to pose [x, y, yaw] in map frame.
        If waypoint_name is provided and exists in CSV, use CSV orientation instead of pose[2].
        Return True if the request was sent (goal will be accepted or rejected async).
        """
        assert len(pose) >= 3
        x, y, yaw = pose[0], pose[1], pose[2]
        
        # Debug: Log waypoint name and CSV availability
        self._node.get_logger().info(
            f"[RobotAPI] navigate() called with waypoint_name='{waypoint_name}', "
            f"nav_graph_pose=({x:.2f}, {y:.2f}, {yaw:.3f})"
        )
        
        # If no waypoint name, try to match by position
        if not waypoint_name and self._waypoint_data:
            matched_name = self._match_waypoint_by_position(x, y)
            if matched_name:
                waypoint_name = matched_name
                self._node.get_logger().info(
                    f"[RobotAPI] 🔍 Matched waypoint by position: '{waypoint_name}'"
                )
        
        # Override ALL coordinates with CSV data if available
        if waypoint_name and waypoint_name in self._waypoint_data:
            csv_x, csv_y, csv_yaw = self._waypoint_data[waypoint_name]
            self._node.get_logger().info(
                f"[RobotAPI] ✅ Using CSV data for '{waypoint_name}': "
                f"pos=({csv_x:.2f}, {csv_y:.2f}), yaw={csv_yaw:.3f} rad ({math.degrees(csv_yaw):.1f}°) "
                f"[nav graph was: ({x:.2f}, {y:.2f}), {yaw:.3f}]"
            )
            x, y, yaw = csv_x, csv_y, csv_yaw
        elif waypoint_name:
            self._node.get_logger().warn(
                f"[RobotAPI] ⚠️  Waypoint '{waypoint_name}' NOT found in CSV. "
                f"Using nav graph coordinates. Available waypoints: {list(self._waypoint_data.keys())[:5]}..."
            )
        else:
            self._node.get_logger().warn(
                f"[RobotAPI] ⚠️  No waypoint name provided and no position match found. "
                f"Using nav graph coordinates: ({x:.2f}, {y:.2f}, {yaw:.3f})"
            )

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
