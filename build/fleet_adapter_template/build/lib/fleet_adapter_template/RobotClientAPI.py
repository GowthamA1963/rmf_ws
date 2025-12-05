import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class RobotAPI(Node):

    def __init__(self, config_yaml):
        super().__init__('robot_api')

        self.prefix = config_yaml['prefix']
        self.user = config_yaml['user']
        self.password = config_yaml['password']

        # -----------------------
        # Internal storage
        # -----------------------
        self.current_pose = None      # [x, y, theta]
        self.current_soc = None       # 0.0–1.0
        self.current_goal_handle = None
        self.goal_in_progress = False
        self.result_future = None

        # -----------------------
        # Subscribers
        # -----------------------
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/battery_percentage',
            self.battery_callback,
            10
        )

        # -----------------------
        # Nav2 Action Client
        # -----------------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

    # ------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------
    def amcl_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.current_pose = [x, y, yaw]

    def battery_callback(self, msg):
        # Convert % to SOC (0.0–1.0)
        self.current_soc = msg.data / 100.0

    # ------------------------------------------------------------
    # REQUIRED API FUNCTIONS
    # ------------------------------------------------------------
    def check_connection(self):
        return True  # always true for ROS setup

    def navigate(self, robot_name: str, pose, map_name: str, speed_limit=0.0):

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server unavailable")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]

        q = quaternion_from_euler(0, 0, pose[2])
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        self.current_goal_handle = send_goal_future.result()

        if not self.current_goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            return False

        self.goal_in_progress = True
        self.result_future = self.current_goal_handle.get_result_async()

        return True

    def stop(self, robot_name: str):
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.goal_in_progress = False
            return True
        return False

    def is_command_completed(self):
        if not self.goal_in_progress:
            return True

        if self.result_future and self.result_future.done():
            result = self.result_future.result()
            if result.status == 4:  # succeeded
                self.goal_in_progress = False
                return True

        return False

    def position(self, robot_name: str):
        return self.current_pose

    def battery_soc(self, robot_name: str):
        return self.current_soc

    def map(self, robot_name: str):
        return "L1"  # Single floor

    def get_data(self, robot_name: str):
        if self.current_pose is None or self.current_soc is None:
            return None
        return RobotUpdateData(robot_name, "L1", self.current_pose, self.current_soc)

    def navigation_completed(self, robot_name: str):
        return self.is_command_completed()



class RobotUpdateData:
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list,
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
