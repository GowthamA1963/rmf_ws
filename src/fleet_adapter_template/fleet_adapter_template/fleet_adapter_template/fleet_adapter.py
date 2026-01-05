# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import argparse
import yaml
import time
import threading
import datetime

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType
from rmf_fleet_msgs.msg import LaneRequest, ClosedLanes

from functools import partial

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

from .RobotCommandHandle import RobotCommandHandle
from .RobotClientAPI import RobotAPI


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time):
    # Profile and traits
    fleet_config = config_yaml['rmf_fleet']
    profile = traits.Profile(
        geometry.make_final_convex_circle(fleet_config['profile']['footprint']),
        geometry.make_final_convex_circle(fleet_config['profile']['vicinity'])
    )
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*fleet_config['limits']['linear']),
        angular=traits.Limits(*fleet_config['limits']['angular']),
        profile=profile
    )
    vehicle_traits.differential.reversible = fleet_config['reversible']

    # Battery system
    voltage = fleet_config['battery_system']['voltage']
    capacity = fleet_config['battery_system']['capacity']
    charging_current = fleet_config['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(
        voltage, capacity, charging_current)

    # Mechanical system
    mass = fleet_config['mechanical_system']['mass']
    moment = fleet_config['mechanical_system']['moment_of_inertia']
    friction = fleet_config['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

    # Power systems
    ambient_power_sys = battery.PowerSystem.make(
        fleet_config['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        fleet_config['tool_system']['power'])

    # Power sinks
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(
        battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    # Navigation graph
    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    # Adapter
    fleet_name = fleet_config['name']
    adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')
    if use_sim_time:
        adapter.node.use_sim_time()
    assert adapter, (
        "Unable to initialize fleet adapter. Please ensure "
        "RMF Schedule Node is running"
    )
    adapter.start()
    time.sleep(1.0)

    # We are not using an HTTP fleet manager, so server_uri is always None
    server_uri = "ws://localhost:8000/_internal"

    fleet_handle = adapter.add_fleet(
        fleet_name, vehicle_traits, nav_graph, server_uri)

    fleet_state_update_frequency = fleet_config['publish_fleet_state']
    fleet_handle.fleet_state_publish_period(
        datetime.timedelta(seconds=1.0 / fleet_state_update_frequency))

    # Account for battery drain
    drain_battery = fleet_config['account_for_battery_drain']
    lane_merge_distance = fleet_config.get('lane_merge_distance', 0.1)
    recharge_threshold = fleet_config['recharge_threshold']
    recharge_soc = fleet_config['recharge_soc']
    finishing_request = fleet_config['task_capabilities']['finishing_request']
    node.get_logger().info(f"Finishing request: [{finishing_request}]")

    # Set task planner params
    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        finishing_request)
    assert ok, "Unable to set task planner params"

    # ----------------------------------------------------------------------
    # Task acceptance
    # ----------------------------------------------------------------------
    # Accept ALL tasks (loop, delivery, patrol, clean, etc.) while debugging.
    # Accept ALL tasks (loop, delivery, patrol, clean, etc.) while debugging.
    def _task_request_check(msg: TaskProfile):
        node.get_logger().info(
            f"Fleet [{fleet_name}] received task request "
            f"id=[{msg.task_id}] type=[{msg.description.task_type}]"
        )
        return True

    # Keep callback alive
    node.task_request_check = _task_request_check
    fleet_handle.accept_task_requests(_task_request_check)

    # ----------------------------------------------------------------------
    # Teleop action configuration
    # ----------------------------------------------------------------------
    def _consider(description: dict):
        confirm = adpt.fleet_update_handle.Confirmation()
        confirm.accept()
        return confirm

    # Keep callback alive
    node.consider_teleop = _consider
    # Configure this fleet to perform any kind of teleop action
    fleet_handle.add_performable_action("teleop", _consider)

    def _updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.update_handle = update_handle

        def _action_executor(category: str,
                             description: dict,
                             execution:
                             adpt.robot_update_handle.ActionExecution):
            with cmd_handle._lock:
                if len(description) > 0 and \
                        description in cmd_handle.graph.keys:
                    cmd_handle.action_waypoint_index = \
                        cmd_handle.find_waypoint(description).index
                else:
                    cmd_handle.action_waypoint_index = \
                        cmd_handle.last_known_waypoint_index
                cmd_handle.on_waypoint = None
                cmd_handle.on_lane = None
                cmd_handle.action_execution = execution

        # Set the action_executioner for the robot
        cmd_handle.update_handle.set_action_executor(_action_executor)

        if "max_delay" in cmd_handle.config.keys():
            max_delay = cmd_handle.config["max_delay"]
            cmd_handle.node.get_logger().info(
                f"Setting max delay to {max_delay}s")
            cmd_handle.update_handle.set_maximum_delay(max_delay)

        if (cmd_handle.charger_waypoint_index <
                cmd_handle.graph.num_waypoints):
            cmd_handle.update_handle.set_charger_waypoint(
                cmd_handle.charger_waypoint_index)
        else:
            cmd_handle.node.get_logger().warn(
                "Invalid waypoint supplied for charger. "
                "Using default nearest charger in the map")

    # ----------------------------------------------------------------------
    # Robot API (still used by RobotCommandHandle for navigation/stop/etc.)
    # ----------------------------------------------------------------------
    # NOTE: Our ROS-based RobotAPI can ignore these HTTP fields internally.

    # Extract robot names from config
    robot_names = list(config_yaml['robots'].keys())
    node.get_logger().info(f"Initializing RobotAPI for robots: {robot_names}")

    api = RobotAPI(node, robot_names)

    # ----------------------------------------------------------------------
    # Initialize robots directly from config (no HTTP discovery)
    # ----------------------------------------------------------------------
    robots = {}

    for robot_name, robots_config in config_yaml['robots'].items():
        node.get_logger().info(f"Initializing robot from config: {robot_name}")
        rmf_config = robots_config['rmf_config']
        robot_config = robots_config['robot_config']

        initial_waypoint_name = rmf_config['start']['waypoint']
        initial_orientation = rmf_config['start']['orientation']
        map_name = rmf_config['start']['map_name']

        wp = nav_graph.find_waypoint(initial_waypoint_name)
        if wp is None:
            node.get_logger().error(
                f"Initial waypoint [{initial_waypoint_name}] for robot "
                f"[{robot_name}] not found in nav graph"
            )
            continue

        time_now = adapter.now()
        start = plan.Start(time_now, wp.index, initial_orientation)

        # Initial position: use waypoint location + configured orientation
        wp_loc = wp.location  # (x, y)
        position = [wp_loc[0], wp_loc[1], initial_orientation]

        robot = RobotCommandHandle(
            name=robot_name,
            fleet_name=fleet_name,
            config=robot_config,
            node=node,
            graph=nav_graph,
            vehicle_traits=vehicle_traits,
            map_name=map_name,
            start=start,
            position=position,
            charger_waypoint=rmf_config['charger']['waypoint'],
            update_frequency=rmf_config.get(
                'robot_state_update_frequency', 1),
            lane_merge_distance=lane_merge_distance,
            adapter=adapter,
            api=api)

        if robot.initialized:
            robots[robot_name] = robot
            fleet_handle.add_robot(
                robot,
                robot_name,
                profile,
                [start],
                partial(_updater_inserter, robot)
            )
            node.get_logger().info(
                f"Successfully added new robot: {robot_name}")
        else:
            node.get_logger().error(
                f"Failed to initialize robot: {robot_name}")

    # ----------------------------------------------------------------------
    # Lane closure handling (unchanged)
    # ----------------------------------------------------------------------
    closed_lanes = []

    def _lane_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name:
            return

        fleet_handle.open_lanes(msg.open_lanes)
        fleet_handle.close_lanes(msg.close_lanes)

        newly_closed_lanes = []

        for lane_idx in msg.close_lanes:
            if lane_idx not in closed_lanes:
                newly_closed_lanes.append(lane_idx)
                closed_lanes.append(lane_idx)

        for lane_idx in msg.open_lanes:
            if lane_idx in closed_lanes:
                closed_lanes.remove(lane_idx)

        for r_name, robot in robots.items():
            robot.newly_closed_lanes(newly_closed_lanes)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = closed_lanes
        closed_lanes_pub.publish(state_msg)

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL)

    node.create_subscription(
        LaneRequest,
        'lane_closure_requests',
        _lane_request_cb,
        qos_profile=qos_profile_system_default)

    closed_lanes_pub = node.create_publisher(
        ClosedLanes,
        'closed_lanes',
        qos_profile=transient_qos)

    return adapter


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    adapter = initialize_fleet(
        config_yaml,
        nav_graph_path,
        node,
        args.use_sim_time)

    # NOTE: In initialize_fleet, we now return (adapter, fleet_handle) to keep
    # the python object alive. But if we only need adapter to spin, we can just
    # unpack it or modify initialize_fleet slightly.
    # To minimize diffs and ensure safety, let's just make the callbacks persistent
    # INSIDE initialize_fleet by attaching them to the adapter or node.

    # Wait, I will modify initialize_fleet to attach callbacks to adapter.

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
