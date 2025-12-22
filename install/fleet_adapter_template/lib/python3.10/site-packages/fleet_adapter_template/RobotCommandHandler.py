
from rclpy.duration import Duration
import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule
from rmf_fleet_msgs.msg import DockSummary

import numpy as np
import threading
import math
import copy
import enum
import time
from datetime import timedelta


class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api):

        super().__init__()

        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        self.adapter = adapter
        self.api = api

        # Charger waypoint index
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} does not exist"
        self.charger_waypoint_index = waypoint.index
        self.charger_is_set = False

        # Robot state
        self.position = position
        self.battery_soc = 1.0
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""

        # RMF waypoint tracking
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        self.on_waypoint = None
        self.on_lane = None
        self.target_waypoint = None
        self.dock_waypoint_index = None

        # RMF path execution state
        self.requested_waypoints = []
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0

        # Threading
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        self.node.get_logger().info(
            f"Robot starts at {self.position}")

        # Start at correct RMF location
        if start.lane is not None:
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        # Update timer
        self.update_frequency = update_frequency
        self.update_handle = None
        self.state_update_timer = self.node.create_timer(
            1.0 / self.update_frequency, self.update)

        self.initialized = True

    # Utility sleep
    def sleep_for(self, seconds):
        goal_time = self.node.get_clock().now() + Duration(seconds=seconds)
        while self.node.get_clock().now() <= goal_time:
            time.sleep(0.001)

    def clear(self):
        with self._lock:
            self.requested_waypoints = []
            self.remaining_waypoints = []
            self.path_finished_callback = None
            self.next_arrival_estimator = None
            self.docking_finished_callback = None
            self.state = RobotState.IDLE

    # STOP robot
    def stop(self):
        while True:
            self.node.get_logger().info("Requesting stop...")
            if self.api.stop(self.name):
                break
            self.sleep_for(0.1)

        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()

    # FOLLOW PATH
    def follow_new_path(self, waypoints, next_arrival_estimator, path_finished_callback):

        self.stop()
        self._quit_path_event.clear()

        self.remaining_waypoints = self.get_remaining_waypoints(waypoints)
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        def _follow_path():

            target_pose = []

            while self.remaining_waypoints or \
                    self.state in (RobotState.MOVING, RobotState.WAITING):

                if self._quit_path_event.is_set():
                    self.node.get_logger().info("Aborting path")
                    return

                if self.state == RobotState.IDLE:

                    # Get the next RMF waypoint
                    self.target_waypoint = self.remaining_waypoints[0][1]
                    self.path_index = self.remaining_waypoints[0][0]

                    target_pose = self.target_waypoint.position

                    # Convert RMF → robot coords
                    x_robot, y_robot = self.transforms["rmf_to_robot"].transform(
                        target_pose[:2]
                    )
                    theta = target_pose[2] + self.transforms['orientation_offset']

                    # Send Nav2 goal
                    response = self.api.navigate(
                        self.name,
                        [x_robot, y_robot, theta],
                        self.map_name
                    )

                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = RobotState.MOVING
                    else:
                        self.node.get_logger().warn(
                            f"Navigation to {x_robot:.2f},{y_robot:.2f} failed. Retrying..."
                        )
                        self.sleep_for(0.1)

                elif self.state == RobotState.WAITING:
                    self.sleep_for(0.1)
                    time_now = self.adapter.now()
                    waypoint_wait_time = self.target_waypoint.time

                    if waypoint_wait_time < time_now:
                        self.state = RobotState.IDLE
                    else:
                        wait_duration = (waypoint_wait_time - time_now).seconds
                        self.node.get_logger().info(
                            f"Waiting {wait_duration}s"
                        )
                        self.next_arrival_estimator(self.path_index, timedelta(seconds=0))

                elif self.state == RobotState.MOVING:
                    self.sleep_for(0.1)

                    if self.api.navigation_completed(self.name):
                        self.node.get_logger().info(
                            f"{self.name} reached its waypoint"
                        )
                        self.state = RobotState.WAITING
                        if self.target_waypoint.graph_index is not None:
                            self.on_waypoint = self.target_waypoint.graph_index
                            self.last_known_waypoint_index = self.on_waypoint
                        else:
                            self.on_waypoint = None

                    else:
                        # Estimate remaining duration
                        dist = self.dist(self.position, target_pose)
                        v = self.vehicle_traits.linear.nominal_velocity
                        if v <= 0.01:
                            duration = 0.0
                        else:
                            duration = dist / v

                        if self.path_index is not None:
                            self.next_arrival_estimator(
                                self.path_index,
                                timedelta(seconds=duration)
                            )

            self.path_finished_callback()
            self.node.get_logger().info("Path completed")

        self._follow_path_thread = threading.Thread(target=_follow_path)
        self._follow_path_thread.start()

    # DOCKING
    def dock(self, dock_name, docking_finished_callback):

        self._quit_dock_event.clear()
        if self._dock_thread:
            self._dock_thread.join()

        self.dock_name = dock_name
        self.docking_finished_callback = docking_finished_callback

        dock_waypoint = self.graph.find_waypoint(self.dock_name)
        assert dock_waypoint
        self.dock_waypoint_index = dock_waypoint.index

        def _dock():

            self.node.get_logger().info(
                f"Docking at {self.dock_name} (external charger handles motion)"
            )

            # Robot already goes to charger via external node
            self.sleep_for(1.0)

            self.node.get_logger().info("Docking complete")
            self.docking_finished_callback()

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    # GET POSITION
    def get_position(self):
        position = self.api.position(self.name)
        if position is None:
            self.node.get_logger().error("Position unavailable")
            return self.position

        x_r, y_r, theta_r = position

        # Robot → RMF frame transform
        x, y = self.transforms['robot_to_rmf'].transform([x_r, y_r])
        theta = theta_r - self.transforms['orientation_offset']

        if theta > np.pi:
            theta -= 2 * np.pi
        if theta < -np.pi:
            theta += 2 * np.pi

        return [x, y, theta]

    # BATTERY
    def get_battery_soc(self):
        soc = self.api.battery_soc(self.name)
        if soc is None:
            return self.battery_soc
        return soc

    # UPDATE
    def update(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        if self.update_handle:
            self.update_state()

    # UPDATE STATE TO RMF
    def update_state(self):

        self.update_handle.update_battery_soc(self.battery_soc)

        if not self.charger_is_set:
            if "max_delay" in self.config:
                self.update_handle.set_maximum_delay(self.config["max_delay"])

            self.update_handle.set_charger_waypoint(
                self.charger_waypoint_index
            )
            self.charger_is_set = True

        with self._lock:
            if self.on_waypoint is not None:
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2]
                )
            elif self.on_lane is not None:
                forward_lane = self.graph.get_lane(self.on_lane)
                entry = forward_lane.entry.waypoint_index
                exit = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit, entry)
                lane_indices = [self.on_lane]
                if reverse_lane:
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(self.position, lane_indices)
            else:
                self.update_handle.update_off_grid_position(
                    self.position, self.map_name
                )

    # LANE CHECK
    def get_current_lane(self):

        def projection(pose, target, lane_entry, lane_exit):
            p = np.array([pose[0], pose[1]])
            t = np.array(target)
            e = np.array(lane_entry)
            ex = np.array(lane_exit)
            return np.dot(p - t, ex - e)

        if self.target_waypoint is None:
            return None

        approach_lanes = self.target_waypoint.approach_lanes
        if not approach_lanes:
            return None

        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            entry = self.graph.get_waypoint(lane.entry.waypoint_index).location
            exit = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            if not (projection(p, entry, entry, exit) < 0 or
                    projection(p, exit, entry, exit) >= 0):
                return lane_index
        return None

    def dist(self, A, B):
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def get_remaining_waypoints(self, waypoints):
        return [(i, waypoints[i]) for i in range(len(waypoints))]
