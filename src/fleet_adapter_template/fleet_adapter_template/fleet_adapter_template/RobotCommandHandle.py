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

import rclpy
import rclpy.node
from rclpy.duration import Duration

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary, ModeRequest

import numpy as np

import threading
import math
import enum
import time

from datetime import timedelta


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    MOVING = 1


# Custom wrapper for Plan::Waypoint. We use this to modify position of
# waypoints to prevent backtracking
class PlanWaypoint:
    def __init__(self, index, wp: plan.Waypoint):
        # the index of the Plan::Waypoint in the waypoints in follow_new_path
        self.index = index
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 lane_merge_distance,
                 adapter,
                 api):
        adpt.RobotCommandHandle.__init__(self)
        self.debug = False
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.map_name = map_name
        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} " \
                         f"does not exist in the navigation graph"
        self.charger_waypoint_index = waypoint.index
        self.update_frequency = update_frequency
        self.lane_merge_distance = lane_merge_distance
        self.update_handle = None  # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        self.position = position  # (x,y,theta) in RMF crs (meters,radians)
        self.initialized = False

        # Breakdown detection
        self.breakdown_detection = {
            'nav_failure_count': 0,
            'nav_failure_threshold': 3,
            'action_failure_count': 0,
            'action_failure_threshold': 2,
            'last_breakdown_check': None
        }
        self.breakdown_callback = None  # Set by fleet adapter
        self.state = RobotState.IDLE
        self.dock_name = ""
        # used for time comparison with Plan::Waypoint::time
        self.adapter = adapter
        self.action_execution = None

        self.requested_waypoints = []  # RMF Plan waypoints
        self.remaining_waypoints = []
        self.docks = {}

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        self.last_replan_time = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint = None  # this is a Plan::Waypoint
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None
        # The graph index of the waypoint the robot starts or ends an action
        self.action_waypoint_index = None
        self.current_cmd_id = 0
        self.started_action = False

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()
        self._stopping_thread = None
        self._quit_stopping_event = threading.Event()

        self.node.get_logger().info(
            f"The robot is starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.node.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        self.node.create_subscription(
            ModeRequest,
            'action_execution_notice',
            self.mode_request_cb,
            qos_profile=qos_profile_system_default)

        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.start()

        self.initialized = True

    # --------------------------------------------------------------------------
    # Utility helpers
    # --------------------------------------------------------------------------

    def next_cmd_id(self):
        self.current_cmd_id = self.current_cmd_id + 1
        if self.debug:
            print(f'Issuing cmd_id for {self.name}: {self.current_cmd_id}')
        return self.current_cmd_id

    def sleep_for(self, seconds):
        goal_time = (
            self.node.get_clock().now() + Duration(nanoseconds=1e9 * seconds)
        )
        while self.node.get_clock().now() <= goal_time:
            time.sleep(0.001)

    def wait_on(self, event: threading.Event, seconds):
        goal_time = (
            self.node.get_clock().now() + Duration(nanoseconds=1e9 * seconds)
        )
        while self.node.get_clock().now() <= goal_time:
            if event.wait(0.001):
                return True
        return False

    def clear(self):
        self.requested_waypoints = []
        self.remaining_waypoints = []
        self.state = RobotState.IDLE

    # --------------------------------------------------------------------------
    # Interrupt / stop / replan
    # --------------------------------------------------------------------------

    def interrupt(self):
        if self.debug:
            print(
                f'Interrupting {self.name} '
                f'(latest cmd_id is {self.current_cmd_id})'
            )
        self._quit_dock_event.set()
        self._quit_path_event.set()
        self._quit_stopping_event.set()

        if self._follow_path_thread is not None:
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()

        if self._dock_thread is not None:
            if self._dock_thread.is_alive():
                self._dock_thread.join()

        if self._stopping_thread is not None:
            if self._stopping_thread.is_alive():
                self._stopping_thread.join()

    def stop(self):
        if self.debug:
            plan_id = self.update_handle.unstable_current_plan_id()
            print(f'stop for {self.name} with PlanId {plan_id}')

        self.interrupt()
        # Stop the robot. Tracking variables should remain unchanged.
        with self._lock:
            self._quit_stopping_event.clear()

            def _stop():
                while not self._quit_stopping_event.is_set():
                    self.node.get_logger().info(
                        f"Requesting {self.name} to stop..."
                    )
                    if self.api.stop(self.name, self.next_cmd_id()):
                        break
                    self._quit_stopping_event.wait(0.1)

            self._stopping_thread = threading.Thread(target=_stop)
            self._stopping_thread.start()

    def replan(self):
        if self.update_handle is not None:
            now = self.adapter.now()
            if self.last_replan_time is not None:
                # TODO(MXG): Make the 15s replan cooldown configurable
                if now - self.last_replan_time < timedelta(seconds=15.0):
                    return
            self.last_replan_time = now
            self.update_handle.replan()
            self.node.get_logger().info(
                f'Requesting replan for {self.name} because of an obstacle'
            )

    # --------------------------------------------------------------------------
    # follow_new_path: **updated signature**
    # --------------------------------------------------------------------------

    def follow_new_path(self, waypoints, *args):
        """
        Called by RMF when a new path is assigned to this robot.

        We support both of these call patterns:
          1) follow_new_path(waypoints, next_arrival_estimator, path_finished_callback)
          2) follow_new_path(waypoints, path_finished_callback)

        In case (2), we simply won't report arrival estimates.
        """

        # Unpack args depending on how many RMF passed
        next_arrival_estimator = None
        if len(args) == 2:
            next_arrival_estimator, path_finished_callback = args
        elif len(args) == 1:
            (path_finished_callback,) = args
        else:
            raise ValueError(
                f"follow_new_path expected 2 or 3 arguments after 'self', "
                f"got {1 + len(args)} total"
            )

        if self.debug:
            plan_id = self.update_handle.unstable_current_plan_id()
            print(f'follow_new_path for {self.name} with PlanId {plan_id}')

        self.interrupt()
        with self._lock:
            self._follow_path_thread = None
            self._quit_path_event.clear()
            self.clear()

            self.node.get_logger().info(f"Received new path for {self.name} with {len(waypoints)} waypoints")
            if len(waypoints) > 0:
                 wp_start = waypoints[0].position
                 wp_end = waypoints[-1].position
                 self.node.get_logger().info(f"  Path start: ({wp_start[0]:.2f}, {wp_start[1]:.2f}), End: ({wp_end[0]:.2f}, {wp_end[1]:.2f})")

            self.remaining_waypoints = self.filter_waypoints(waypoints)
            assert path_finished_callback is not None

            def _follow_path():
                target_pose = None
                path_index = 0
                while self.remaining_waypoints \
                        or self.state == RobotState.MOVING:
                    # Save the current_cmd_id before checking if we need to
                    # abort.
                    cmd_id = self.current_cmd_id

                    # Check if we need to abort
                    if self._quit_path_event.is_set():
                        self.node.get_logger().info(
                            f"[{self.name}] aborting path request"
                        )
                        return

                    # Check if robot is online before proceeding with any navigation logic
                    if not self.api.is_robot_online(self.name):
                        # Track when robot first went offline during this navigation
                        if not hasattr(self, '_offline_since'):
                            self._offline_since = {}
                        
                        if self.name not in self._offline_since:
                            self._offline_since[self.name] = self.node.get_clock().now()
                            self.node.get_logger().warn(
                                f"[{self.name}] Robot is OFFLINE during navigation. Will abort if offline for 30s..."
                            )
                        else:
                            offline_duration = (self.node.get_clock().now() - self._offline_since[self.name]).nanoseconds / 1e9
                            if offline_duration > 30.0:  # 30 second timeout
                                self.node.get_logger().error(
                                    f"[{self.name}] Robot offline for {offline_duration:.1f}s. Aborting navigation."
                                )
                                return  # Exit the _follow_path thread
                            
                            self.node.get_logger().warn(
                                f"[{self.name}] Robot OFFLINE for {offline_duration:.1f}s. Pausing...",
                                throttle_duration_sec=5.0
                            )
                        self.sleep_for(1.0)
                        continue
                    else:
                        # Robot is back online - clear offline timer
                        if hasattr(self, '_offline_since') and self.name in self._offline_since:
                            del self._offline_since[self.name]

                    # State machine
                    if self.state == RobotState.IDLE or target_pose is None:
                        # Assign the next waypoint
                        self.target_waypoint = self.remaining_waypoints[0]
                        path_index = self.remaining_waypoints[0].index

                        # Check if this is the FINAL waypoint in the path
                        is_final_waypoint = (len(self.remaining_waypoints) == 1)

                        if not is_final_waypoint:
                            # This is an intermediate waypoint - skip navigation
                            # Just mark it as completed and move to the next one
                            self.node.get_logger().info(
                                f"[{self.name}] ⏭️  Skipping intermediate waypoint at "
                                f"path index {path_index} (graph_index: {self.target_waypoint.graph_index}). "
                                f"Remaining waypoints: {len(self.remaining_waypoints)}"
                            )

                            # Update tracking to reflect we've passed this waypoint
                            graph_index = self.target_waypoint.graph_index
                            if graph_index is not None:
                                self.last_known_waypoint_index = graph_index

                            # Remove this waypoint and continue to next
                            self.remaining_waypoints = self.remaining_waypoints[1:]
                            # Stay in IDLE state to process next waypoint immediately
                            continue

                        # This is the FINAL waypoint - send navigation command
                        self.node.get_logger().info(
                            f"[{self.name}] 🎯 Final waypoint detected at path index {path_index}. "
                            f"Sending navigation command."
                        )

                        # Move robot to next waypoint
                        target_pose = self.target_waypoint.position
                        [x, y] = target_pose[:2]
                        theta = target_pose[2]
                        speed_limit = \
                            self.get_speed_limit(self.target_waypoint)

                        # Get waypoint name if available
                        waypoint_name = None
                        if self.target_waypoint.graph_index is not None:
                            wp = self.graph.get_waypoint(self.target_waypoint.graph_index)
                            if wp:
                                # Try multiple methods to extract name
                                # Method 1: Direct name attribute
                                if hasattr(wp, 'name') and wp.name:
                                    waypoint_name = wp.name
                                # Method 2: Check in properties dict
                                elif hasattr(wp, 'properties') and hasattr(wp.properties, 'name'):
                                    waypoint_name = wp.properties.name
                                # Method 3: Try to get name from graph keys
                                elif hasattr(self.graph, 'keys') and self.target_waypoint.graph_index in self.graph.keys:
                                    waypoint_name = self.graph.keys[self.target_waypoint.graph_index]

                                if waypoint_name:
                                    self.node.get_logger().info(
                                        f"[{self.name}] Extracted waypoint name: '{waypoint_name}' "
                                        f"from graph_index {self.target_waypoint.graph_index}"
                                    )
                                else:
                                    self.node.get_logger().warn(
                                        f"[{self.name}] Waypoint at graph_index {self.target_waypoint.graph_index} "
                                        f"has no name attribute or empty name. Position: ({x:.2f}, {y:.2f}). "
                                        f"Will try position matching with CSV."
                                    )
                        else:
                            self.node.get_logger().info(
                                f"[{self.name}] No graph_index for target waypoint. "
                                f"Position: ({x:.2f}, {y:.2f})"
                            )

                        response = self.api.navigate(
                            self.name,
                            self.next_cmd_id(),
                            [x, y, theta],
                            self.map_name,
                            speed_limit,
                            waypoint_name
                        )

                        if response:
                            self.remaining_waypoints = \
                                self.remaining_waypoints[1:]
                            self.state = RobotState.MOVING
                        else:
                            self.node.get_logger().info(
                                f"Robot {self.name} failed to request "
                                f"navigation to "
                                f"[{x:.0f}, {y:.0f}, {theta:.0f}]. "
                                f"Retrying..."
                            )
                            self._handle_navigation_failure()
                            self._quit_path_event.wait(0.1)

                    elif self.state == RobotState.MOVING:
                        if self.api.requires_replan(self.name):
                            self.replan()

                        if self._quit_path_event.wait(0.1):
                            return

                        # Check if we have reached the target
                        with self._lock:
                            if self.api.navigation_completed(
                                    self.name, cmd_id):
                                self.node.get_logger().info(
                                    f"Robot [{self.name}] has reached the "
                                    f"destination for cmd_id {cmd_id}"
                                )
                                self._handle_navigation_success()
                                self.state = RobotState.IDLE
                                graph_index = self.target_waypoint.graph_index
                                if graph_index is not None:
                                    self.on_waypoint = graph_index
                                    self.last_known_waypoint_index = \
                                        graph_index
                                else:
                                    self.on_waypoint = None  # still on a lane
                            else:
                                # Update the lane the robot is on
                                lane = self.get_current_lane()
                                if lane is not None:
                                    self.on_waypoint = None
                                    self.on_lane = lane
                                else:
                                    # The robot may either be on the previous
                                    # waypoint or the target one
                                    if (self.target_waypoint.graph_index
                                            is not None and
                                            self.dist(
                                                self.position,
                                                target_pose) < 0.5):
                                        self.on_waypoint = \
                                            self.target_waypoint.graph_index
                                    elif (self.last_known_waypoint_index
                                          is not None and
                                          self.dist(
                                              self.position,
                                              self.graph.get_waypoint(
                                                  self.last_known_waypoint_index
                                              ).location) < 0.5):
                                        self.on_waypoint = \
                                            self.last_known_waypoint_index
                                    else:
                                        # update_off_grid()
                                        self.on_lane = None
                                        self.on_waypoint = None

                            # If the RMF core gave us an ETA callback, update it
                            if next_arrival_estimator is not None:
                                # We don't have a precise remaining time from
                                # the robot API, so we just skip this or you
                                # can plug in your own approximation here.
                                pass

                if (not self.remaining_waypoints) \
                        and self.state == RobotState.IDLE:
                    path_finished_callback()
                    self.node.get_logger().info(
                        f"Robot {self.name} has successfully navigated along "
                        f"requested path."
                    )

            self._follow_path_thread = threading.Thread(
                target=_follow_path)
            self._follow_path_thread.start()

    # --------------------------------------------------------------------------
    # Docking
    # --------------------------------------------------------------------------

    def dock(self, dock_name, docking_finished_callback):
        """Docking behavior (application-specific)."""
        self.interrupt()
        with self._lock:
            self._quit_dock_event.clear()
            self.dock_name = dock_name
            assert docking_finished_callback is not None

            # Get the waypoint that the robot is trying to dock into
            dock_waypoint = self.graph.find_waypoint(self.dock_name)
            assert dock_waypoint
            self.dock_waypoint_index = dock_waypoint.index

            def _dock():
                # Request the robot to start the relevant process
                cmd_id = self.next_cmd_id()
                while not self.api.start_process(
                    self.name, cmd_id, self.dock_name, self.map_name
                ):
                    self.node.get_logger().info(
                        f"Requesting robot {self.name} to dock at "
                        f"{self.dock_name}"
                    )
                    if self._quit_dock_event.wait(1.0):
                        break

                with self._lock:
                    self.on_waypoint = None
                    self.on_lane = None

                if self.dock_name not in self.docks:
                    self.node.get_logger().info(
                        f"Requested dock {self.dock_name} not found, "
                        "ignoring docking request"
                    )
                    return

                positions = []
                for wp in self.docks[self.dock_name]:
                    positions.append([wp.x, wp.y, wp.yaw])
                self.node.get_logger().info(
                    f"Robot {self.name} is docking at {self.dock_name}..."
                )

                while not self.api.process_completed(self.name, cmd_id):
                    if len(positions) < 1:
                        break

                    traj = schedule.make_trajectory(
                        self.vehicle_traits,
                        self.adapter.now(),
                        positions
                    )
                    itinerary = schedule.Route(self.map_name, traj)
                    if self.update_handle is not None:
                        participant = \
                            self.update_handle.get_unstable_participant()
                        participant.set_itinerary([itinerary])

                    # Check if we need to abort
                    if self._quit_dock_event.wait(0.1):
                        self.node.get_logger().info("Aborting docking")
                        return

                with self._lock:
                    self.on_waypoint = self.dock_waypoint_index
                    self.dock_waypoint_index = None
                    docking_finished_callback()
                    self.node.get_logger().info(
                        f"Robot {self.name} has completed docking"
                    )

            self._dock_thread = threading.Thread(target=_dock)
            self._dock_thread.start()

    # --------------------------------------------------------------------------
    # Position / battery updates
    # --------------------------------------------------------------------------

    def get_position(self):
        """Return live position of the robot in RMF frame."""
        position = self.api.position(self.name)
        if position is not None:
            x, y = [position[0], position[1]]
            theta = position[2]
            # Wrap theta between [-pi, pi]
            if theta > np.pi:
                theta = theta - (2 * np.pi)
            if theta < -np.pi:
                theta = (2 * np.pi) + theta
            return [x, y, theta]
        else:
            self.node.get_logger().error(
                "Unable to retrieve position from robot.")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error(
                "Unable to retrieve battery data from robot.")
            return self.battery_soc

    def update(self):
        while rclpy.ok():
            # Always update state for web dashboard visibility
            # For offline robots, we'll use last known position
            is_online = self.api.is_robot_online(self.name)
            
            if is_online:
                # Robot is online - get fresh position and battery data
                self.position = self.get_position()
                self.battery_soc = self.get_battery_soc()
            # else: Robot is offline - keep using last known position and battery
            
            # Manage commission state based on online status
            if self.update_handle is not None:
                # Check current commission state
                is_commissioned = self.update_handle.unstable_is_commissioned()
                
                if is_online and not is_commissioned:
                    # Robot came online - recommission it
                    self.node.get_logger().info(
                        f"[{self.name}] Recommissioning robot (now online)"
                    )
                    self.update_handle.unstable_recommission()
                elif not is_online and is_commissioned:
                    # Robot went offline - decommission it
                    self.node.get_logger().warn(
                        f"[{self.name}] Decommissioning robot (now offline)"
                    )
                    self.update_handle.unstable_decommission()
                    
                    # Interrupt any active navigation to allow task reassignment
                    self.node.get_logger().warn(
                        f"[{self.name}] Interrupting active tasks due to offline status"
                    )
                    self.interrupt()  # This stops the follow_new_path thread
                    
            # Always update state so web dashboard shows robot (online or offline)
            if self.update_handle is not None:
                self.update_state()
            
            sleep_duration = float(1.0 / self.update_frequency)
            self.sleep_for(sleep_duration)

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        # Update position
        with self._lock:
            if self.on_waypoint is not None:  # robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif self.on_lane is not None:  # robot is on a lane
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif self.dock_waypoint_index is not None:
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            elif self.action_execution is not None:
                # Performing an action
                if not self.started_action:
                    self.started_action = True
                    self.api.toggle_action(self.name, self.started_action)
                self.update_handle.update_off_grid_position(
                    self.position, self.action_waypoint_index)
            elif (self.target_waypoint is not None and
                  self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:
                # Robot is lost
                self.update_handle.update_lost_position(
                    self.map_name,
                    self.position,
                    max_merge_lane_distance=self.lane_merge_distance
                )

    # --------------------------------------------------------------------------
    # Geometry / lane helpers
    # --------------------------------------------------------------------------

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py, _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        # Spin on the spot
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:
                # The robot is on this lane
                return lane_index
        return None

    def dist(self, A, B):
        """Euclidean distance between A(x,y) and B(x,y)"""
        assert len(A) > 1
        assert len(B) > 1
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)

    def get_speed_limit(self, target_waypoint):
        approach_lane_limit = np.inf
        approach_lanes = target_waypoint.approach_lanes
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            lane_limit = lane.properties.speed_limit
            if lane_limit is not None:
                if lane_limit < approach_lane_limit:
                    approach_lane_limit = lane_limit
        return approach_lane_limit if approach_lane_limit != np.inf else 0.0

    def filter_waypoints(self, wps: list):
        """Return filtered PlanWaypoints close to current position."""
        assert len(wps) > 0
        p = np.array([self.position[0], self.position[1]])

        waypoints = []
        for i in range(len(wps)):
            waypoints.append(PlanWaypoint(i, wps[i]))

        # If the robot is already in the middle of two waypoints, then we can
        # truncate all the waypoints that come before it.
        begin_at_index = 0
        for i in reversed(range(len(waypoints) - 1)):
            i0 = i
            i1 = i + 1
            p0 = waypoints[i0].position
            p0 = np.array([p0[0], p0[1]])
            p1 = waypoints[i1].position
            p1 = np.array([p1[0], p1[1]])
            dp_lane = p1 - p0
            lane_length = np.linalg.norm(dp_lane)
            if lane_length < 1e-3:
                continue
            n_lane = dp_lane / lane_length
            p_l = p - p0
            p_l_proj = np.dot(p_l, n_lane)
            if lane_length < p_l_proj:
                # near lane endpoint
                if np.linalg.norm(p - p1) <= self.lane_merge_distance:
                    begin_at_index = i1
                    break
                continue
            if p_l_proj < 0.0:
                # near lane start
                if np.linalg.norm(p - p0) <= self.lane_merge_distance:
                    begin_at_index = i0
                    break
                continue

            lane_dist = np.linalg.norm(p_l - p_l_proj * n_lane)
            if lane_dist <= self.lane_merge_distance:
                begin_at_index = i1
                break

        if begin_at_index > 0:
            del waypoints[:begin_at_index]

        return waypoints

    # --------------------------------------------------------------------------
    # Actions / docking / mode
    # --------------------------------------------------------------------------

    def complete_robot_action(self):
        with self._lock:
            if self.action_execution is None:
                return
            self.action_execution.finished()
            self.action_execution = None
            self.started_action = False
            self.api.toggle_action(self.name, self.started_action)
            self.node.get_logger().info(
                f"Robot {self.name} has completed the action it was performing"
            )

    def newly_closed_lanes(self, closed_lanes):
        need_to_replan = False
        current_lane = self.get_current_lane()

        if (self.target_waypoint is not None and
                self.target_waypoint.approach_lanes is not None):
            for lane_idx in self.target_waypoint.approach_lanes:
                if lane_idx in closed_lanes:
                    need_to_replan = True
                    if lane_idx == current_lane:
                        lane = self.graph.get_lane(current_lane)
                        return_waypoint = lane.entry.waypoint_index
                        reverse_lane = self.graph.lane_from(
                            lane.entry.waypoint_index,
                            lane.exit.waypoint_index
                        )

                        with self._lock:
                            if reverse_lane:
                                self.on_lane = reverse_lane.index
                            else:
                                self.target_waypoint = return_waypoint

        if (not need_to_replan and self.target_waypoint is not None):
            for wp in self.remaining_waypoints:
                for lane in wp.approach_lanes:
                    if lane in closed_lanes:
                        need_to_replan = True
                        break
                if need_to_replan:
                    break

        if need_to_replan:
            self.update_handle.replan()

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if fleet.fleet_name == self.fleet_name:
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path


    # --------------------------------------------------------------------------
    # Breakdown detection
    # --------------------------------------------------------------------------

    def check_for_breakdown(self):
        """
        Check if robot has experienced a breakdown.
        
        Returns:
            tuple: (has_breakdown: bool, reason: str or None)
        """
        # Check navigation failures
        if self.breakdown_detection['nav_failure_count'] >=            self.breakdown_detection['nav_failure_threshold']:
            return True, "consecutive_navigation_failures"
        
        # Check action failures
        if self.breakdown_detection['action_failure_count'] >=            self.breakdown_detection['action_failure_threshold']:
            return True, "consecutive_action_failures"
        
        # Check connection loss during active task
        if self.state == RobotState.MOVING and not self.api.is_robot_online(self.name):
            return True, "connection_loss_during_task"
        
        return False, None

    def _handle_navigation_failure(self):
        """Track navigation failure and check for breakdown."""
        self.breakdown_detection['nav_failure_count'] += 1
        
        # Suppressed to reduce log noise - failure count still tracked for breakdown detection
        # self.node.get_logger().warn(
        #     f"[{self.name}] Navigation failure count: "
        #     f"{self.breakdown_detection['nav_failure_count']}/"
        #     f"{self.breakdown_detection['nav_failure_threshold']}"
        # )
        
        has_breakdown, reason = self.check_for_breakdown()
        if has_breakdown and self.breakdown_callback:
            self.node.get_logger().error(
                f"🔴 BREAKDOWN DETECTED for [{self.name}]: {reason}"
            )
            self.breakdown_callback(self.name, reason)

    def _handle_navigation_success(self):
        """Reset navigation failure counter on success."""
        if self.breakdown_detection['nav_failure_count'] > 0:
            self.node.get_logger().info(
                f"[{self.name}] Navigation succeeded, resetting failure count"
            )
        self.breakdown_detection['nav_failure_count'] = 0

    def _handle_action_failure(self):
        """Track action failure and check for breakdown."""
        self.breakdown_detection['action_failure_count'] += 1
        
        self.node.get_logger().warn(
            f"[{self.name}] Action failure count: "
            f"{self.breakdown_detection['action_failure_count']}/"
            f"{self.breakdown_detection['action_failure_threshold']}"
        )
        
        has_breakdown, reason = self.check_for_breakdown()
        if has_breakdown and self.breakdown_callback:
            self.node.get_logger().error(
                f"🔴 BREAKDOWN DETECTED for [{self.name}]: {reason}"
            )
            self.breakdown_callback(self.name, reason)

    def _handle_action_success(self):
        """Reset action failure counter on success."""
        if self.breakdown_detection['action_failure_count'] > 0:
            self.node.get_logger().info(
                f"[{self.name}] Action succeeded, resetting failure count"
            )
        self.breakdown_detection['action_failure_count'] = 0


    def mode_request_cb(self, msg):
        if (not msg.fleet_name or msg.fleet_name != self.fleet_name or
                not msg.robot_name or msg.robot_name != self.name):
            return
        if msg.mode.mode == RobotState.IDLE:
            self.complete_robot_action()