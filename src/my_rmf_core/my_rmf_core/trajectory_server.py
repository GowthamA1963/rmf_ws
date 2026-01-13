#!/usr/bin/env python3
import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState

class TrajectoryServer(Node):
    def __init__(self):
        super().__init__('trajectory_server')
        self.fleet_states = {}
        self.create_subscription(
            FleetState,
            'fleet_states',
            self.fleet_state_callback,
            10
        )
        self.get_logger().info('Trajectory Server Started')

    def fleet_state_callback(self, msg):
        self.fleet_states[msg.name] = msg

    async def handler(self, websocket):
        try:
            async for message in websocket:
                data = json.loads(message)
                if data.get('request') == 'trajectory':
                    response = self.generate_trajectory_response(data)
                    await websocket.send(json.dumps(response))
                elif data.get('request') == 'fleet_state':
                    response = self.generate_fleet_state_response(data)
                    await websocket.send(json.dumps(response))
                elif data.get('request') == 'time':
                     # Simple time response
                    response = {
                        'response': 'time',
                        'values': [self.get_clock().now().nanoseconds // 1000000]
                    }
                    await websocket.send(json.dumps(response))
        except websockets.exceptions.ConnectionClosed:
            pass

    def get_robot_status(self, robot):
        """
        Determine if a robot is ONLINE or OFFLINE based on fleet state data.
        
        A robot is considered ONLINE if:
        - It has a valid location (commissioned and Nav2 running)
        
        A robot is considered OFFLINE if:
        - It has no location or invalid position (decommissioned or Nav2 down)
        """
        if robot.location and (robot.location.x != 0 or robot.location.y != 0):
            return "ONLINE"
        return "OFFLINE"

    def generate_fleet_state_response(self, request):
        """
        Generate response with robot status information for all robots.
        This includes robots without active paths (idle robots).
        """
        robots = []
        
        for fleet_name, fleet_state in self.fleet_states.items():
            for robot in fleet_state.robots:
                robot_info = {
                    'name': robot.name,
                    'fleet': fleet_name,
                    'status': self.get_robot_status(robot),
                    'battery': robot.battery_percent,
                    'location': {
                        'x': robot.location.x if robot.location else 0,
                        'y': robot.location.y if robot.location else 0,
                        'yaw': robot.location.yaw if robot.location else 0,
                        'level': robot.location.level_name if robot.location else ""
                    } if robot.location else None,
                    'mode': robot.mode.mode if robot.mode else 0
                }
                robots.append(robot_info)
        
        return {
            'response': 'fleet_state',
            'robots': robots
        }

    def generate_trajectory_response(self, request):
        trajectories = []

        # Convert FleetStates to Trajectories
        # ID generation is tricky without a persistent store, but for visualization
        # distinct IDs for distinct paths is key.
        # We'll use a hash of the robot name + timestamp or just loop through.

        path_id_counter = 0

        for fleet_name, fleet_state in self.fleet_states.items():
            for robot in fleet_state.robots:
                if not robot.path:
                    continue

                # Convert path to knots
                segments = []
                # Add current location as start of path
                if robot.location:
                     segments.append({
                        't': robot.location.t.sec * 1000 + robot.location.t.nanosec // 1000000,
                        'v': [0, 0, 0], # Velocity not always available, assuming 0 for vis
                        'x': [robot.location.x, robot.location.y, robot.location.yaw]
                    })

                for loc in robot.path:
                    segments.append({
                        't': loc.t.sec * 1000 + loc.t.nanosec // 1000000,
                        'v': [0, 0, 0],
                        'x': [loc.x, loc.y, loc.yaw]
                    })

                traj = {
                    'id': path_id_counter,
                    'shape': 'circle', # Placeholder
                    'dimensions': 0.3, # Placeholder
                    'segments': segments,
                    'robot_name': robot.name,
                    'fleet_name': fleet_name,
                    'map_name': robot.location.level_name if robot.location else "",
                    'status': self.get_robot_status(robot)  # Add ONLINE/OFFLINE status
                }
                trajectories.append(traj)
                path_id_counter += 1

        return {
            'response': 'trajectory',
            'values': trajectories,
            'conflicts': [],
            'error': None
        }


async def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)

async def main_async(args=None):
    rclpy.init(args=args)
    server_node = TrajectoryServer()

    stop_future = asyncio.Future()
    loop = asyncio.get_running_loop()
    import signal
    loop.add_signal_handler(signal.SIGINT, stop_future.set_result, None)

    async with websockets.serve(server_node.handler, "0.0.0.0", 8006):
        spin_task = asyncio.create_task(ros_spin(server_node))
        await stop_future
        spin_task.cancel()
        try:
            await spin_task
        except asyncio.CancelledError:
            pass

    server_node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    asyncio.run(main_async(args))

if __name__ == '__main__':
    main()
