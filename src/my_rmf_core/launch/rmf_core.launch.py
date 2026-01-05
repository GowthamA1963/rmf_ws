from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nav_graph_path = "/home/robot1/rmf_ws/src/my_rmf_core/maps/office_test.building.yaml"

    return LaunchDescription([

        # 1) RMF Schedule Node
        Node(
            package="rmf_traffic_ros2",
            executable="rmf_traffic_schedule",
            output="screen"
        ),

        # 3) Building Map Server
        Node(
            package="rmf_building_map_tools",
            executable="building_map_server",
            name="rmf_building_map_server",
            arguments=[nav_graph_path],
            output="screen"
        ),

        Node(
            package="rmf_task_ros2",
            executable="rmf_task_dispatcher",
            name="rmf_task_dispatcher",
            output="screen"
        ),

        # 4) Trajectory Server
        Node(
            package="my_rmf_core",
            executable="trajectory_server",
            name="trajectory_server",
            output="screen"
        )


    ])
