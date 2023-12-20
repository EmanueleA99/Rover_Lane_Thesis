import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Build config file path
    config_file = os.path.join(
        get_package_share_directory('rover_lane'),
        'config',
        'lane_sub.yaml'
    )

    node = Node(
            package='rover_lane',
            executable='lane_sub',
            output = 'screen',
            emulate_tty = True,
            name='lane_sub',
            namespace='rover',
            parameters=[config_file])

    ld.add_action(node)

    return ld
