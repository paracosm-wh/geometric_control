import os.path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('geometric_control'),
        'config',
        'geometric_control.yaml'
    )

    geometric_controller_node = Node(
        package='geometric_control',
        executable='geometric_controller_node',
        output='screen',
        parameters=[params_file],
        shell=True,
    )

    return LaunchDescription([
        # micro_XRCE_DDS_agent,
        geometric_controller_node]
    )
