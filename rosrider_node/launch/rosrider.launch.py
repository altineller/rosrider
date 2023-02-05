import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('rosrider_node'), 'config', 'rosrider.yaml')

    rosrider_node = Node(
        package='rosrider_node',
        executable='rosrider_node',
        name='rosrider_node',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    service_node = Node(
        package='rosrider_services',
        executable='service',
        name='service',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    ld.add_action(rosrider_node)
    ld.add_action(service_node)

    return ld
