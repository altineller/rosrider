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

    drivemode_service_node = Node(
        package='rosrider_services',
        executable='drivemode',
        name='drivemode',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    led_service_node = Node(
        package='rosrider_services',
        executable='led',
        name='led',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    sysctl_node = Node(
        package='rosrider_services',
        executable='sysctl',
        name='sysctl',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    ld.add_action(rosrider_node)
    ld.add_action(drivemode_service_node)
    ld.add_action(led_service_node)
    ld.add_action(sysctl_node)

    return ld
