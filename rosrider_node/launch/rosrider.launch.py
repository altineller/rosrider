import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('rosrider_node'), 'config', 'rosrider.yaml')

    diff_drive_node = Node(
        package='rosrider_node',
        executable='diff_drive',
        name='diff_drive',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    odometry_publisher_node = Node(
        package='rosrider_node',
        executable='odometry_publisher',
        name='odometry_publisher',
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

    ld.add_action(diff_drive_node)
    ld.add_action(odometry_publisher_node)
    ld.add_action(drivemode_service_node)
    ld.add_action(led_service_node)
    ld.add_action(sysctl_node)

    return ld
