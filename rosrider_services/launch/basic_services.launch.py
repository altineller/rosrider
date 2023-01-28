from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosrider_services',
            executable='drivemode',
            name='drivemode_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'I2C_ENABLED': False}
            ]
        ),
        Node(
            package='rosrider_services',
            executable='led',
            name='led_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'I2C_ENABLED': False}
            ]
        ),
        Node(
            package='rosrider_services',
            executable='sysctl',
            name='sysctl_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'I2C_ENABLED': False}
            ]
        )
    ])
