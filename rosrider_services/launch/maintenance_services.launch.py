from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosrider_services',
            executable='pidtune',
            name='pidtune_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'I2C_ENABLED': False}
            ]
        ),
        Node(
            package='rosrider_services',
            executable='setfloat',
            name='setfloat_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'I2C_ENABLED': False}
            ]
        ),
        Node(
            package='rosrider_services',
            executable='setint',
            name='setint_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'I2C_ENABLED': False}
            ]
        ),
        Node(
            package='rosrider_services',
            executable='setrtc',
            name='setrtc_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'I2C_ENABLED': False}
            ]
        )
    ])
