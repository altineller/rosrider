import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ROBOT_MODEL = os.environ['ROBOT_MODEL']

    param_dir = LaunchConfiguration('param_dir', default=os.path.join(get_package_share_directory('rosrider_fakenode'), 'param', ROBOT_MODEL + '.yaml'))

    rviz_dir = LaunchConfiguration('rviz_dir', default=os.path.join(get_package_share_directory('rosrider_fakenode'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(get_package_share_directory('rosrider_description'), 'urdf', ROBOT_MODEL + '.urdf')

    return LaunchDescription([

        LogInfo(msg=['Execute ROSRider FakeNode!']),

        DeclareLaunchArgument('param_dir', default_value=param_dir, description='Specifying parameter direction'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource([rviz_dir, '/rviz2.launch.py'])),

        Node(package='rosrider_fakenode', executable='rosrider_fakenode', parameters=[param_dir], output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             output='screen', parameters=[{'use_sim_time': use_sim_time}], arguments=[urdf]),
    ])
