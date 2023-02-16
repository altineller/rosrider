import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('rosrider_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(get_package_share_directory('rosrider_gazebo'), 'worlds', 'empty_world.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ukf_launch_dir = get_package_share_directory('rosrider_node')
    ukf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ukf_launch_dir, 'ukf.launch.py')
        ),
        launch_arguments={
        }.items()
    )

    rviz_launch_dir = os.path.join(get_package_share_directory('rosrider_fakenode'), 'launch')
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz2.launch.py')
        ),
        launch_arguments={
        }.items()
    )

    joy_launch_dir = os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch')
    joy_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_launch_dir, 'teleop-launch.py')
        ),
        launch_arguments={
            'joy_config': 'f710'
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(joy_cmd)

    return ld
