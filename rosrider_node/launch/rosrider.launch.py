import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ROBOT_MODEL = os.environ['ROBOT_MODEL']

    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('rosrider_node'), 'param', ROBOT_MODEL + '.yaml')

    urdf = os.path.join(get_package_share_directory('rosrider_description'), 'urdf', ROBOT_MODEL + '.urdf')

    rosrider_node = Node(package='rosrider_node', executable='rosrider_node', name='rosrider_node',
                         output='screen', emulate_tty=True, parameters=[config])

    service_node = Node(package='rosrider_services', executable='service', name='service',
                        output='screen', emulate_tty=True, parameters=[config])

    # TODO make configurable or even another
    state_publisher_node =  Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
                                 output='screen', parameters=[config], arguments=[urdf])

    ld.add_action(state_publisher_node)
    ld.add_action(rosrider_node)
    ld.add_action(service_node)

    return ld
