from math import pi
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rosrider_node import OdometryController

import math
from smbus2 import SMBus

# TODO: parameter callback
# TODO: global parameter
# TODO: check twist coming from odom
# TODO: optional TF bcast of wheel positions


# TODO: this packet entails bus current, and status data, that should be published as well.
'''
bus_current_raw = status[12] << 8 | status[13]
bus_voltage_raw = status[14] << 8 | status[15]
'''


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk
    q = [0] * 4
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss
    return q


class OdometryPublisher(Node):

    def __init__(self):

        super().__init__('odometry_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('I2C_ENABLED', True),
                ('UPDATE_RATE', 10),
                ('WHEEL_DIA', 0.0685),
                ('BASE_WIDTH', 0.1),
                ('MAX_RPM', 90.0),
                ('ENCODER_PPR', 48),
                ('GEAR_RATIO', 65.0),
                ('ODOM_FRAME_ID', 'odom'),
                ('BASE_FRAME_ID', 'base_footprint'),
                ('BROADCAST_TF2', True),
                ('PUB_ODOMETRY', True),
                ('PUB_JOINTS', True)
            ])

        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').value
        self.UPDATE_RATE = self.get_parameter('UPDATE_RATE').value
        self.WHEEL_DIA = self.get_parameter('WHEEL_DIA').value
        self.BASE_WIDTH = self.get_parameter('BASE_WIDTH').value
        self.MAX_RPM = self.get_parameter('MAX_RPM').value
        self.ENCODER_PPR = self.get_parameter('ENCODER_PPR').value
        self.GEAR_RATIO = self.get_parameter('GEAR_RATIO').value
        self.ODOM_FRAME_ID = self.get_parameter('ODOM_FRAME_ID').value
        self.BASE_FRAME_ID = self.get_parameter('BASE_FRAME_ID').value
        self.BROADCAST_TF2 = self.get_parameter('BROADCAST_TF2').value
        self.PUB_ODOMETRY = self.get_parameter('PUB_ODOMETRY').value
        self.PUB_JOINTS = self.get_parameter('PUB_JOINTS').value  # TODO: joint data should be sent over tf2

        self.odomPub = None
        if self.PUB_ODOMETRY:
            self.odomPub = self.create_publisher(Odometry, '/odometry', 10)

        self.tf2_broadcaster = None
        if self.BROADCAST_TF2:
            self.tf2_broadcaster = TransformBroadcaster(self)

        self.jointPub = None
        if self.PUB_JOINTS:
            self.jointPub = self.create_publisher(JointState, '/joint_states', 10)

        # calculated parameters
        self.PULSE_PER_REV = self.GEAR_RATIO * self.ENCODER_PPR
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_DIA * pi
        self.TICKS_PER_METER = self.PULSE_PER_REV * (1.0 / self.WHEEL_CIRCUMFERENCE)
        self.ROUNDS_PER_MINUTE = (60.0 / (1.0 / self.UPDATE_RATE)) / self.PULSE_PER_REV

        self.odometryController = OdometryController.Controller(self.BASE_WIDTH, self.TICKS_PER_METER)

        self.timer = self.create_timer(1.0 / self.UPDATE_RATE, self.odometry_callback)

        self.left_wheel_position = 0
        self.right_wheel_position = 0

    def odometry_callback(self):

        current_time = self.get_clock().now()
        odom_time = current_time
        encoder_left = 0
        encoder_right = 0
        odom = Odometry()

        # read status from device
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                status = bus.read_i2c_block_data(0x3C, 0xA0, 22)
                encoder_left = status[0] << 24 | status[1] << 16 | status[2] << 8 | status[3]
                encoder_right = status[4] << 24 | status[5] << 16 | status[6] << 8 | status[7]

                # TODO: add checksum checking

                delta_msec = status[20] << 8 | status[21]
                odom_time = current_time - Duration(nanoseconds=(delta_msec * 1000))
                self.odometryController.update_left_wheel(encoder_left)
                self.odometryController.update_right_wheel(encoder_right)
                self.odometryController.update_pose(odom_time.nanoseconds)  # update_pose expects time as nanoseconds

        if self.BROADCAST_TF2:

            t = TransformStamped()
            t.header.stamp = odom_time.to_msg()
            t.header.frame_id = self.ODOM_FRAME_ID
            t.child_frame_id = self.BASE_FRAME_ID

            t.transform.translation.x = self.odometryController.get_pose().x
            t.transform.translation.y = self.odometryController.get_pose().y
            t.transform.translation.z = 0.0

            q = quaternion_from_euler(0, 0, self.odometryController.get_pose().theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf2_broadcaster.sendTransform(t)

        if self.PUB_ODOMETRY:

            odom.header.stamp = odom_time.to_msg()
            odom.header.frame_id = self.ODOM_FRAME_ID
            odom.child_frame_id = self.BASE_FRAME_ID

            odom.pose.pose.position.x = self.odometryController.get_pose().x
            odom.pose.pose.position.y = self.odometryController.get_pose().y

            q = quaternion_from_euler(0, 0, self.odometryController.get_pose().theta)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            odom.twist.twist.linear.x = self.odometryController.get_pose().xVel
            odom.twist.twist.angular.z = self.odometryController.get_pose().thetaVel

            odom.pose.covariance[0] = 0.01
            odom.pose.covariance[7] = 0.0
            odom.pose.covariance[14] = 0.0
            odom.pose.covariance[21] = 0.0
            odom.pose.covariance[28] = 0.0
            odom.pose.covariance[35] = 0.1

            odom.twist.covariance[0] = 0.01
            odom.twist.covariance[7] = 0.0
            odom.twist.covariance[14] = 0.0
            odom.twist.covariance[21] = 0.0
            odom.twist.covariance[28] = 0.0
            odom.twist.covariance[35] = 0.01

            self.odomPub.publish(odom)

        if self.PUB_JOINTS:
            self.left_wheel_position = (encoder_left % self.PULSE_PER_REV) / self.PULSE_PER_REV * 2 * pi
            self.right_wheel_position = (encoder_right % self.PULSE_PER_REV) / self.PULSE_PER_REV * 2 * pi

            joint_states = JointState()
            joint_states.header.stamp = odom_time.to_msg()
            joint_states.name = ['wheel_left_joint', 'wheel_right_joint']
            joint_states.position = [self.left_wheel_position, self.right_wheel_position]
            self.jointPub.publish(joint_states)


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
