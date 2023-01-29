from math import pi
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rosrider_node import OdometryController
from geometry_msgs.msg import Twist
from rosrider_node import DiffDriveController
from rosrider_interfaces.msg import Diagnostics

import math
import struct
from smbus2 import SMBus

TX_BUFFER_SIZE = 18
DEBUG_BUFFER_SIZE = 20
NIBBLE_LOOKUP = [0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4]

# TODO: problem with twist coming from odom, sometimes sharp increases. find reason
# TODO: handle smbus fail
# TODO: this packet entails bus current, and status data, that should be published as well.
# TODO: add velocity and effort to joint states, check where it is done.


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


def count_ones(byte):
    return NIBBLE_LOOKUP[byte & 0x0F] + NIBBLE_LOOKUP[byte >> 4]


class ROSRiderNode(Node):

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
                ('PUB_JOINTS', True),
                ('PUB_DIAGNOSTICS', True)
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
        self.PUB_JOINTS = self.get_parameter('PUB_JOINTS').value
        self.PUB_DIAGNOSTICS = self.get_parameter('PUB_DIAGNOSTICS').value

        # create pub for odometry, if required
        self.odomPub = None
        if self.PUB_ODOMETRY:
            self.odomPub = self.create_publisher(Odometry, '/odometry', 10)

        # create broadcaster for tf2, if required
        self.tf2_broadcaster = None
        if self.BROADCAST_TF2:
            self.tf2_broadcaster = TransformBroadcaster(self)

        # create pub for joint states, if required
        self.jointPub = None
        if self.PUB_JOINTS:
            self.jointPub = self.create_publisher(JointState, '/joint_states', 10)

        # create pub for diagnostics, if required
        self.diagPub = None
        if self.PUB_DIAGNOSTICS:
            self.diagPub = self.create_publisher(Diagnostics, '/rosrider/diagnostics', 10)

        # calculated parameters
        self.PULSE_PER_REV = self.GEAR_RATIO * self.ENCODER_PPR
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_DIA * pi
        self.TICKS_PER_METER = self.PULSE_PER_REV * (1.0 / self.WHEEL_CIRCUMFERENCE)
        self.ROUNDS_PER_MINUTE = (60.0 / (1.0 / self.UPDATE_RATE)) / self.PULSE_PER_REV
        self.LINEAR_RPM = (1.0 / self.WHEEL_CIRCUMFERENCE) * 60.0
        self.ANGULAR_RPM = (self.BASE_WIDTH / (self.WHEEL_CIRCUMFERENCE * 2.0)) * 60.0
        # TODO: use
        self.COMMAND_TIMEOUT_MS = 2000.0 / self.UPDATE_RATE

        # controllers
        self.odometryController = OdometryController.Controller(self.BASE_WIDTH, self.TICKS_PER_METER)
        self.diffDriveController = DiffDriveController.Controller(self.MAX_RPM, self.LINEAR_RPM, self.ANGULAR_RPM)

        # fire up main timer
        self.main_timer = self.create_timer(1.0 / self.UPDATE_RATE, self.main_timer_callback)

        # create sub for cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 2)
        self.subscription

        # for calculating wheel positions
        self.left_wheel_position = 0
        self.right_wheel_position = 0

        # diagnostic data, from read_status
        self.bus_current_avg = 0.0
        self.bus_voltage_avg = 0.0
        self.PWR_STATUS = 0
        self.MTR_STATUS = 0
        self.SYS_STATUS = 0

        # TODO: publish, also optional but with message, battery msg type.

    def cmd_vel_callback(self, twist):
        pid_commands = self.diffDriveController.get_pid_commands(twist.linear.x, twist.angular.z)
        pid_left_array = list(bytearray(struct.pack("f", pid_commands.left)))
        pid_right_array = list(bytearray(struct.pack("f", pid_commands.right)))
        send_array = pid_left_array + pid_right_array
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                bus.write_i2c_block_data(0x3C, 0x02, send_array)

    def main_timer_callback(self):

        current_time = self.get_clock().now()
        odom_time = current_time
        encoder_left = 0
        encoder_right = 0

        # read status from device
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                status = bus.read_i2c_block_data(0x3C, 0xA0, TX_BUFFER_SIZE)
                encoder_left = status[0] << 24 | status[1] << 16 | status[2] << 8 | status[3]
                encoder_right = status[4] << 24 | status[5] << 16 | status[6] << 8 | status[7]
                packet_checksum = status[TX_BUFFER_SIZE - 1]
                calculated_checksum = 0
                for i in range(TX_BUFFER_SIZE - 3):
                    calculated_checksum += count_ones(status[i])
                if calculated_checksum == packet_checksum:
                    delta_msec = status[TX_BUFFER_SIZE - 3] << 8 | status[TX_BUFFER_SIZE - 2]
                    odom_time = current_time - Duration(nanoseconds=(delta_msec * 1000))
                    self.bus_current_avg = status[8] << 8 | status[9]
                    self.bus_voltage_avg = status[10] << 8 | status[11]
                    self.PWR_STATUS = status[12]
                    self.MTR_STATUS = status[13]
                    self.SYS_STATUS = status[14]
                    self.odometryController.update_left_wheel(encoder_left)
                    self.odometryController.update_right_wheel(encoder_right)
                    self.odometryController.update_pose(odom_time.nanoseconds)  # update expects time in nanoseconds
                else:
                    self.get_logger().info('status checksum fail')

        # read debug from device, this must come second
        if self.PUB_DIAGNOSTICS:

            diagnostics = Diagnostics()
            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    # read debug
                    debug = bus.read_i2c_block_data(0x3C, 0xA1, DEBUG_BUFFER_SIZE)
                    packet_checksum = debug[DEBUG_BUFFER_SIZE - 1]
                    calculated_checksum = 0
                    for i in range(DEBUG_BUFFER_SIZE - 3):
                        calculated_checksum += count_ones(debug[i])
                    if calculated_checksum == packet_checksum:
                        pwm_left = debug[0] << 8 | debug[1]
                        pwm_right = debug[2] << 8 | debug[3]
                        dir_left = debug[4] & 0x01 > 0
                        dir_right = debug[4] & 0x02 > 0
                        # measured rpm, left and right
                        rpm_left = struct.unpack('f', bytearray([debug[5], debug[6], debug[7], debug[8]]))[0]
                        rpm_right = struct.unpack('f', bytearray([debug[9], debug[10], debug[11], debug[12]]))[0]
                        # measured current left, right
                        cs_left_raw = debug[13] << 8 | debug[14]
                        cs_right_raw = debug[15] << 8 | debug[16]
                        cs_left = cs_left_raw * (6.6 / 4095)  # 500mV/A
                        cs_right = cs_right_raw * (6.6 / 4095)  # 500mV/A
                        packet_age_ms = (debug[DEBUG_BUFFER_SIZE - 3] << 8) | debug[DEBUG_BUFFER_SIZE - 2]
                        # apply sign to pwm and rpm, based on dir_left, dir_right
                        if dir_left:
                            diagnostics.pwm_left = -pwm_left
                            diagnostics.rpm_left = -rpm_left
                        else:
                            diagnostics.pwm_left = pwm_left
                            diagnostics.rpm_left = rpm_left
                        if dir_right:
                            diagnostics.pwm_right = -pwm_right
                            diagnostics.rpm_right = -rpm_right
                        else:
                            diagnostics.pwm_right = pwm_right
                            diagnostics.rpm_right = rpm_right

                        diagnostics.cs_left = cs_left
                        diagnostics.cs_right = cs_right
                        diagnostics.packet_age_ms = packet_age_ms
                        self.diagPub.publish(diagnostics)
                    else:
                        self.get_logger().info('diagnostics checksum fail.')

        if self.PUB_ODOMETRY:

            odom = Odometry()
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
    rosrider_node = ROSRiderNode()
    rclpy.spin(rosrider_node)
    rosrider_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
