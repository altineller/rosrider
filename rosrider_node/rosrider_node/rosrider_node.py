from math import pi
import crc8
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

STATUS_BUFFER_SIZE = 32

# TODO: P1: add velocity and effort to joint states, check where it is done.
# TODO: P1: initial test that board works, it should read status of course?
# TODO: P1: we have a packet sequence, so no packet should be read twice. also no need to invalidate packet if age > period


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


def cap(input_number, min_number, max_number):
    return max(min(max_number, input_number), min_number)


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
            self.odomPub = self.create_publisher(Odometry, '/odom', 10)

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

        self.UPDATE_PERIOD_NS = 1000000000 / self.UPDATE_RATE
        self.UPDATE_PERIOD_MS = 1000 / self.UPDATE_RATE

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

        # Loop PI controller
        self.error_integral = 0.0

    def cmd_vel_callback(self, twist):

        # convert twist command linear, angular to pid command left, right
        pid_commands = self.diffDriveController.get_pid_commands(twist.linear.x, twist.angular.z)

        pid_left_array = bytearray(struct.pack("f", pid_commands.left))
        pid_right_array = bytearray(struct.pack("f", pid_commands.right))

        send_array = pid_left_array + pid_right_array
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                bus.write_i2c_block_data(0x3C, 0x02, send_array)

    def main_timer_callback(self):

        current_time = self.get_clock().now()
        odom = Odometry()
        diagnostics = Diagnostics()
        t = TransformStamped()
        joint_states = JointState()

        # read status from device
        if self.I2C_ENABLED:

            with SMBus(1) as bus:

                try:
                    status = bus.read_i2c_block_data(0x3C, 0xA0, STATUS_BUFFER_SIZE)
                except IOError as e:
                    self.get_logger().info('IOError: %s' % e)
                    return

                # last byte is packet checksum, the one before packet_age_ms
                packet_checksum = status[STATUS_BUFFER_SIZE - 1]
                packet_age_ms = status[STATUS_BUFFER_SIZE - 2]
                diagnostics.packet_age_ms = packet_age_ms

                crc8hash = crc8.crc8()
                crc8hash.update(bytearray(status[0:STATUS_BUFFER_SIZE - 2]))
                calculated_checksum = int(crc8hash.hexdigest(), 16)

                # PI loop lock
                normalized_error = (self.UPDATE_PERIOD_MS / 2) - packet_age_ms  # error = target - packet_age
                error_proportional = normalized_error * 20000  # kP
                self.error_integral += normalized_error * 1000  # kI
                self.error_integral = cap(self.error_integral, -50000, 50000)

                self.main_timer.timer_period_ns = self.UPDATE_PERIOD_NS + (error_proportional + self.error_integral)

                if calculated_checksum == packet_checksum:

                    odom_time = current_time - Duration(nanoseconds=(packet_age_ms * 1000))

                    # anything that sets diagnostics must be in this try
                    try:

                        encoder_left = status[0] << 24 | status[1] << 16 | status[2] << 8 | status[3]
                        encoder_right = status[4] << 24 | status[5] << 16 | status[6] << 8 | status[7]

                        # notice these are not in diagnostics but should be
                        diagnostics.bus_current = (status[8] << 8 | status[9]) / (10.0 * 1000.0)  # convert to amps
                        diagnostics.bus_voltage = (status[10] << 8 | status[11]) / 1000.0  # convert to volts

                        # measured current left, right
                        cs_left_raw = (status[12] << 8 | status[13])
                        cs_right_raw = (status[14] << 8 | status[15])

                        diagnostics.cs_left = cs_left_raw * (6.6 / 4095)  # 500mV/A
                        diagnostics.cs_right = cs_right_raw * (6.6 / 4095)  # 500mV/A

                        # control effort
                        pwm_left = status[16] << 8 | status[17]
                        pwm_right = status[18] << 8 | status[19]

                        # motor pwm direction
                        dir_left = (status[20] & 0x01) > 0
                        dir_right = (status[20] & 0x02) > 0

                        # apply sign to pwm, based on motor pwm direction
                        if dir_left:
                            diagnostics.pwm_left = -pwm_left
                        else:
                            diagnostics.pwm_left = pwm_left

                        if dir_right:
                            diagnostics.pwm_right = -pwm_right
                        else:
                            diagnostics.pwm_right = pwm_right

                        # encoder dir values
                        enc_dir_left = (status[25] & 0x0F) - 1
                        enc_dir_right = ((status[25] & 0xF0) >> 4) - 1

                        # current rpm raw values, multiply by ROUNDS_PER_MINUTE and apply sign by encoder_dir
                        if enc_dir_left == -1:
                            diagnostics.rpm_left = -((status[21] << 8 | status[22]) * self.ROUNDS_PER_MINUTE)
                        else:
                            diagnostics.rpm_left = (status[21] << 8 | status[22]) * self.ROUNDS_PER_MINUTE

                        if enc_dir_right == -1:
                            diagnostics.rpm_right = -((status[23] << 8 | status[24]) * self.ROUNDS_PER_MINUTE)
                        else:
                            diagnostics.rpm_right = (status[23] << 8 | status[24]) * self.ROUNDS_PER_MINUTE

                        diagnostics.power_status = status[26]
                        diagnostics.motor_status = status[27]
                        diagnostics.system_status = status[28]

                    except AssertionError as e:
                        self.get_logger().info('assertion error: %s' % e)

                    # update odometry controller, and pose
                    self.odometryController.update_left_wheel(encoder_left)
                    self.odometryController.update_right_wheel(encoder_right)
                    self.odometryController.update_pose(odom_time.nanoseconds)  # update expects time in nanoseconds

                    # create quaternion
                    q = quaternion_from_euler(0, 0, self.odometryController.get_pose().theta)

                    # fill in odometry values
                    odom.header.stamp = odom_time.to_msg()
                    odom.header.frame_id = self.ODOM_FRAME_ID
                    odom.child_frame_id = self.BASE_FRAME_ID
                    odom.pose.pose.position.x = self.odometryController.get_pose().x
                    odom.pose.pose.position.y = self.odometryController.get_pose().y

                    # fill in pose
                    odom.pose.pose.orientation.x = q[0]
                    odom.pose.pose.orientation.y = q[1]
                    odom.pose.pose.orientation.z = q[2]
                    odom.pose.pose.orientation.w = q[3]

                    # twist
                    odom.twist.twist.linear.x = self.odometryController.get_pose().xVel
                    odom.twist.twist.angular.z = self.odometryController.get_pose().thetaVel

                    # covariances
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

                    # prepare transform
                    t.header.stamp = odom_time.to_msg()
                    t.header.frame_id = self.ODOM_FRAME_ID
                    t.child_frame_id = self.BASE_FRAME_ID
                    t.transform.translation.x = self.odometryController.get_pose().x
                    t.transform.translation.y = self.odometryController.get_pose().y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    self.left_wheel_position = (encoder_left % self.PULSE_PER_REV) / self.PULSE_PER_REV * 2 * pi
                    self.right_wheel_position = (encoder_right % self.PULSE_PER_REV) / self.PULSE_PER_REV * 2 * pi
                    joint_states.header.stamp = odom_time.to_msg()
                    joint_states.name = ['wheel_left_joint', 'wheel_right_joint']
                    joint_states.position = [self.left_wheel_position, self.right_wheel_position]

                else:
                    self.get_logger().info('packet:%s != calculated: %s' % (packet_checksum, calculated_checksum))
                    self.get_logger().info('%s,%s' % (current_time, status))

        if self.PUB_ODOMETRY:
            self.odomPub.publish(odom)

        if self.BROADCAST_TF2:
            self.tf2_broadcaster.sendTransform(t)

        if self.PUB_JOINTS:
            self.jointPub.publish(joint_states)

        if self.PUB_DIAGNOSTICS:
            self.diagPub.publish(diagnostics)


def main(args=None):
    rclpy.init(args=args)
    rosrider_node = ROSRiderNode()
    rclpy.spin(rosrider_node)
    rosrider_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
