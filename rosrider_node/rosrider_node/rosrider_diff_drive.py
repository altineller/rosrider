import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosrider_node import DiffDriveController

import struct
from smbus2 import SMBus


class ROSRiderDiffDrive(Node):

    def __init__(self):

        super().__init__('diff_drive')

        # notcie: queue size=2
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 2)
        self.subscription

        self.declare_parameters(
            namespace='',
            parameters=[
                ('I2C_ENABLED', False),
                ('UPDATE_RATE', 10),
                ('WHEEL_DIA', 0.0685),
                ('BASE_WIDTH', 0.1),
                ('MAX_RPM', 90.0),
                ('ENCODER_PPR', 48),
                ('GEAR_RATIO', 65.0)
            ])

        # TODO: pass the new parameters frame id, and such
        # TODO: global parameter server

        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').value
        self.UPDATE_RATE = self.get_parameter('UPDATE_RATE').value
        self.WHEEL_DIA = self.get_parameter('WHEEL_DIA').value
        self.BASE_WIDTH = self.get_parameter('BASE_WIDTH').value
        self.MAX_RPM = self.get_parameter('MAX_RPM').value
        self.ENCODER_PPR = self.get_parameter('ENCODER_PPR').value
        self.GEAR_RATIO = self.get_parameter('GEAR_RATIO').value

        self.diffDriveCntroller = DiffDriveController.Controller(self.WHEEL_DIA, self.BASE_WIDTH, self.MAX_RPM)

    def cmd_vel_callback(self, twist):

        pid_commands = self.diffDriveController.get_pid_commands(twist.linear.x, twist.angular.z)

        pid_left_array = list(bytearray(struct.pack("f", pid_commands.left)))
        pid_left_array.insert(0, 0)

        pid_right_array = list(bytearray(struct.pack("f", pid_commands.right)))
        pid_right_array.insert(0, 1)

        # TODO: what might be best is, two motors gets i2c commands at the same time
        # TODO: ow
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                bus.write_i2c_block_data(0x3C, 0x02, pid_left_array)
                bus.write_i2c_block_data(0x3C, 0x02, pid_right_array)

        self.get_logger().info("command: %f %f" % (pid_commands.left, pid_commands.right))


def main(args=None):

    rclpy.init(args=args)

    diff_drive = ROSRiderDiffDrive()

    rclpy.spin(diff_drive)

    diff_drive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
