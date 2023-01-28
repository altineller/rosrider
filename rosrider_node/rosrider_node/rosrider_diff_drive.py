import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosrider_node import DiffDriveController

import struct
from smbus2 import SMBus


# TODO: global parameter server
# TODO: cmd_vel mode?
# TODO: there are different scopes of parameters that can be overrittedn
# TODO: copy launch file
# TODO: read from

class ROSRiderDiffDrive(Node):

    def __init__(self):

        super().__init__('diff_drive')

        # notice: queue size=2
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 2)
        self.subscription

        self.declare_parameters(
            namespace='',
            parameters=[
                ('I2C_ENABLED', True),
                ('UPDATE_RATE', 10),
                ('WHEEL_DIA', 0.0685),
                ('BASE_WIDTH', 0.1),
                ('MAX_RPM', 90.0),
                ('ENCODER_PPR', 48),
                ('GEAR_RATIO', 65.0)
            ])

        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').value
        self.UPDATE_RATE = self.get_parameter('UPDATE_RATE').value
        self.WHEEL_DIA = self.get_parameter('WHEEL_DIA').value
        self.BASE_WIDTH = self.get_parameter('BASE_WIDTH').value
        self.MAX_RPM = self.get_parameter('MAX_RPM').value
        self.ENCODER_PPR = self.get_parameter('ENCODER_PPR').value
        self.GEAR_RATIO = self.get_parameter('GEAR_RATIO').value

        self.diffDriveController = DiffDriveController.Controller(self.WHEEL_DIA, self.BASE_WIDTH, self.MAX_RPM)

    def cmd_vel_callback(self, twist):

        pid_commands = self.diffDriveController.get_pid_commands(twist.linear.x, twist.angular.z)

        pid_left_array = list(bytearray(struct.pack("f", pid_commands.left)))
        pid_right_array = list(bytearray(struct.pack("f", pid_commands.right)))

        send_array = pid_left_array + pid_right_array

        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                bus.write_i2c_block_data(0x3C, 0x02, send_array)


def main(args=None):

    rclpy.init(args=args)
    diff_drive = ROSRiderDiffDrive()
    rclpy.spin(diff_drive)
    diff_drive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
