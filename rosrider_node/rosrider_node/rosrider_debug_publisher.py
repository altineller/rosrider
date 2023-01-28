import rclpy
from rclpy.node import Node
from rosrider_interfaces.msg import Diagnostics
import struct
from smbus2 import SMBus

DEBUG_BUFFER_SIZE = 20
NIBBLE_LOOKUP = [0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4]


def count_ones(byte):
    return NIBBLE_LOOKUP[byte & 0x0F] + NIBBLE_LOOKUP[byte >> 4]


class DebugPublisher(Node):

    def __init__(self):

        super().__init__('debug_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('I2C_ENABLED', True),
                ('UPDATE_RATE', 10)
            ])

        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value
        self.UPDATE_RATE = self.get_parameter('UPDATE_RATE').value

        self.diagPub = self.create_publisher(Diagnostics, '/rosrider/diagnostics', 10)

        self.timer = self.create_timer(1.0 / self.UPDATE_RATE, self.debug_callback)

    def debug_callback(self):

        diagnostics = Diagnostics()

        # read debug status from device
        if self.I2C_ENABLED:
            with SMBus(1) as bus:

                # read debug
                debug = bus.read_i2c_block_data(0x3C, 0xA1, 20)

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

                packet_checksum = debug[DEBUG_BUFFER_SIZE - 1]
                calculated_checksum = 0
                for i in range(DEBUG_BUFFER_SIZE - 3):
                    calculated_checksum += count_ones(debug[i])

                if calculated_checksum == packet_checksum:
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

                    # publish when checksum is correct
                    self.diagPub.publish(diagnostics)


def main(args=None):
    rclpy.init(args=args)
    debug_publisher = DebugPublisher()
    rclpy.spin(debug_publisher)
    debug_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
