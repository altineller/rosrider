from rosrider_interfaces.srv import PidCtl
import rclpy
from rclpy.node import Node
import struct
from smbus2 import SMBus

'''
PID_TUNE_LEFT 0x08
PID_TUNE_RIGHT 0x09
'''


class PidTuneService(Node):

    def __init__(self):

        super().__init__('service')

        self.srv_pidctl = self.create_service(PidCtl, 'rosrider/pidctl', self.pid_callback)

        self.declare_parameter('I2C_ENABLED', False)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

    def pid_callback(self, request, response):

        kp = request.kP
        ki = request.kI
        kd = request.kD
        i = request.i

        response.result = 0

        # check k is not negative
        if (kp < 0.0) or (ki < 0.0) or (kd < 0.0):
            response.result += 2

        if response.result == 0:

            p_array = bytearray(struct.pack("f", kp))
            i_array = bytearray(struct.pack("f", ki))
            d_array = bytearray(struct.pack("f", kd))

            send_array = p_array + i_array + d_array

            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    if i == 0:
                        bus.write_i2c_block_data(0x3C, 0x08, send_array)
                    elif i == 1:
                        bus.write_i2c_block_data(0x3C, 0x09, send_array)
                    else:
                        response.result += 4

        return response


def main():

    rclpy.init()
    service = PidTuneService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
