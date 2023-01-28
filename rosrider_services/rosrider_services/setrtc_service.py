from rosrider_interfaces.srv import SetRtc
import rclpy
from rclpy.node import Node
import struct
from smbus2 import SMBus

from time import time
import datetime


'''
#define SET_RTC 0x06
'''


class SetRtcService(Node):

    def __init__(self):

        super().__init__('service')

        self.srv_sysctl = self.create_service(SetRtc, 'rosrider/sysctl', self.setrtc_callback)

        self.declare_parameter('I2C_ENABLED', True)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

    def setrtc_callback(self, request, response):

        response.result = 0

        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                # gets time to seconds
                current_time = int(time())
                send_array = struct.pack("<i", current_time)
                bus.write_i2c_block_data(0x3C, 0x06, send_array)
        else:
            response.result += 128

        # TODO: return some result if i2c disabled in others too
        return response


def main():

    rclpy.init()
    service = SetRtcService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
