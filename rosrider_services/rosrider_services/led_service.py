from rosrider_interfaces.srv import LedCtl
import rclpy
from rclpy.node import Node
import numpy as np
from smbus2 import SMBus

'''
LED_COMMAND 0x07
'''


class LedService(Node):

    def __init__(self):

        super().__init__('led_service')

        self.srv_ledctl = self.create_service(LedCtl, 'rosrider/ledctl', self.led_callback)

        self.declare_parameter('I2C_ENABLED', True)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

    def led_callback(self, request, response):

        led_array = request.colormask
        frequency = request.frequency

        # check if frequency is 1 to 5
        if frequency not in range(6):
            response.result += 1

        if response.result == 0:
            send_array = np.append(led_array, frequency)
            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    bus.write_i2c_block_data(0x3C, 0x07, send_array)

        return response


def main():

    rclpy.init()
    service = LedService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
