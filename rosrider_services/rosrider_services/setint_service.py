from rosrider_interfaces.srv import SetInt
import rclpy
from rclpy.node import Node
import struct
from smbus2 import SMBus

'''
EEPROM_WRITE_INT 0x0A
'''


class SetIntService(Node):

    def __init__(self):

        super().__init__('service')

        self.srv_setint = self.create_service(SetInt, '/rosrider/setint', self.setint_callback)

        self.declare_parameter('I2C_ENABLED', True)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

    # TODO: read result
    def setint_callback(self, request, response):

        u32 = request.u32
        addr = request.addr

        response.result = 0

        # check address is 0x00 to 0x40
        if addr >= 0x40:
            response.result += 1

        if response.result == 0:
            set_int_array = list(bytearray(struct.pack("i", u32)))
            set_int_array.insert(0, addr)

            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    bus.write_i2c_block_data(0x3C, 0x0A, set_int_array)

        return response


def main():

    rclpy.init()
    service = SetIntService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
