from rosrider_interfaces.srv import SetFloat
import rclpy
from rclpy.node import Node
import struct
from smbus2 import SMBus

'''
EEPROM_WRITE_FLOAT 0x0B
'''


class SetFloatService(Node):

    def __init__(self):

        super().__init__('service')

        self.srv_setfloat = self.create_service(SetFloat, 'rosrider/setfloat', self.setfloat_callback)

        self.declare_parameter('I2C_ENABLED', False)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

    def setfloat_callback(self, request, response):

        f32 = request.f32
        addr = request.addr

        response.result = 0

        # check address is 0x40 to 0x80
        if addr < 0x40 or addr >= 0x80:
            response.result += 1

        if response.result == 0:
            set_float_array = list(bytearray(struct.pack("f", f32)))
            set_float_array.insert(0, addr)
            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    bus.write_i2c_block_data(0x3C, 0x0B, set_float_array)

        return response


def main():

    rclpy.init()
    service = SetFloatService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
