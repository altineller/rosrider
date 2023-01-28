from rosrider_interfaces.srv import SysCtl
import rclpy
from rclpy.node import Node
from smbus2 import SMBus

'''
SYSCTL_COMMAND 0x04
'''


class SysctlService(Node):

    def __init__(self):

        super().__init__('sysctl_service')

        self.srv_sysctl = self.create_service(SysCtl, '/rosrider/sysctl', self.sys_callback)

        self.declare_parameter('I2C_ENABLED', True)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

    def sys_callback(self, request, response):
        response.result = 0
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                bus.write_i2c_block_data(0x3C, 0x04, [0, 0, 0, request.cmd])

        return response


def main():

    rclpy.init()
    service = SysctlService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
