from rosrider_interfaces.srv import DriveCtl
import rclpy
from rclpy.node import Node
from smbus2 import SMBus

'''
SET_DRIVE_MODE 0x05
'''


class DriveModeService(Node):

    def __init__(self):

        super().__init__('service')

        self.srv_drivectl = self.create_service(DriveCtl, '/rosrider/drivectl', self.drivectl_callback)

        self.declare_parameter('I2C_ENABLED', True)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

    def drivectl_callback(self, request, response):
        response.result = 0
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                bus.write_i2c_block_data(0x3C, 0x05, [0, 0, 0, request.drivemode])
        return response


def main():

    rclpy.init()
    service = DriveModeService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
