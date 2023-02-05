from rosrider_interfaces.srv import SysCtl
from rosrider_interfaces.srv import DriveCtl
from rosrider_interfaces.srv import SetRtc
from rosrider_interfaces.srv import LedCtl
from rosrider_interfaces.srv import PidCtl
from rosrider_interfaces.srv import SetInt
from rosrider_interfaces.srv import SetFloat

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
import struct
import numpy as np
from time import time
import crc8

'''
SYSCTL_COMMAND 0x04
SET_DRIVE_MODE 0x05
SET_RTC 0x06
LED_COMMAND 0x07
PID_TUNE_LEFT 0x08
PID_TUNE_RIGHT 0x09
EEPROM_WRITE_INT 0x0A
EEPROM_WRITE_FLOAT 0x0B
'''


class ROSRiderService(Node):

    def __init__(self):

        super().__init__('service')

        self.declare_parameter('I2C_ENABLED', True)
        self.I2C_ENABLED = self.get_parameter('I2C_ENABLED').get_parameter_value().bool_value

        self.declare_parameter('MAINTENANCE', True)
        self.MAINTENANCE = self.get_parameter('MAINTENANCE').get_parameter_value().bool_value

        self.srv_sysctl = self.create_service(SysCtl, '/rosrider/sysctl', self.sys_callback)
        self.srv_drivectl = self.create_service(DriveCtl, '/rosrider/drivectl', self.drivectl_callback)
        self.srv_ledctl = self.create_service(LedCtl, '/rosrider/ledctl', self.led_callback)

        if self.MAINTENANCE:
            self.srv_rtcctl = self.create_service(SetRtc, '/rosrider/rtcctl', self.setrtc_callback)
            self.srv_pidctl = self.create_service(PidCtl, '/rosrider/pidctl', self.pidtune_callback)
            self.srv_setint = self.create_service(SetInt, '/rosrider/setint', self.setint_callback)
            self.srv_setfloat = self.create_service(SetFloat, '/rosrider/setfloat', self.setfloat_callback)

    def sys_callback(self, request, response):
        response.result = 0
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                try:
                    bus.write_i2c_block_data(0x3C, 0x04, [0, 0, 0, request.cmd])
                except IOError as e:
                    response.result += 64  # ioerror
                    self.get_logger().info('IOError @ sys_callback: %s' % e)
        else:
            response.result += 128  # i2c disabled
        return response

    def drivectl_callback(self, request, response):
        response.result = 0
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                try:
                    bus.write_i2c_block_data(0x3C, 0x05, [0, 0, 0, request.drivemode])
                except IOError as e:
                    response.result += 64  # ioerror
                    self.get_logger().info('IOError @ drivectl_callback: %s' % e)
        else:
            response.result += 128  # i2c disabled
        return response

    def pidtune_callback(self, request, response):

        kp = request.kP
        ki = request.kI
        kd = request.kD
        i = request.i

        response.result = 0

        if (kp < 0.0) or (ki < 0.0) or (kd < 0.0):
            response.result += 16  # k is negative
        else:

            p_array = bytearray(struct.pack("f", kp))
            i_array = bytearray(struct.pack("f", ki))
            d_array = bytearray(struct.pack("f", kd))
            send_array = p_array + i_array + d_array

            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    try:
                        if i == 0:
                            bus.write_i2c_block_data(0x3C, 0x08, send_array)
                        elif i == 1:
                            bus.write_i2c_block_data(0x3C, 0x09, send_array)
                        else:
                            response.result += 1  # wrong index
                    except IOError as e:
                        response.result += 64  # ioerror
                        self.get_logger().info('IOError @ pid_callback: %s' % e)
            else:
                response.result += 128  # i2c disabled
        return response

    def setrtc_callback(self, request, response):
        response.result = 0
        if self.I2C_ENABLED:
            with SMBus(1) as bus:
                try:
                    # gets time to seconds
                    current_time = int(time())
                    send_array = struct.pack("<i", current_time)
                    bus.write_i2c_block_data(0x3C, 0x06, send_array)
                except IOError as e:
                    response.result += 64  # ioerror
                    self.get_logger().info('IOError @ setrtc_callback: %s' % e)
        else:
            response.result += 128  # i2c disabled
        return response

    def led_callback(self, request, response):

        led_array = request.colormask
        frequency = request.frequency

        if frequency not in [0, 1, 2]:
            response.result += 16  # frequency is not 0 to 2
        else:
            send_array = np.append(led_array, frequency)
            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    try:
                        bus.write_i2c_block_data(0x3C, 0x07, send_array)
                    except IOError as e:
                        response.result += 64  # ioerror
                        self.get_logger().info('IOError @ led_callback: %s' % e)
            else:
                response.result += 128  # i2c disabled
        return response

    def setint_callback(self, request, response):

        u32 = request.u32
        addr = request.addr

        response.result = 0
        if addr >= 0x40:
            response.result += 16  # address is not between 0x00 to 0x40
        else:

            set_int_array = bytearray(struct.pack("i", u32))

            hash8 = crc8.crc8()
            hash8.update(set_int_array)
            write_checksum = int(hash8.hexdigest(), 16)

            # first byte is address
            set_int_array.insert(0, addr)

            # last byte is checksum
            set_int_array.insert(len(set_int_array), write_checksum)

            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    try:
                        bus.write_i2c_block_data(0x3C, 0x0A, set_int_array)
                        result = bus.read_i2c_block_data(0x3C, 0xB0, 4)
                        if result[0] == 0 and result[1] == 0x0A:
                            # returns: 0 success, 1 write error, 2 checksum fail
                            response.result = result[3]
                        else:
                            response.result += 32  # assert fail
                    except IOError as e:
                        response.result += 64  # ioexception
                        self.get_logger().info('IOError @ setint_callback: %s' % e)
            else:
                response.result += 128  # i2c disabled

        return response

    def setfloat_callback(self, request, response):

        f32 = request.f32
        addr = request.addr

        response.result = 0
        if addr < 0x40 or addr >= 0x80:
            response.result += 16  # address is not between 0x40 to 0x80
        else:

            set_float_array = bytearray(struct.pack("f", f32))

            # calculate checksum
            hash8 = crc8.crc8()
            hash8.update(set_float_array)
            write_checksum = int(hash8.hexdigest(), 16)

            # first byte is address
            set_float_array.insert(0, addr)

            # last byte is checksum
            set_float_array.insert(len(set_float_array), write_checksum)

            if self.I2C_ENABLED:
                with SMBus(1) as bus:
                    try:
                        bus.write_i2c_block_data(0x3C, 0x0B, set_float_array)
                        result = bus.read_i2c_block_data(0x3C, 0xB0, 4)
                        if result[0] == 0 and result[1] == 0x0B:
                            # returns: 0 success, 1 write error, 2 checksum fail
                            response.result += result[3]
                        else:
                            response.result += 32  # assert fail
                    except IOError as e:
                        response.result += 64  # ioexception
                        self.get_logger().info('IOError @ setfloat_callback: %s' % e)
            else:
                response.result += 128  # i2c disabled
        return response


def main():

    rclpy.init()
    service = ROSRiderService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
