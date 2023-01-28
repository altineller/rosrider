from math import pi, cos, sin

from rosrider_node import EncoderController
from rosrider_node.Pose import Pose

CONVERSION_CONSTANT = 10 ** 9


class Controller:

    def __init__(self, base_width, ticks_per_meter):

        self.left_encoder = EncoderController.Controller()
        self.right_encoder = EncoderController.Controller()

        self.BASE_WIDTH = base_width
        self.TICKS_PER_METER = ticks_per_meter

        self.pose = Pose()

        self.prev_nanoseconds = 0

    def update_left_wheel(self, position):
        self.left_encoder.update(position)

    def update_right_wheel(self, position):
        self.right_encoder.update(position)

    def update_pose(self, nanoseconds):

        left_travel = self.left_encoder.get_delta() / self.TICKS_PER_METER
        right_travel = self.right_encoder.get_delta() / self.TICKS_PER_METER

        delta_nanoseconds = nanoseconds - self.prev_nanoseconds

        delta_travel = (right_travel + left_travel) / 2
        delta_theta = (right_travel - left_travel) / self.BASE_WIDTH

        if right_travel == left_travel:
            delta_x = left_travel * cos(self.pose.theta)
            delta_y = left_travel * sin(self.pose.theta)
        else:

            radius = delta_travel / delta_theta

            # Find the instantaneous center of curvature (ICC).
            icc_x = self.pose.x - radius * sin(self.pose.theta)
            icc_y = self.pose.y + radius * cos(self.pose.theta)

            delta_x = cos(delta_theta) * (self.pose.x - icc_x) - sin(delta_theta) * (self.pose.y - icc_y) + icc_x - self.pose.x
            delta_y = sin(delta_theta) * (self.pose.x - icc_x) + cos(delta_theta) * (self.pose.y - icc_y) + icc_y - self.pose.y

        self.pose.x += delta_x
        self.pose.y += delta_y

        self.pose.theta = (self.pose.theta + delta_theta) % (2 * pi)
        self.pose.yVel = 0

        if delta_nanoseconds > 0:
            self.pose.xVel = delta_travel / (delta_nanoseconds / CONVERSION_CONSTANT)
            self.pose.thetaVel = delta_theta / (delta_nanoseconds / CONVERSION_CONSTANT)
        else:
            self.pose.xVel = 0
            self.pose.thetaVel = 0

        self.prev_nanoseconds = nanoseconds

    def get_pose(self):
        return self.pose

