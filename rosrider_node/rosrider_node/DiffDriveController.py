from math import pi

# TODO: add velocity and effort to joint states
class PIDCommand:
    def __init__(self):
        self.left = 0
        self.right = 0


class Controller:

    def __init__(self, wheel_dia, base_width, max_rpm):
        self.wheel_dia = wheel_dia
        self.base_width = base_width
        self.max_rpm = max_rpm
        # init calculated parameters
        self.wheel_circumference = wheel_dia * pi
        self.linear_rpm = (1.0 / self.wheel_circumference) * 60.0
        self.angular_rpm = (base_width / (self.wheel_circumference * 2.0)) * 60.0

    def get_pid_commands(self, command_linear_x, command_angular_z):

        commands = PIDCommand()

        commands.left = (command_linear_x * self.linear_rpm) - (command_angular_z * self.angular_rpm)
        commands.right = (command_linear_x * self.linear_rpm) + (command_angular_z * self.angular_rpm)

        # adjust motor speeds if they exceed the maximum motor speed
        if max(commands.left, commands.right) > self.max_rpm:
            factor = self.max_rpm / max(commands.left, commands.right)
            commands.left *= factor
            commands.right *= factor

        return commands
