from math import pi


class PIDCommand:
    def __init__(self):
        self.left = 0
        self.right = 0


class Controller:

    def __init__(self, max_rpm, linear_rpm, angular_rpm):
        self.MAX_RPM = max_rpm
        self.LINEAR_RPM = linear_rpm
        self.ANGULAR_RPM = angular_rpm

    def get_pid_commands(self, command_linear_x, command_angular_z):

        commands = PIDCommand()

        commands.left = (command_linear_x * self.LINEAR_RPM) - (command_angular_z * self.ANGULAR_RPM)
        commands.right = (command_linear_x * self.LINEAR_RPM) + (command_angular_z * self.ANGULAR_RPM)

        # adjust motor speeds if they exceed the maximum motor speed
        if max(commands.left, commands.right) > self.MAX_RPM:
            factor = self.MAX_RPM / max(commands.left, commands.right)
            commands.left *= factor
            commands.right *= factor

        return commands
