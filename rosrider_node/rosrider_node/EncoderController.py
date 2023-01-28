class Controller:

    def __init__(self):

        self.position = 0
        self.prev_position = 0
        self.delta_position = 0

        self.limit_min = -2147483648
        self.limit_max = 2147483647

        self.encoder_max = 4294967296

    def get_delta(self):
        delta_position = self.delta_position
        self.delta_position = 0
        return delta_position

    def update(self, position):

        increment = position - self.prev_position

        # adjust increment for overflow
        if increment < self.limit_min:
            increment = (self.encoder_max - self.prev_position) + position
        elif increment > self.limit_max:
            increment = ((self.encoder_max - position) + self.prev_position) * -1

        # increment delta position
        self.delta_position += increment

        # record prev position
        self.prev_position = position


