class PID:
    def __init__(self, Kp, Ki, Kd, min_out=-float("inf"), max_out=float("inf")):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.min_out = min_out
        self.max_out = max_out

        self.last_e = None
        self.sum_e = 0

    def step(self, e, dt):
        """ dt should be the time elapsed from the last time step was called """

        if self.last_e is not None:
            derivative = (e - self.last_e) / dt
        else:
            derivative = 0

        self.last_e = e
        self.sum_e += e * dt

        output = self.Kp * e + self.Kd * derivative + self.Ki * self.sum_e
        output = min(max(self.min_out, output), self.max_out)

        return output
