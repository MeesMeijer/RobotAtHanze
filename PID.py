



class PID:
    _i = 0
    _lastP = 0

    def __init__(self, P=None, I=None, D=None):
        self._Kp = P  # 5
        self._Ki = I  # 0
        self._Kd = D  # (p-1)*10

    def calc(self, setpoint, measurement):

        error = measurement - setpoint

        p = error
        self._i += p
        d = p - self._lastP

        self._lastP = p

        return int(self._Kp * p + self._Ki * self._i + self._Kd * d)
