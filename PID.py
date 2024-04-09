import time


class PID:
    _i = 0
    _prevE = 0
    _Kp, _Ki, _Kp = 0, 0, 0
    _lastCalc = 0

    def __init__(self, P=None, I=None, D=None):
        self._Kp = P  # 5
        self._Ki = I  # 0
        self._Kd = D  # (p-1)*10
        self._lastCalc = time.ticks_ms()

    def calc(self, data):
        delta_t = time.ticks_diff(time.ticks_ms(), self._lastCalc) / 1000
        P = self._Kp * data
        I = self._i + self._Ki * data * delta_t
        D = self._Kd * (data - self._prevE) / delta_t
        self._lastCalc = time.ticks_ms()

        return P + I + D

