



class PID:
    _i = 0
    _lp = 0

    def __init__(self, P=None, I=None, D=None):
        self._Kp = P
        self._Ki = I
        self._Kd = D

    def calc(self, setpoint,  mesurement):

        error = mesurement-setpoint;

        p = error;
        self._i += p;
        d = p - self._lp;

        self._lp = p;

        return int(self._Kp *p + self._Ki*self._i + self._Kd*d);
