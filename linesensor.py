from machine import Pin, ADC


def normalize(x: int):
    in_min, out_min = 0, 0
    in_max, out_max = 65535, 1000
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    # (23749) * (-64535) // (0-0) + 10000

class LineSensor:
    def __init__(self, A1: Pin, A2: Pin, A3: Pin, A4: Pin, A5: Pin):
        self.A1 = ADC(A1)
        self.A2 = ADC(A2)
        self.A3 = ADC(A3)
        self.A4 = ADC(A4)
        self.A5 = ADC(A5)

    def read(self) -> list[int]:
        """Read between A1-A5"""

        return [
            normalize(self.A1.read_u16()),
            normalize(self.A2.read_u16()),
            normalize(self.A3.read_u16()),
            normalize(self.A4.read_u16()),
            normalize(self.A5.read_u16())
        ]

    def movingAvarage(self):
        readings = self.read()
        return (
            -2000*readings[0] + -1000*readings[1] + 0*readings[2] + 1000*readings[3] + 2000*readings[4]
        ) / (sum(readings))

