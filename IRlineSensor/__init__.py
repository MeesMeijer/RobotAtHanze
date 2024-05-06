from machine import Pin, ADC


def normalize(x: int):
    return (x - 0) * (1000 - 0) // (65535 - 0) + 0




class readData:
    def __init__(self, data):
        self.data = data
        # [A1, A2, A3, A4, A5] = data

    def __str__(self):
        return f"{self.data}"

    def __getitem__(self, item: int):
        if type(item) is type(int()):
            return self.data[item]
        raise Exception("Only supports types of ints!")

    def toBooleans(self, whiteThreshold: int) -> list[bool]:
        return [bool(x < whiteThreshold) for x in self.data]


class LineSensor:
    def __init__(self, A1: Pin, A2: Pin, A3: Pin, A4: Pin, A5: Pin):
        self.A1 = ADC(A1, atten=ADC.ATTN_11DB)
        self.A2 = ADC(A2, atten=ADC.ATTN_11DB)
        self.A3 = ADC(A3, atten=ADC.ATTN_11DB)
        self.A4 = ADC(A4, atten=ADC.ATTN_11DB)
        self.A5 = ADC(A5, atten=ADC.ATTN_11DB)

    def read(self) -> readData:
        """Read between A1-A5 LOWER = BLACK 0-1000 """

        return readData([
            normalize(self.A1.read_u16()),
            normalize(self.A2.read_u16()),
            normalize(self.A3.read_u16()),
            normalize(self.A4.read_u16()),
            normalize(self.A5.read_u16())
        ])

    def readRaw(self) -> list[float]:
        """Read between A1-A5"""
        return [
            self.A1.read_u16(),
            self.A2.read_u16(),
            self.A3.read_u16(),
            self.A4.read_u16(),
            self.A5.read_u16()
        ]

    def movingAvarage(self):
        readings = self.read()
        return (
                -2000*readings[0] + -1000*readings[1] + 0*readings[2] + 1000*readings[3] + 2000*readings[4]
        ) / (sum(readings))

