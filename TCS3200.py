import time

import machine

from mpc23017 import MCP23017
from machine import Pin

class TCS3200:
    cyles = 10
    signal = 0

    RED = (0,0)
    GREEN = (1,1)
    BLUE = (0,1)
    CLEAR = (1,0)

    def __init__(self, MCP: MCP23017, S2, S3, LED, OUT):
        self._MCP = MCP
        self.S2 = S2
        self.S3 = S3
        self.LED = LED
        self.OUT = OUT

    def get(self, color: tuple[int, int]):
        self._MCP.pin(self.S2, value=color[0])
        self._MCP.pin(self.S3, value=color[1])

        time.sleep(0.2)
        start = time.ticks_us()
        for _ in range(self.cyles):
            machine.time_pulse_us(self.OUT, 0)
        duration = time.ticks_diff(time.ticks_us(), start)

        return self.cyles*(1_000_000) / (duration)

    def rgb(self) -> tuple[float, float, float]:
        """ returns (r,g,b) """

        return (
            self.get(self.RED),
            self.get(self.GREEN),
            self.get(self.BLUE),
        )