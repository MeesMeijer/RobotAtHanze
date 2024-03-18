import machine
from mpc23017 import MCP23017



class DCMotor:
    _speed = 0

    def __init__(self, mcp, pin1, pin2, enable_pin, min_duty=50000, max_duty=65535):
        self.MCP: MCP23017 = mcp
        self.pin1 = pin1
        self.pin2 = pin2
        self.enable_pin: machine.PWM = enable_pin
        self.min_duty = min_duty
        self.max_duty = max_duty

    def forward(self, speed):
        self._speed = speed
        self.enable_pin.duty_u16(self._duty_cycle(self._speed))
        self.MCP.pin(self.pin1, value=0)
        self.MCP.pin(self.pin2, value=1)

    def backwards(self, speed):
        self._speed = speed
        self.enable_pin.duty_u16(self._duty_cycle(self._speed))
        # self.pin2.value(0)
        # self.pin1.value(1)

        self.MCP.pin(self.pin1, value=1)
        self.MCP.pin(self.pin2, value=0)

    def stop(self):
        self.enable_pin.duty_u16(0)
        # self.pin1.value(0)
        # self.pin2.value(0)
        self.MCP.pin(self.pin1, value=0)
        self.MCP.pin(self.pin2, value=0)

    def _duty_cycle(self, speed):
        if self._speed <= 0 or self._speed > 100:
            duty_cycle = 0
        else:
            duty_cycle = int(self.min_duty + (self.max_duty - self.min_duty)*((self._speed-1)/(100-1)))
        return duty_cycle
