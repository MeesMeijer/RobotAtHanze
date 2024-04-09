import machine
from MCP23017 import MCP23017

class DCMotor:
    _speed = 0
    BACKWARDS = (1,0)
    FORWARDS = (0,1)
    STOP = (0,0)

    def __init__(self, mcp, pin1, pin2, enable_pin, min_duty=50000, max_duty=65535):
        self.MCP: MCP23017 = mcp
        self.pin1 = pin1
        self.pin2 = pin2
        self.enable_pin: machine.PWM = enable_pin
        self.min_duty = min_duty
        self.max_duty = max_duty

    def setState(self, state: tuple[int,int]):
        self.MCP.pin(self.pin1, value=state[0])
        self.MCP.pin(self.pin2, value=state[1])

        # if state == self.STOP:
        #     return self.setSpeed(0)

    def setSpeed(self, speed: float):
        self.enable_pin.duty_u16(self._duty_cycle(speed))

    # def forward(self, speed):
    #     self._speed = speed if speed < 100 else 100
    #     self.enable_pin.duty_u16(self._duty_cycle(self._speed))
    #     self.MCP.pin(self.pin1, value=0)
    #     self.MCP.pin(self.pin2, value=1)
    #
    # def backward(self, speed):
    #     self._speed = speed if speed < 100 else 100
    #     self.enable_pin.duty_u16(self._duty_cycle(self._speed))
    #     self.MCP.pin(self.pin1, value=1)
    #     self.MCP.pin(self.pin2, value=0)

    # def stop(self):
    #     self.enable_pin.duty_u16(0)
    #     self.MCP.pin(self.pin1, value=0)
    #     self.MCP.pin(self.pin2, value=0)

    def _duty_cycle(self, speed: float):
        """ speed: 'int' 0-100% motor power """

        if int(speed) <= 0: return 0
        elif speed > 100: speed = 100

        return int(self.min_duty + (self.max_duty - self.min_duty)*((speed-1)/(100-1)))
