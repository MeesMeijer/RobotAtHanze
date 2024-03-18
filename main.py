from DCMOTOR import DCMotor
import machine
import time
from machine import Pin, PWM
from mpc23017 import MCP23017
from SR04 import HCSR04
from imu import MPU6050
from AS5600 import AS5600
from TCS3200 import TCS3200
from pins import *
from linesensor import LineSensor

frequency = 15000

I2CA = machine.I2C(0, sda=Pin(SDA), scl=Pin(SCL))
I2CB = machine.I2C(1, sda=Pin(SDA2), scl=Pin(SCL2))

MCP = MCP23017(I2CB, 0x20)
MPU = MPU6050(I2CB)

ASL = AS5600(I2CB, 0x36)
ASR = AS5600(I2CA, 0x36)

SONIC = HCSR04(echo_pin=4, trigger_pin=2)
LINE = LineSensor(Pin(A1, Pin.IN), Pin(A2, Pin.IN), Pin(A3, Pin.IN), Pin(A4, Pin.IN), Pin(A5, Pin.IN))

MPU.wake()
MCP.init()

MCP.pin(IN1, mode=0, value=0)  # in1
MCP.pin(IN2, mode=0, value=0)  # in2
MCP.pin(IN3, mode=0, value=0)  # in3
MCP.pin(IN4, mode=0, value=0)  # in4
MCP.pin(S0, mode=0, value=1)  # S0
MCP.pin(S1, mode=0, value=0)  # S1
MCP.pin(S2, mode=0, value=0)  # S2
MCP.pin(S3, mode=0, value=0)  # S3
MCP.pin(LED, mode=0, value=0)  # LED
MCP.pin(DIRA, mode=0, value=0)
MCP.pin(DIRB, mode=0, value=0)

motorL = DCMotor(MCP, IN1, IN2, PWM(Pin(ENA), freq=frequency))
motorR = DCMotor(MCP, IN3, IN4, PWM(Pin(ENB), freq=frequency))

# Color sensor
TCS = TCS3200(MCP, S2, S3, LED, Pin(OUT, Pin.IN, Pin.PULL_UP))
while True:
    # r = TCS.get(TCS3200.RED)
    # g = TCS.get(TCS3200.GREEN)
    # b = TCS.get(TCS3200.BLUE)
    #
    # print(f"R {r} G {g} B {b}")
    print(LINE.read())
    time.sleep(0.2)