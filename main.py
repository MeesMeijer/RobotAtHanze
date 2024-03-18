from DCMOTOR import DCMotor
import machine
import time
from machine import Pin, PWM
from mpc23017 import MCP23017
from SR04 import HCSR04
from imu import MPU6050
from AS5600 import AS5600, ASMethods
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

SWITCH = Pin(LIMIT_SWITCH, Pin.IN)

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


def checkI2CDevices():
    busA = [0x36, 0x68, 0x20]
    busB = [0x36]

    devicesA = I2CA.scan()
    devicesB = I2CB.scan()

    # TODO: Check if this works
    print("DEBUG", devicesA)
    print("DEBUG", devicesB)

    found = 0
    for addr in busA:
        if addr not in devicesA:
            print(hex(addr), "Expected, but not found in busA")
        else:
            found += 1

    for addr in busB:
        if addr not in devicesB:
            print(hex(addr), "Expected, but not found in busB")
        else:
            found += 1

    return True if found == 4 else False

def driveToBoxAndConnect():
    while not SWITCH.value():
        # TODO: drive straight on the line to the box.
        pass


# Blocking when magnet is not detected by sensors.
#TODO: Check if this works
while not ASL.MD or not ASR.MD:
    # L = 1 if ASL.MD else 0
    print("Magnet not detected by AS5600 L/R", ASL.MD, ASR.MD)
    time.sleep(0.3)


# defining start angles.
MASL = ASMethods(ASL.RAWANGLE)
MASR = ASMethods(ASR.RAWANGLE)


# TODO: Check if this works
# degAngleL = MASL.toDeg(ASL.RAWANGLE)
# totalAngleL = MASL.checkQuadrant(degAngle)
# posL = totalAngleL / 0.45

# r,g,b = TCS.rgb()


while True:
    # TODO: Check if A3 and A4 are fliped.
    print(LINE.read())
    time.sleep(0.2)