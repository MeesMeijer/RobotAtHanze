from motors import DCMotor
import machine
import time
from machine import Pin, PWM
from MCP23017 import MCP23017
from HSR04 import HCSR04
from COMPAS import QMC5883L
from AS5600 import AS5600, ASMethods
from TCS3200 import TCS3200
from pins import *
from IRlineSensor import LineSensor

class MedianFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []
    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)

    def get_median(self):
        sorted_values = sorted(self.values)
        window_midpoint = len(sorted_values) // 2
        if len(sorted_values) % 2 == 0:
            return (sorted_values[window_midpoint - 1] + sorted_values[window_midpoint]) / 2
        else:
            return sorted_values[window_midpoint]
    def reset(self):
        self.values = [120]


frequency = 15000

I2CA = machine.I2C(0, sda=Pin(SDA), scl=Pin(SCL))
I2CB = machine.I2C(1, sda=Pin(SDA2), scl=Pin(SCL2))

MCP = MCP23017(I2CA, 0x20)
COMP = QMC5883L(i2c=I2CA, offset=50.0)

ASL = AS5600(I2CB, 0x36)
ASR = AS5600(I2CA, 0x36)

# MASL = ASMethods(ASL.RAWANGLE)
# MASR = ASMethods(ASR.RAWANGLE)


SONIC = HCSR04(echo_pin=ECHO, trigger_pin=TRIG)
LINE = LineSensor(Pin(A2, Pin.IN), Pin(A2, Pin.IN), Pin(A3, Pin.IN), Pin(A4, Pin.IN), Pin(A5, Pin.IN))

SWITCH = Pin(LIMIT_SWITCH, Pin.IN)
MAGNET = Pin(TRIG_MAGNET, Pin.OUT, value=0)
# MPU.wake()
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
MCP.pin(DIRB, mode=0, value=1)

motorL = DCMotor(MCP, IN1, IN2, PWM(Pin(ENA), freq=frequency))
motorR = DCMotor(MCP, IN4, IN3, PWM(Pin(ENB), freq=frequency))

# Color sensor
TCS = TCS3200(MCP, S2, S3, LED, Pin(OUT, Pin.IN, Pin.PULL_UP))


def checkI2CDevices():
    busB = [0x36]
    busA = [0x36, 0xd, 0x20]

    devicesA = I2CA.scan()
    devicesB = I2CB.scan()

    # TODO: Check if this works
    print("DEBUGA", [hex(d) for d in devicesA])
    print("DEBUGB", [hex(dc) for dc in devicesB])

    found = 0
    for addra in busA:
        if addra not in devicesA:
            print(hex(addra), "Expected, but not found in busA")
        else:
            found += 1

    for addrb in busB:
        if addrb not in devicesB:
            print(hex(addrb), "Expected, but not found in busB")
        else:
            found += 1

    return True if found == 4 else False


# while True:
#     [A1, A2, A3, A4, A5] = LINE.readRaw()
#     print([A1, A2, A3, A4, A5])
#     time.sleep(0.2)

# while True:
#     r,g,b  = TCS.rgb()
#     print(r,g,b,)
#     time.sleep(0.2)
WHITE_TRESHHOLD = 700


def toInts(readings: list[int]) -> list[bool]:
    return [bool(x < WHITE_TRESHHOLD) for x in readings]


def only(pos: int, state: bool, readings: list[bool]) -> bool:
    x_p = 0
    for x in readings:
        if x == state and x_p != pos: return False
        x_p += 1
    return True


STATES = ['STRAIGHT', 'LEFT_CORNER', "RIGHT_CORNER"]

states = {
    (0, 0, 1, 0, 0): STATES[0],
    (0, 1, 0, 0, 0): STATES[0],
    (0, 0, 0, 1, 0): STATES[0],
    (0, 1, 1, 0, 0): STATES[0],
    (0, 0, 1, 1, 0): STATES[0],
    (0 ,1, 1, 1, 0): STATES[0],
    (0, 0, 0, 1, 1): STATES[0],
    (1, 1, 0, 0, 0): STATES[0],
    (1, 1, 1, 0, 0): STATES[0],
    (0, 0, 1, 1, 1): STATES[0],
    (0, 1, 1, 1, 1): STATES[2],
    (1, 1, 1, 1, 0): STATES[1],
    (0, 0, 0, 0, 0): "STOP"
}


# Function to control robot movement based on sensor inputs
def control_robot(A1, A2, A3, A4, A5):
    state = (A1, A2, A3, A4, A5)
    if state in states:
        return states[state]
    else:
        return "STOP"


def drive(left, right):
    if left < 0:
        motorL.backward(abs(left)+20)
        time.sleep_ms(10)
        motorL.backward(abs(left))
    else:
        # motorL.forward(abs(left)+20)
        # time.sleep_ms(10)
        motorL.forward(abs(left))

    if right < 0:
        motorR.backward(abs(right)+20)
        time.sleep_ms(10)
        motorR.backward(abs(right))
    else:
        # motorR.forward(abs(right)+20)
        # time.sleep_ms(10)
        motorR.forward(abs(right))


currentState = STATES[0]
movements = []
steps = 0
leftSpeed, rightSpeed = 0, 0
prevReadings: list[int] = [0, 0, 0, 0, 0]
lastSeen = [0, 0, 0, 0, 0]
# left,right = 0,0
e_acc, e_prev = 0, 0

median_filter = MedianFilter(window_size=10)

def PID(e, e_acc, e_prev, delta_t, kp=1.0, kd=0.0, ki=0):
    P = kp * e
    I = e_acc + ki * e * delta_t
    D = kd * (e - e_prev) / delta_t

    return e, I, P + I + D

A1B, A2B, A3B, A4B, A5B = 0, 0, 0, 0, 0
last = 0

box_detected, objects = False, 0

while True:
    sensors = LINE.read()
    sensorsByBool = toInts(sensors)
    [A1, A2, A3, A4, A5] = sensorsByBool

    currentState = control_robot(A1, A2, A3, A4, A5)
    median_filter.update(SONIC.distance_cm())

    distance = median_filter.get_median()
    #
    if distance < 25 and not SWITCH.value():
        box_detected = True
        print("OBJECT!, COLOR: "+ str(TCS.rgb()))
        objects = 1
        currentState = STATES[0]
        MAGNET.value(1)

    elif distance < 15 and SWITCH.value():
        print("Not a box object, turning")

        [A1B, A2B, A3B, A4B, A5B] = toInts(LINE.read())
        while (not A2B) or (not A3B):
            drive(-50, 60)
            [A1B, A2B, A3B, A4B, A5B] = toInts(LINE.read())

        median_filter.values = [ 120,120,120,120,120,120,120,120,120,120 ]
        currentState = STATES[0]

    if not SWITCH.value():
        box_detected = False

    if currentState == "STOP":
        leftSpeed = 0
        rightSpeed = 0

    if currentState == STATES[0]:
        MovingAverage = -2 * sensors[0] - 1 * sensors[1] + 0 * sensors[2] + 1 * sensors[3] + 2 * sensors[4]
        e_prev, e_acc, output = PID(MovingAverage / 100, e_acc, e_prev, time.ticks_diff(time.ticks_ms(), last) / 1000,
                                    7, 0, 0)
        last = time.ticks_ms()
        e_acc, e_prev = e_acc, e_prev

        # x = 50 if SWITCH.value() else 60
        # x *= 1.7 if SWITCH.value() else 1
        x = 50
        leftSpeed = x - output
        rightSpeed = x + output


    elif currentState == STATES[1]:
        drive(40, 40)
        time.sleep(0.5)
        drive(0, 0)

        [A1B, A2B, A3B, A4B, A5B] = toInts(LINE.read())
        while (not A2B) or (not A3B):
            drive(-50, 60)
            [A1B, A2B, A3B, A4B, A5B] = toInts(LINE.read())

        leftSpeed = 0
        rightSpeed = 0

    elif currentState == STATES[2]:
        drive(40, 40)
        time.sleep(0.5)
        drive(0, 0)

        [A1B, A2B, A3B, A4B, A5B] = toInts(LINE.read())
        while (not A4B) or (not A5B):
            drive(60, -50)
            [A1B, A2B, A3B, A4B, A5B] = toInts(LINE.read())

        leftSpeed = 0
        rightSpeed = 0

    drive(leftSpeed, rightSpeed)
    print(f"{currentState} {sensors} {distance}")
    prevReadings = sensors
    # time.sleep(0.2)
