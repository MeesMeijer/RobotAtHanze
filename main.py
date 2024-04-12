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
from motors.driver import MotorDriver
from filters import MedianFilter
from PID import PID
from statemachine import States
from requests import request
import uheapq

frequency = 15000
WHITE_THRESHOLD = 770

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

leftMotor = DCMotor(MCP, IN1, IN2, PWM(Pin(ENA), freq=frequency))
rightMotor = DCMotor(MCP, IN4, IN3, PWM(Pin(ENB), freq=frequency))
DRIVER = MotorDriver(leftMotor, rightMotor)

# Color sensor
TCS = TCS3200(MCP, S2, S3, LED, Pin(OUT, Pin.IN, Pin.PULL_UP))

PID = PID(3, 0, 0)
SONIC_FILTER = MedianFilter(window_size=10)


def checkI2CDevices():
    busB = [0x36]
    busA = [0x36, 0xd, 0x20]

    devicesA = I2CA.scan()
    devicesB = I2CB.scan()

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


getState = States(States.STOP)

leftSpeed, rightSpeed = 0, 0

prevReadings: list[bool] = [False, False, False, False, False]
lastSeen = [0, 0, 0, 0, 0]
nextState = None

while True:
    # Read the IR sensor data and make it Digital
    # A1...A5 == Analog pin 1..5
    # D1...D5 == The digital representation with WHITE_THRESHOLD
    lineSensorData = [A1, A2, A3, A4, A5] = LINE.read()
    sensorsByBooleans = [D1, D2, D3, D4, D5] = lineSensorData.toBooleans(WHITE_THRESHOLD)

    # UPDATE currentState with the IR sensor data
    sensorState = getState[sensorsByBooleans]

    if nextState is not None:
        currentState = nextState
    else:
        currentState = sensorState

    temp: list[int] = [0, 0, 0, 0, 0]
    for i in range(len(prevReadings)):
        ixs = sensorsByBooleans[i]
        ixps = prevReadings[i]

        if ixps and not ixs:
            lastSeen[i] = time.ticks_ms()
            temp[i] = 1

        if not ixps and not ixs:
            lastSeen[i] = -1


    # Update / get the Median filter with new data
    SONIC_FILTER.update(SONIC.distance_cm())
    distance = SONIC_FILTER.get_median()

    tempStates = {
        (1,0,0,0,0): States.LEFT_CORNER,
        (1,1,0,0,0): States.LEFT_CORNER,
        (1,1,1,0,0): States.LEFT_CORNER,
        (0,0,0,0,1): States.RIGHT_CORNER,
        (0,0,0,1,1): States.RIGHT_CORNER,
        (0,0,1,1,1): States.RIGHT_CORNER
    }

    # if (tempState := tempStates[(tuple(temp))]) != States.STOP:
    #     currentState = tempState

    # if distance < 20 and not SWITCH.value():
    #     print("OBJECT!, COLOR: " + str(TCS.detectColor()))
    #     currentState = States.PICK_UP_BOX
    #
    # elif distance < 20 and SWITCH.value():
    #     print("Not a box, turning")
    #     currentState = States.OBSTACLE

    #Statemachine
    if currentState == States.STOP:
        leftSpeed = 0
        rightSpeed = 0

    elif currentState == States.STRAIGHT:
        # TODO: Check the MAPPING
        MovingAverage = -2.5 * A1 - 2 * A2 + 0 * A3 + 2 * A4 + 2.5 * A5
        output = PID.calc(MovingAverage / 200)
        # print("pid", output)
        x = 50
        leftSpeed = x - output
        rightSpeed = x + output

        # nextState = getState[LINE.read().toBooleans(WHITE_THRESHOLD)]
        nextState = None

    elif currentState == States.LEFT_CORNER:
        # TODO: Check the MAPPING

        DRIVER.drive(40, 40)
        time.sleep(0.5)
        DRIVER.drive(0, 0)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        # lineData[n] == An+1 so An == lineData[n-1]
        while not lineData[2]:
            DRIVER.drive(-25, 20)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        nextState = None

    elif currentState == States.RIGHT_CORNER:
        # TODO: Check the MAPPING
        DRIVER.drive(40, 40)
        time.sleep(0.5)
        DRIVER.drive(0, 0)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        while not lineData[2]:
            DRIVER.drive(20, -25)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        nextState = None

    elif currentState == States.OBSTACLE:
        # TODO: Check the MAPPING
        # [TODO]: Do something with the information and then go to the next state

        nextState = States.TURN_AROUND

    elif currentState == States.TURN_AROUND:
        # TODO: Check the MAPPING
        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        # lineData[x] == A(x+1) so A1 == lineData[0]
        while (not lineData[1]) or (not lineData[2]):
            DRIVER.drive(-40, 50)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        for _ in range(9):
            SONIC_FILTER.update(120)
        # SONIC_FILTER.reset(120) # this is to not trick the if statement

        nextState = States.STRAIGHT

    # Todo: Check if this works..
    elif currentState == States.PICK_UP_BOX:
        # TODO: Check the MAPPING
        MAGNET.value(1)  # Enable the magnet.

        MovingAverage = -2 * A1 - 1 * A2 + 0 * A3 + 1 * A4 + 2 * A5
        output = PID.calc(MovingAverage / 200)

        x = 30
        leftSpeed = x - output
        rightSpeed = x + output

        nextState = States.PICK_UP_BOX

        if SWITCH.value():
            print("Got the box!, COLOR: " + TCS.detectColor())
            for _ in range(9):
                SONIC_FILTER.update(120)
            nextState = States.TURN_AROUND

        # nextState = States.PICK_UP_BOX if not SWITCH.value() else States.TURN_AROUND

    tempState = tempStates.get(tuple(temp))
    if tempState and not nextState and currentState == States.STOP:
        nextState = tempState

    DRIVER.drive(leftSpeed, rightSpeed)
    print(f"{currentState} {nextState} {tempState} {lineSensorData} {distance} {lastSeen}")
    # print(f"C: {currentState} N: {nextState} S: {sensorState}\n",
    #       f"LSB: {sensorsByBooleans} LS: {lineSensorData} \n ",
    #       f"Distance: {distance}")

    prevReadings = sensorsByBooleans
    # time.sleep(0.2)