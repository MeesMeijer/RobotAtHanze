from motors import DCMotor
import machine
import time
from machine import Pin, PWM
from MCP23017 import MCP23017
from HSR04 import HCSR04
from TCS3200 import TCS3200
from pins import *
from IRlineSensor import LineSensor
from motors.driver import MotorDriver
from filters import MedianFilter
from PID import PID
from statemachine import States
from mapping import Heading, Graph, nodes, BoxPlaces, HeadingsToState
from coms import Coms

frequency = 15000
WHITE_THRESHOLD = 850
ROBOT_DATA_PREFIX, DASH_DATA_PREFIX = ">", "<"

I2CA = machine.I2C(0, sda=Pin(SDA), scl=Pin(SCL))
I2CB = machine.I2C(1, sda=Pin(SDA2), scl=Pin(SCL2))

MCP = MCP23017(I2CA, 0x20)

SONIC = HCSR04(echo_pin=ECHO, trigger_pin=TRIG)
LINE = LineSensor(Pin(A1, Pin.IN), Pin(A2, Pin.IN), Pin(A3, Pin.IN), Pin(A4, Pin.IN), Pin(A5, Pin.IN))

# SWITCH = Pin(LIMIT_SWITCH, Pin.IN)
MAGNET = Pin(TRIG_MAGNET, Pin.OUT, value=0)

MCP.init()
MCP.pin(IN1, mode=0, value=1)  # in1
MCP.pin(IN2, mode=0, value=1)  # in2
MCP.pin(IN3, mode=0, value=1)  # in3
MCP.pin(IN4, mode=0, value=1)  # in4
MCP.pin(S0, mode=0, value=1)  # S0
MCP.pin(S1, mode=0, value=0)  # S1
MCP.pin(S2, mode=0, value=0)  # S2
MCP.pin(S3, mode=0, value=0)  # S3
MCP.pin(LED, mode=0, value=0)  # LED
MCP.pin(DIRA, mode=0, value=0)
MCP.pin(DIRB, mode=0, value=0)

leftMotor = DCMotor(MCP, IN1, IN2, PWM(Pin(ENA), freq=frequency))
rightMotor = DCMotor(MCP, IN4, IN3, PWM(Pin(ENB), freq=frequency))
DRIVER = MotorDriver(leftMotor, rightMotor)

# Color sensor
TCS = TCS3200(MCP, S2, S3, LED, Pin(OUT, Pin.IN, Pin.PULL_UP))

PID = PID(4, 0, 0)
SONIC_FILTER = MedianFilter(window_size=10)

# a com protecol using esp-now
com = Coms(peer=b'\xc8\xf0\x9e\xf2X\xe8')

# FRONT or BACK Set where to line sensor is placed.
SENSOR_PLACEMENT = "FRONT"  # or BACK


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
# nextState = None

#TODO: Make this Dynamic

graph = Graph(nodes)

startingPoint = "E"
endpoint = "W"
# graph.BlockConnection("W", "V")
# graph.BlockConnection("O", "L")
# graph.BlockConnection("N", "Q")
cost, shortestPath, pathDirections = graph.dijkstra(startingPoint, endpoint)
print(shortestPath, pathDirections)

posIndx = 0
# if startingPoint == shortestPath[0]: posIndx +=1
currentPos = shortestPath[posIndx]
currentHeading = nodes[shortestPath[posIndx]][shortestPath[posIndx+1]][1]
nextPos, nextHeading, nextState = None, None, None



timeToIntersection = 0
fromTCross = False
IntersectionButStraight = False

while True:
    # Read the IR sensor data and make it Digital
    # A1...A5 == Analog pin 1..5
    # D1...D5 == The digital representation with WHITE_THRESHOLD
    lineSensorData = [A1, A2, A3, A4, A5] = LINE.read()
    sensorsByBooleans = [D1, D2, D3, D4, D5] = lineSensorData.toBooleans(WHITE_THRESHOLD)

    # Update / get the Median filter with new data
    SONIC_FILTER.update(SONIC.distance_cm())
    distance = SONIC_FILTER.get_median()

    # temp: list[int] = [0, 0, 0, 0, 0]
    # for i in range(len(prevReadings)):
    #     ixs = sensorsByBooleans[i]
    #     ixps = prevReadings[i]
    #
    #     if ixps and not ixs:
    #         lastSeen[i] = time.ticks_ms()
    #         temp[i] = 1
    #
    #     if not ixps and not ixs:
    #         lastSeen[i] = 0
    #         temp[i] = 0

    # UPDATE currentState with the IR sensor data
    sensorState = getState[sensorsByBooleans]
    if nextState is not None:
        currentState, currentPos = nextState, nextPos
        currentHeading = nextHeading
        nextPos, nextHeading, nextState = None, None, None
    else:
        if nextPos:
            currentPos = nextPos

        if nextHeading:
            currentHeading = nextHeading

        currentState = sensorState
    # currentState = States.PICK_UP_BOX

    if distance < 15:
        currentState = States.OBSTACLE

    if posIndx < len(shortestPath)-1:
        print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx +1], "), Head:", nodes[currentPos][shortestPath[posIndx+1]][1], "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")
    else:
        print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx], "), Head:", pathDirections[-1], "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")


    # Statemachine
    if currentState == States.STOP:
        leftSpeed = 0
        rightSpeed = 0

        if posIndx + 2 >= len(shortestPath) or posIndx + 1 >= len(shortestPath):
            print("OP BESTEMMING.. " + currentPos)
            nextPos, nextHeading = currentPos, currentHeading
        nextState = None

    elif currentState == States.AT_GOAL:
        leftSpeed = 0
        rightSpeed = 0

        print("ON GOAL.. " + currentPos)
        nextPos, nextHeading, nextState = currentPos, currentHeading, States.AT_GOAL

    elif currentState == States.STRAIGHT:
        if posIndx == len(shortestPath) -1:
            print("OP BESTEMMING STRAIGHT.. " + currentPos)
            nextPos, nextHeading, nextState = currentPos, currentHeading, States.AT_GOAL
        else:
            if IntersectionButStraight:
                A1, A2, A3, A4, A5 = 900, 900, 700, 900, 900
                IntersectionButStraight = False

            MovingAverage = -2 * A1 - 1.5 * A2 + 0 * A3 + 1.5 * A4 + 2 * A5
            output = PID.calc(MovingAverage / 80)

            x = 50
            leftSpeed = x - output
            rightSpeed = x + output

            nextPos, nextHeading, nextState = currentPos, currentHeading, None
            IntersectionBuStraightLeft, IntersectionBuStraightRight = False, False

        fromTCross = False

    elif currentState in [States.LEFT_CORNER, States.RIGHT_CORNER, States.T_CROSS]:

        if posIndx == len(shortestPath) -1:
            print("OP BESTEMMING CORNER.. " + shortestPath[posIndx])
            nextPos, nextHeading, nextState = currentPos, currentHeading, States.AT_GOAL
        else:
            # TODO: Check if works with the new setup
            if time.ticks_diff(time.ticks_ms(), timeToIntersection) < 0.5 * 1000:
                print("this is duplicate", time.ticks_diff(time.ticks_ms(), timeToIntersection))
                IntersectionBuStraightLeft = True
                nextPos, nextHeading, nextState = currentPos, currentHeading, States.STRAIGHT
            else:
                posIndx += 1

                # if len(shortestPath) -1 == posIndx
                # desiredPos = None
                if posIndx < len(shortestPath) -1:
                    desiredPos = shortestPath[posIndx + 1]
                    # desiredHeading = pathDirections[posIndx-2]
                    desiredHeading = nodes[shortestPath[posIndx]][desiredPos][1]
                else:
                    desiredPos = shortestPath[posIndx]
                    # desiredHeading = pathDirections[posIndx]
                    desiredHeading = nodes[currentPos][desiredPos][1]
                    print("At end of shortlist")

                print(currentPos,desiredPos, posIndx)
                # desiredPos = shortestPath[posIndx + 1] if posIndx + 1 <= len(shortestPath) else shortestPath[posIndx]

                desiredState = HeadingsToState[currentHeading][desiredHeading]
                print(f"Desired Pos: {desiredPos} Heading: {desiredHeading} State: {desiredState}")

                if currentState != desiredState:
                    print(f"State is: {currentState} But iam going: {desiredState}")

                    if desiredState == States.STRAIGHT:
                        IntersectionButStraight = True

                timeToIntersection = time.ticks_ms()

                if desiredState == States.LEFT_CORNER:
                    if SENSOR_PLACEMENT == "FRONT":
                        print("turning left ")
                        DRIVER.drive(40, 40)
                        time.sleep(0.4)
                        DRIVER.drive(-40, 40)
                        time.sleep(0.3)

                    lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    while not lineData[3]:
                        DRIVER.drive(-40, 40)
                        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

                    nextState = States.STRAIGHT

                elif desiredState == States.RIGHT_CORNER:
                    if SENSOR_PLACEMENT == "FRONT":
                        print("turning left ")
                        DRIVER.drive(40, 40)
                        time.sleep(0.4)
                        DRIVER.drive(40, -40)
                        time.sleep(0.3)

                    lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    while not lineData[1]:
                        DRIVER.drive(40, -40)
                        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

                    nextState = States.STRAIGHT

                elif desiredState == States.STRAIGHT:
                    nextState = States.STRAIGHT

                nextPos, nextHeading = shortestPath[posIndx], desiredHeading

    elif currentState == States.PICK_UP_BOX:
        #TODO: This only works turning to the right... fix this or make it dynamic
        DRIVER.drive(50,50)
        time.sleep(0.6)
        DRIVER.drive(40, -40)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        while not lineData[2]:
            DRIVER.drive(40, -40)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        MAGNET.value(1)

        DRIVER.drive(30,32)
        time.sleep(0.5)
        DRIVER.drive(-40,-40)
        time.sleep(0.7)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        while lineData[2]:
            DRIVER.drive(-65, 85)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        time.sleep(0.4)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        while not lineData[2]:
            DRIVER.drive(-65, 85)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        DRIVER.drive(0,0)

        raise Exception("error")

    elif currentState == States.OBSTACLE:

        if posIndx < len(shortestPath)-1:
            A, B = currentPos, shortestPath[posIndx +1]
            headingAB = nodes[currentPos][shortestPath[posIndx+1]][1]
            # print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx +1], "), Head:", , "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")
        else:
            A, B = currentPos, shortestPath[posIndx]
            headingAB = pathDirections[-1]
            # print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx], "), Head:", , "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")

        print(f"detected a obstacle between {A}:{B}")
        print(f"currentHeading: {currentHeading}, turn with heading: {nodes[B][A][1]}, from {B}:{A}, with state: ")

        nextPos, nextState, nextHeading = shortestPath[posIndx-1], HeadingsToState[currentHeading][nodes[B][A][1]], nodes[B][A][1]


    prevReadings = sensorsByBooleans

    DRIVER.drive(leftSpeed, rightSpeed)
#    com.send(f"{ROBOT_DATA_PREFIX}STRAIGHT,"+str(utime.time()))

#     if com.available():
#         mac, rev = com.recv(5)
#         print("GOT", rev.decode("utf-8"), len(rev.decode("utf-8")), "isCommand", rev.decode().startswith(DASH_DATA_PREFIX))
#         # print(rev[1].decode("utf-8"))


#         # ints = int(rev[1].decode("utf-8").split(":")[1]) + 1
    # time.sleep_ms(20)