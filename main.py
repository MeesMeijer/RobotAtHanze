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

pid_forwards = PID(4, 0, 0)
pid_backwards = PID(4,0,0)
filter_for_distance = MedianFilter(window_size=10)

# a com protecol using esp-now
now = Coms(peer=b'\xc8\xf0\x9e\xf2X\xe8')

def isCommand(msg: str) -> bool:
    return msg.startswith(DASH_DATA_PREFIX)

def onCommand(cmd:str):
    # struct: prefix,SET/GET,WHAT,DATA
    if not isCommand(cmd): return





def drive_for(timout:float, speed: int):
    pid_backwards.reset()

    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < (timout*1000):
        [A1, A2, A3, A4, A5] = LINE.read()
        #TODO: Check if works when going backwards..
        inp = (-2 * A1 - 1.5 * A2 + 0 * A3 + 1.5 * A4 + 2 * A5) / 80
        apt = pid_backwards.calc(inp)

        DRIVER.drive(speed-apt, speed+apt)
        time.sleep_ms(50)

    DRIVER.drive(0,0)


def checkIfBoxEndpoint(Cpos,Npos):
    boxPickupMates = ["E:A", "F:B", "G:C", "H:D"]
    boxReleasemates = ["T:X", "U:Y", "V:Z", "W:AA"]

    if f"{Cpos}:{Npos}" in boxPickupMates:
        return True, "PICK_BOX"

    elif f"{Cpos}:{Npos}" in boxReleasemates:
        return True, "RELEASE_BOX"

    return False, "ERROR"


def checkI2CDevices(send: bool = False):
    devicesA = I2CA.scan()
    devicesB = I2CB.scan()

    print("[debug] - I2CA Scan Result:", [hex(da) for da in devicesA])
    print("[debug] - I2CB Scan Result:", [hex(db) for db in devicesB])
    
    if send: 
        now.send(f"{ROBOT_DATA_PREFIX},SET,I2C_SCAN,{[devicesA, devicesB]}")


getState = States(States.STOP)

boxesToPick = [
    "D", "C", "B", "A"
]
force = False
graph = Graph(nodes)

startCel = ["I","H"]
endpoint = boxesToPick[1]

cost, shortestPath, pathDirections = graph.dijkstra(startCel[0], endpoint)
print(shortestPath, pathDirections)
now.send(f"{ROBOT_DATA_PREFIX},CURRENT_PATH,{shortestPath}")

posIndx = 0
currentPos = startCel[0]
currentHeading = nodes[startCel[0]][startCel[1]][1]
nextPos, nextHeading, nextState = None, None, None

START_ROBOT = True
timeToIntersection = 0
fromTCross = False
IntersectionButStraight = False
leftSpeed, rightSpeed = 0, 0
while True:
    # Read the IR sensor data and make it Digital
    # A1...A5 == Analog pin 1..5
    # D1...D5 == The digital representation with WHITE_THRESHOLD
    lineSensorData = [A1, A2, A3, A4, A5] = LINE.read()
    sensorsByBooleans = [D1, D2, D3, D4, D5] = lineSensorData.toBooleans(WHITE_THRESHOLD)

    # Update / get the Median filter with new data
    SONIC_FILTER.update(raw := SONIC.distance_cm())
    distance = SONIC_FILTER.get_median()

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
    # currentState = States.STOP

    if not START_ROBOT:
        currentState = States.STOP

    # if distance < 15:
    #     currentState = States.OBSTACLE

    if posIndx < len(shortestPath)-1:
        now.send(f"{ROBOT_DATA_PREFIX},{currentState},{currentPos},{shortestPath[posIndx +1]},{nodes[currentPos][shortestPath[posIndx+1]][1]},{currentHeading},{lineSensorData},{distance},{raw}")
        print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx +1], "), Head:", nodes[currentPos][shortestPath[posIndx+1]][1], "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")
    else:
        now.send(f"{ROBOT_DATA_PREFIX},{currentState},{currentPos},{shortestPath[posIndx]},{pathDirections[-1]},{currentHeading},{lineSensorData},{distance},{raw}")
        print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx], "), Head:", pathDirections[-1], "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")

    # Statemachine
    if currentState == States.STOP:
        leftSpeed = 0
        rightSpeed = 0

        # drivePid(1.5, 60)
        # time.sleep(3)
        # drivePid(1.5, -60)

        if posIndx == len(shortestPath) -1:
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
            output = pid.calc(MovingAverage / 80)

            x = 60
            leftSpeed = x - output
            rightSpeed = x + output

            if posIndx < len(shortestPath) -1:
                desiredPos = shortestPath[posIndx + 1]
                # desiredHeading = nodes[shortestPath[posIndx]][desiredPos][1]
            else:
                desiredPos = shortestPath[posIndx]
                # desiredHeading = nodes[currentPos][desiredPos][1]
            desiredState = None
            isBoxPickUpState = checkIfBoxEndpoint(currentPos, desiredPos)
            if isBoxPickUpState:
                desiredState = States.PICK_UP_BOX

            nextPos, nextHeading, nextState = currentPos, currentHeading, desiredState
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

                if posIndx < len(shortestPath) -1:
                    desiredPos = shortestPath[posIndx + 1]
                    desiredHeading = nodes[shortestPath[posIndx]][desiredPos][1]
                else:
                    desiredPos = shortestPath[posIndx]
                    desiredHeading = nodes[currentPos][desiredPos][1]

                desiredState = HeadingsToState[currentHeading][desiredHeading]
                print(f"Desired Pos: {desiredPos} Heading: {currentHeading} -> {desiredHeading} State: {desiredState}")

                if currentState != desiredState and not force:
                    print(f"State is: {currentState} But iam going: {desiredState}")

                    if desiredState == States.STRAIGHT:
                        IntersectionButStraight = True
                else:
                    desiredState = currentState


                timeToIntersection = time.ticks_ms()

                if desiredState == States.LEFT_CORNER:
                    # print("turning left ")
                    # if SENSOR_PLACEMENT == "FRONT":
                    print("turning left ")
                    if not force:
                        drivePid(0.4, 60)
                        # time.sleep(0.4)

                    DRIVER.drive(-60, 60)
                    time.sleep(0.3)

                    lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    while not lineData[3]:
                        DRIVER.drive(-60, 60)
                        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    SONIC_FILTER.reset(120)
                    nextState = States.STRAIGHT

                elif desiredState == States.RIGHT_CORNER:
                    print("turning right ")

                    if not force:
                        drivePid(0.4, 60)
                        # time.sleep(0.4)
                    DRIVER.drive(80, -60)
                    time.sleep(0.5)

                    lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    while not lineData[2]:
                        DRIVER.drive(60, -100)
                        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    SONIC_FILTER.reset(120)
                    nextState = States.STRAIGHT

                elif desiredState == States.STRAIGHT:
                    nextState = States.STRAIGHT

                force = False
                nextPos, nextHeading = shortestPath[posIndx], desiredHeading

    elif currentState == States.PICK_UP_BOX:
        #TODO: This only works turning to the right... fix this or make it dynamic
        # DRIVER.drive(50,50)
        # time.sleep(0.6)
        # DRIVER.drive(40, -40)
        #
        # lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        # while not lineData[2]:
        #     DRIVER.drive(40, -40)
        #     lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        # left = False

        # speed = (80,-65) if left else (-65,80)
        # MAGNET.value(1)


        # PID_BACK.calc()

        drivePid(0.4, 45)
        # time.sleep(0.2)
        # drivePid(0.3,45)
        # time.sleep(0.3)

        drivePid(1, -50)

        # time.sleep(1)

        # DRIVER.drive(-70, 70)

        # lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        # while lineData[2]:
        #     DRIVER.drive(speed[0], speed[1])
        #     lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        #
        # time.sleep(0.8)
        #
        # lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        # while not lineData[2]:
        #     DRIVER.drive(speed[0], speed[1])
        #     lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        DRIVER.drive(0,0)
        SONIC_FILTER.reset(120)

        if endpoint == boxesToPick[1]:
            startCel = ["C", "G"]
            endpoint = BoxPlaces.BLACK

            cost, shortestPath, pathDirections = graph.dijkstra(startCel[0], endpoint)
            print(shortestPath, pathDirections)
            now.send(f"{ROBOT_DATA_PREFIX},CURRENT_PATH,{shortestPath}")

            posIndx = 0
            currentPos = startCel[0]
            currentHeading = nodes[startCel[1]][startCel[0]][1]
            nextHeading = nodes[shortestPath[1]][shortestPath[2]][1]
            nextPos, nextHeading, nextState = currentPos, nextHeading, HeadingsToState[currentHeading][nextHeading]
            force = True
            print(nextPos, nextHeading, nextState)

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
        print(f"currentHeading: {currentHeading}, turn with heading: {nodes[B][A][1]}, from {B}:{A}, with state: {HeadingsToState[currentHeading][nodes[B][A][1]]}")

        nextPos, nextState, nextHeading = A, HeadingsToState[currentHeading][nodes[B][A][1]], nodes[B][A][1]
    elif currentState == States.TURN_AROUND:
        # if SENSOR_PLACEMENT == "FRONT":
        print("turning AROAUND")
        drivePid(0.4,  80)
        # time.sleep(0.4)
        DRIVER.drive(-80, 80)
        time.sleep(0.5)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        while not lineData[3]:
            DRIVER.drive(-80, 80)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        DRIVER.drive(0,0)
        SONIC_FILTER.reset(120)

        # posIndx += 1
        nextPos, nextState, nextHeading = currentPos, States.STRAIGHT, Heading.EAST

    prevReadings = sensorsByBooleans

    DRIVER.drive(leftSpeed, rightSpeed)

    if now.available():
        mac, rev = now.recv(5)
        print("GOT", rev.decode("utf-8"), len(rev.decode("utf-8")), "isCommand", rev.decode().startswith(DASH_DATA_PREFIX))
        # print(rev[1].decode("utf-8"))
        # START_ROBOT = True


#         # ints = int(rev[1].decode("utf-8").split(":")[1]) + 1
    # time.sleep_ms(20)