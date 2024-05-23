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

I2CB = machine.I2C(1, sda=Pin(SDA2), scl=Pin(SCL2))
I2CA = machine.I2C(0, sda=Pin(SDA), scl=Pin(SCL))

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
filter_for_distance = MedianFilter(window_size=20)

# a com protecol using esp-now
now = Coms(peer=b'\xc8\xf0\x9e\xf2X\xe8')


def isCommand(msg: str) -> bool:
    return msg.startswith(DASH_DATA_PREFIX)


def onCommand(cmd:str):
    # struct: prefix,SET/GET,WHAT,DATA
    if not isCommand(cmd):
        return None


def drive_for(timout:float, speed: int):
    pid_backwards.reset()

    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < (timout*1000):
        [A1, A2, A3, A4, A5] = LINE.read()
        #TODO: Check if works when going backwards..
        inp = (-2 * A1 - 1.5 * A2 + 0 * A3 + 1.5 * A4 + 2 * A5) / 80
        apt = pid_backwards.calc(inp)
        DRIVER.drive(speed-apt, speed+apt)
        time.sleep_ms(10)

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
        now.send(f"{ROBOT_DATA_PREFIX},I2C_SCAN,{[devicesA, devicesB]}")


getState = States(States.STOP)

temp = {
    "A": "E",
    "B": "F",
    "C": "G",
    "D": "H",

    "AA": "W",
    "X": "T",
    "Y": "U",
    "Z": "V",
    "Blue": BoxPlaces.BLUE,
    "Green": BoxPlaces.GREEN,
    "Red": BoxPlaces.RED,
    "Black": BoxPlaces.BLACK
}


boxesToPick = [
    "D", "C", "B", "A"
]
force = False
graph = Graph(nodes)

startCel = ["J", "I"]
endpoint = boxesToPick.pop()

cost, shortestPath, pathDirections = graph.dijkstra(startCel[0], endpoint)
print(shortestPath, pathDirections)

now.send(f"CURRENT_PATH,{shortestPath},BLOCKS,{graph.getBlocks()}")

hasBox = False
isBoxTurn = False
posIndx = 0
currentPos = startCel[0]
currentHeading = nodes[startCel[0]][startCel[1]][1]
nextPos, nextHeading, nextState = None, None, None

startState = HeadingsToState[currentHeading][ nodes[shortestPath[0]][shortestPath[1]][1]]

# print(currentPos, currentHeading, shortestPath[1], nodes[shortestPath[0]][shortestPath[1]][1], )
START_ROBOT = True
# MAGNET.value(1)

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
    filter_for_distance.update(raw := SONIC.distance_cm())
    distance = filter_for_distance.get_median()

    # UPDATE currentState with the IR sensor data
    sensorState = getState[sensorsByBooleans]
    if nextState is not None:
        currentState, currentPos, currentHeading = nextState, nextPos, nextHeading
        nextPos, nextHeading, nextState = None, None, None
    else:
        if nextPos:
            currentPos = nextPos

        if nextHeading:
            currentHeading = nextHeading

        currentState = sensorState
    # currentState = States.STOP

    if startState:
        currentState = startState
        # force = True
        startState = None

    if not START_ROBOT:
        currentState = States.STOP

    # if distance < 15:
    #     currentState = States.OBSTACLE
    #     filter_for_distance.reset(200)

    if posIndx < len(shortestPath) - 1:
        # print(f"{currentPos} to {shortestPath[posIndx +1]}")
        now.send(f"{currentState},{currentPos},{shortestPath[posIndx+1]},{nodes[currentPos][shortestPath[posIndx+1]][1]},{currentHeading},{lineSensorData},{distance},{raw}")
        print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx +1], "), Head:", nodes[currentPos][shortestPath[posIndx+1]][1], "->", currentHeading, f" ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")
    else:
        # print(f"{currentPos} to {shortestPath[posIndx]}")
        now.send(f"{currentState},{currentPos},{shortestPath[posIndx]},{pathDirections[-1]},{currentHeading},{lineSensorData},{distance},{raw}")
        print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx], "), Head:", pathDirections[-1], "->", currentHeading, f" ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")

    # Statemachine
    if currentState == States.STOP:
        leftSpeed = 0
        rightSpeed = 0

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
            output = pid_forwards.calc(MovingAverage / 70)

            x = 60 if not hasBox else 80
            leftSpeed = x - output
            rightSpeed = x + output

            if posIndx < len(shortestPath) - 1:
                desiredPos = shortestPath[posIndx + 1]
            else:
                desiredPos = shortestPath[posIndx]


            isBoxPickUpState, type = checkIfBoxEndpoint(shortestPath[posIndx], shortestPath[-1])
            isBoxTurn = isBoxPickUpState

            desiredState = None
            if isBoxPickUpState and type == 'PICK_BOX':
                desiredState = States.PICK_UP_BOX
            elif isBoxPickUpState and type == "RELEASE_BOX":
                desiredState = States.RELEASE_BOX

            nextPos, nextHeading, nextState = currentPos, currentHeading, desiredState

        fromTCross = False

    elif currentState in [States.LEFT_CORNER, States.RIGHT_CORNER, States.T_CROSS]:

        if posIndx == len(shortestPath) -1:
            print("OP BESTEMMING CORNER.. " + shortestPath[posIndx])
            nextPos, nextHeading, nextState = currentPos, currentHeading, States.AT_GOAL
        else:
            # TODO: Check if works with the new setup
            if time.ticks_diff(time.ticks_ms(), timeToIntersection) < 0.4 * 1000:
                print("this is duplicate", time.ticks_diff(time.ticks_ms(), timeToIntersection))
                IntersectionButStraight = True
                nextPos, nextHeading, nextState = currentPos, currentHeading, States.STRAIGHT
            else:
                posIndx += 1

                if posIndx < len(shortestPath) - 1:
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

                timeToIntersection = time.ticks_ms() if not force else None

                if desiredState == States.PICK_UP_BOX:
                    nextState = States.PICK_UP_BOX
                elif desiredState == States.RELEASE_BOX:
                    nextState = States.RELEASE_BOX

                elif desiredState == States.LEFT_CORNER:
                    print("turning left ")
                    if not force:
                        DRIVER.drive(45,45)
                        time.sleep(0.45 if isBoxTurn else 0.3)
                        isBoxTurn = False

                    DRIVER.drive(-50, 90)
                    time.sleep(0.5)

                    lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    while not lineData[1]:
                        DRIVER.drive(-65, 73)
                        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    filter_for_distance.reset(120)
                    nextState = States.STRAIGHT

                elif desiredState == States.RIGHT_CORNER:
                    print("turning right ")

                    if not force:
                        DRIVER.drive(45,45)
                        time.sleep(0.45 if isBoxTurn else 0.3)
                        isBoxTurn = False

                    DRIVER.drive(90, -60)
                    time.sleep(0.5)

                    lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    while not lineData[3]:
                        DRIVER.drive(73, -75)
                        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
                    filter_for_distance.reset(120)
                    nextState = States.STRAIGHT

                elif desiredState == States.STRAIGHT:
                    nextState = States.STRAIGHT

                force = False
                nextPos, nextHeading = shortestPath[posIndx], desiredHeading

    elif currentState == States.RELEASE_BOX:
        print("Releasing the box")

        drive_for(0.4, 45)

        MAGNET.value(0)

        startCel = [endpoint, temp[endpoint]]
        endpoint = boxesToPick.pop()

        cost, shortestPath, pathDirections = graph.dijkstra(startCel[0], endpoint)
        now.send(f"CURRENT_PATH,{shortestPath},BLOCKS,{graph.getBlocks()}")

        posIndx = 0
        currentPos = startCel[0]
        currentHeading = nodes[startCel[1]][startCel[0]][1]

        desiredPos = shortestPath[posIndx + 1]
        desiredHeading = nodes[shortestPath[1]][shortestPath[2]][1]
        desiredState = HeadingsToState[currentHeading][desiredHeading]

        print(shortestPath, desiredState, f"{currentHeading}:{desiredHeading}", f"{currentPos} -> {desiredPos}")
        DRIVER.drive(-70,-70)
        time.sleep(0.1)

        if desiredState == States.RIGHT_CORNER:
            DRIVER.drive(30, -80)
        elif desiredState == States.LEFT_CORNER:
            DRIVER.drive(-80, 35)
        elif desiredState == States.TURN_AROUND:
            DRIVER.drive(-60,60)

        time.sleep(0.30)

        DRIVER.drive(-70, -70)
        time.sleep(0.1)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        num = 3
        if desiredState in [States.LEFT_CORNER, States.TURN_AROUND]:
            num = 1

        while not lineData[num]:
            if desiredState == States.RIGHT_CORNER:
                DRIVER.drive(0,-80)
            elif desiredState == States.TURN_AROUND or nextState == States.LEFT_CORNER:
                DRIVER.drive(-80, 10)

            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        DRIVER.drive(0,0)

        currentPos = desiredPos
        currentHeading = desiredHeading

        # print("After turn: ", currentPos, currentHeading)

        posIndx += 1
        desiredPos = shortestPath[posIndx+1]
        desiredHeading = nodes[currentPos][desiredPos][1]
        desiredState = HeadingsToState[currentHeading][desiredHeading]

        nextPos, nextState, nextHeading = shortestPath[posIndx], desiredState, desiredHeading
        print(f"pos: {currentPos}->{desiredPos}, head: {currentHeading}->{desiredHeading}")
        force = True
        hasBox = False

        # print(nextPos, nextHeading, nextState)

    elif currentState == States.PICK_UP_BOX:
        print("Pick the box")

        drive_for(0.4, 45)

        MAGNET.value(1)

        print((color := TCS.detectColor()))
        print(TCS.rgb())

        startCel = [endpoint, temp[endpoint]]
        endpoint = temp[str(color)]

        cost, shortestPath, pathDirections = graph.dijkstra(startCel[0], endpoint)
        now.send(f"CURRENT_PATH,{shortestPath}")

        posIndx = 0
        currentPos = startCel[0]
        currentHeading = nodes[startCel[1]][startCel[0]][1]

        desiredPos = shortestPath[posIndx + 1]
        desiredHeading = nodes[shortestPath[1]][shortestPath[2]][1]
        desiredState = HeadingsToState[currentHeading][desiredHeading]

        print(shortestPath, desiredState, f"{currentHeading}:{desiredHeading}", f"{currentPos} -> {desiredPos}")

        DRIVER.drive(-70,-70)
        time.sleep(0.2)

        if desiredState == States.RIGHT_CORNER:
            DRIVER.drive(30, -80)
        elif desiredState == States.LEFT_CORNER:
            DRIVER.drive(-80, 30)
        elif desiredState == States.TURN_AROUND:
            DRIVER.drive(-60,60)

        time.sleep(0.55)

        DRIVER.drive(-70, -70)
        time.sleep(0.1)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        num = 3
        if desiredState in [States.LEFT_CORNER, States.TURN_AROUND]:
            num = 1

        while not lineData[num]:
            if desiredState == States.RIGHT_CORNER:
                DRIVER.drive(0,-80)
            elif desiredState == States.TURN_AROUND or nextState == States.LEFT_CORNER:
                DRIVER.drive(-80, 10)

            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        DRIVER.drive(0,0)

        currentPos = desiredPos
        currentHeading = desiredHeading

        # print("After turn: ", currentPos, currentHeading)

        posIndx += 1
        desiredPos = shortestPath[posIndx+1]
        desiredHeading = nodes[currentPos][desiredPos][1]
        desiredState = HeadingsToState[currentHeading][desiredHeading]

        nextPos, nextState, nextHeading = shortestPath[posIndx], desiredState, desiredHeading

        print(f"pos: {currentPos}->{desiredPos}, head: {currentHeading}->{desiredHeading}")
        force = True
        hasBox = True

        timeToIntersection = time.ticks_ms()
        print(nextPos, nextHeading, nextState)
        # raise Exception("te")

    elif currentState == States.OBSTACLE:

        if posIndx < len(shortestPath)-1:
            A, B = currentPos, shortestPath[posIndx +1]
            headingAB = nodes[currentPos][shortestPath[posIndx+1]][1]
            # print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx +1], "), Head:", , "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")
        else:
            A, B = currentPos, shortestPath[posIndx]
            headingAB = pathDirections[-1]
            # print("State: ", currentState, ", (", currentPos, ":", shortestPath[posIndx], "), Head:", , "->", currentHeading, " ", [int(x) for x in sensorsByBooleans], " ", lineSensorData, sep="")
        graph.setBlocks(f"{A}-{B}")
        print(f"detected a obstacle between {A}:{B}")

        cost, path, dirs = graph.dijkstra(A, endpoint)

        print("TURN AROUND")

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        while not lineData[1]:
            DRIVER.drive(-85, 80)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        DRIVER.drive(0,0)

        currentPos = B
        currentHeading = nodes[B][A][1]

        print(f"New path: {path}, old path: {shortestPath}")

        posIndx = 0
        desiredPos = path[posIndx + 1]
        desiredHeading = nodes[path[0]][path[1]][1]
        desiredState = HeadingsToState[currentHeading][desiredHeading]

        print(f"nextpos: {path[0]} next intersection: {desiredState}")
        print(shortestPath, desiredState, f"{currentHeading}:{desiredHeading}", f"{currentPos} -> {desiredPos}")

        shortestPath = path
        nextPos, nextState, nextHeading = shortestPath[posIndx], desiredState, desiredHeading
        force = True

    elif currentState == States.TURN_AROUND:
        # if SENSOR_PLACEMENT == "FRONT":
        print("turning AROAUND")
        drive_for(0.4,  80)
        # time.sleep(0.4)
        DRIVER.drive(-80, 80)
        time.sleep(0.5)

        lineData = LINE.read().toBooleans(WHITE_THRESHOLD)
        while not lineData[3]:
            DRIVER.drive(-80, 80)
            lineData = LINE.read().toBooleans(WHITE_THRESHOLD)

        DRIVER.drive(0,0)
        filter_for_distance.reset(120)

        # posIndx += 1
        nextPos, nextState, nextHeading = currentPos, States.STRAIGHT, Heading.EAST

    DRIVER.drive(leftSpeed, rightSpeed)

    if now.available():
        mac, rev = now.recv(5)
        print("GOT", rev.decode("utf-8"), len(rev.decode("utf-8")), "isCommand", rev.decode().startswith(DASH_DATA_PREFIX))
