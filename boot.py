# def do_connect(ssid, pwd)
##  import network
#
##  sta_if = network.WLAN(network.STA_IF)
##  if not sta_if.isconnected()
## #  print('connecting to network...')
## #  sta_if.active(True)
## #  sta_if.connect(ssid, pwd)
## #  while not sta_if.isconnected()
## # #  pass
##  print('network config', sta_if.ifconfig())
#
# # This file is executed on every boot (including wake-boot from deepsleep)
# #import esp
# #esp.osdebug(None)
# from pins import SSID, PASSWORD
# # Attempt to connect to WiFi network
# do_connect(SSID, PASSWORD)
#
# import webrepl
# webrepl.start()
import time

from mapping import Graph, nodes, Heading, BoxPlaces
from statemachine import States

events = [
    States.STRAIGHT,
    States.STRAIGHT,

    States.LEFT_CORNER,

    States.STRAIGHT,

    States.T_CROSS,

    States.STRAIGHT,

    States.LEFT_CORNER,

    States.STRAIGHT,
    States.STRAIGHT,
]


dictHeadings = {
    Heading.EAST: {
        Heading.NORTH: States.LEFT_CORNER,
        Heading.SOUTH: States.RIGHT_CORNER,
        Heading.WEST: States.TURN_AROUND,
        Heading.EAST: States.STRAIGHT
    },
    Heading.NORTH: {
        Heading.NORTH: States.STRAIGHT,
        Heading.SOUTH: States.TURN_AROUND,
        Heading.WEST: States.LEFT_CORNER,
        Heading.EAST: States.RIGHT_CORNER
    },
    Heading.WEST: {
        Heading.NORTH: States.RIGHT_CORNER,
        Heading.SOUTH: States.LEFT_CORNER,
        Heading.WEST: States.STRAIGHT,
        Heading.EAST: States.TURN_AROUND
    },
    Heading.SOUTH: {
        Heading.NORTH: States.TURN_AROUND,
        Heading.SOUTH: States.STRAIGHT,
        Heading.WEST: States.RIGHT_CORNER,
        Heading.EAST: States.LEFT_CORNER
    }
}


graph = Graph(nodes)

startingPoint = "M"
endPoint = "AA"
goingBack = True


graph.BlockConnection("M", "P")
graph.BlockConnection("N", "Q")
cost, shortestPath, pathDirections = graph.dijkstra(startingPoint, endPoint)

# t = {}
# for i in range(0, len(shortestPath)-1):
#     t[f"{shortestPath[i]}:{shortestPath[i + 1]}"] = pathDirections[i]

# print(t)
posIndx = 0
if startingPoint == shortestPath[0]: posIndx +=1
currentPos = shortestPath[posIndx]
currentHeading = nodes[shortestPath[posIndx]][shortestPath[posIndx+1]][1]

nextPos, nextHeading, nextState = None, None, None

for event in events:

    if nextState is not None:
        currentState, currentPos = nextState, nextPos

        currentHeading = nextHeading

        nextPos, nextHeading, nextState = None, None, None
    else:
        currentState = event

    if currentState == States.STOP:
        break;

    elif currentState == States.STRAIGHT:
        # print("PID GOING STRAIGHT")
        if posIndx == len(shortestPath)-1:
            print("OP BESTEMMING.. "+ currentPos)
            nextPos, nextHeading, nextState = currentPos, currentHeading, States.STOP
        else:
            nextPos, nextHeading, nextState = currentPos, currentHeading, None

    elif currentState == States.LEFT_CORNER:
        print("AT LEFT CORNER, WHAT TO DO? ")

        if posIndx == len(shortestPath):
            print("OP BESTEMMING.. "+ currentPos)
            nextPos, nextHeading, nextState = currentPos, currentHeading, States.STOP
        else:

            desiredPos = shortestPath[posIndx + 1]
            desiredHeading = nodes[currentPos][desiredPos][1]
            desiredState = dictHeadings[currentHeading][desiredHeading]
            posIndx += 1

            print(f"Desired Pos: {desiredPos} Heading: {desiredHeading} State: {desiredState}")

            if currentState != desiredState:
                print(f"State is: {currentState} But iam going: {desiredState}")
                nextState = desiredState
            else:
                print("Turing Left")

            nextPos, nextHeading = desiredPos, desiredHeading

    elif currentState == States.RIGHT_CORNER:
        print("AT RIGHT CORNER, WHAT TO DO? ")

        if posIndx == len(shortestPath)-1:
            print("OP BESTEMMING.. "+ currentPos)
            nextPos, nextHeading, nextState = currentPos, currentHeading, States.STOP
        else:

            desiredPos = shortestPath[posIndx + 1]
            desiredHeading = nodes[currentPos][desiredPos][1]
            desiredState = dictHeadings[currentHeading][desiredHeading]
            posIndx += 1
            print(f"Desired Pos: {desiredPos} Heading: {desiredHeading} State: {desiredState}")

            if currentState != desiredState:
                print(f"State is: {currentState} But iam going: {desiredState}")
                nextState = desiredState
            else:
                print("Turing Right")

            nextPos, nextHeading = desiredPos, desiredHeading

    elif currentState == States.T_CROSS:
        print("AT T-CROSS, WHAT TO DO? ")

        if posIndx == len(shortestPath)-1:
            print("OP BESTEMMING.. "+ currentPos)
            nextPos, nextHeading, nextState = currentPos, currentHeading, States.STOP

        else:

            desiredPos = shortestPath[posIndx + 1]
            desiredHeading = nodes[currentPos][desiredPos][1]
            desiredState = dictHeadings[currentHeading][desiredHeading]
            posIndx += 1
            print(f"Desired Pos: {desiredPos} Heading: {desiredHeading} State: {desiredState}")

            nextPos, nextHeading, nextState = desiredPos, desiredHeading, desiredState
            print(f"State is: {currentState} But iam going: {desiredState}")

    # print(ts)
    print(f"Present: Going to: {currentPos} Heading: {currentHeading} State: {currentState}\nNext:    Going To: {nextPos} Heading: {nextHeading} State: {nextState}\n")
    # time.sleep(0.2)
