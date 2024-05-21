
try:
    import uheapq as heapq
except ImportError:
    import heapq

from statemachine import States

class Heading:
    __reversed = False

    NORTH = "N"
    SOUTH = "S"
    WEST = "W"
    EAST = "E"

    def __init__(self, reversed: bool = False):
        self.reverse(reversed)

    def reverse(self, state: bool):
        self.__reversed = state
        self.NORTH = "N" if not state else "S"
        self.SOUTH = "S" if not state else "N"
        self.WEST = "W" if not state else "E"
        self.EAST = "E" if not state else "W"

class Length:
    x = 1
    x12 = 1 / 2
    x13 = 1 / 3
    x16 = 1 / 4

class BoxPlaces:
    RED = "Y"
    BLUE = "AA"
    BLACK = "X"
    GREEN = "Z"

    A = "A"
    B = "B"
    C = "C"
    D = "D"

#   N
# W   E
#   S

HeadingsToState = {
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

nodes: dict[str, dict[str, list[float, str]]] = {
    "A": {
        "E": [Length.x13, Heading.SOUTH]
    },
    "B": {
        "F": [Length.x13, Heading.SOUTH]
    },
    "C": {
        "G": [Length.x13, Heading.SOUTH]
    },
    "D": {
        "H": [Length.x13, Heading.SOUTH]
    },
    "E": {
        "F": [Length.x16, Heading.EAST],
        "M": [Length.x12, Heading.SOUTH],
        "A": [Length.x13, Heading.NORTH]
    },
    "F": {
        "B": [Length.x13, Heading.NORTH],
        "E": [Length.x16, Heading.WEST],
        "G": [Length.x16, Heading.EAST]
    },
    "G": {
        "F": [Length.x16, Heading.WEST],
        "C": [Length.x13, Heading.NORTH],
        "H": [Length.x16, Heading.EAST]
    },
    "H": {
        "D": [Length.x13, Heading.NORTH],
        "G": [Length.x16, Heading.WEST],
        "I": [Length.x13, Heading.EAST]
    },
    "I": {
        "H": [Length.x13, Heading.WEST],
        "K": [Length.x13, Heading.SOUTH],
        "J": [Length.x, Heading.EAST]
    },
    "J": {
        "I": [Length.x, Heading.WEST],
        "L": [Length.x13, Heading.SOUTH]
    },
    "K": {
        "I": [Length.x13, Heading.NORTH],
        "L": [Length.x, Heading.EAST],
        "N": [Length.x16, Heading.SOUTH]
    },
    "L": {
        "J": [Length.x13, Heading.NORTH],
        "K": [Length.x, Heading.WEST],
        "O": [Length.x16, Heading.SOUTH]
    },
    "M": {
        "E": [Length.x12, Heading.NORTH],
        "N": [Length.x, Heading.EAST],
        "P": [Length.x16, Heading.SOUTH]
    },
    "N": {
        "K": [Length.x16, Heading.NORTH],
        "M": [Length.x, Heading.WEST],
        "Q": [Length.x16, Heading.SOUTH],
        "O": [Length.x, Heading.EAST]
    },
    "O": {
        "L": [Length.x16, Heading.NORTH],
        "N": [Length.x, Heading.WEST],
        "W": [Length.x12, Heading.SOUTH]
    },
    "P": {
        "M": [Length.x16, Heading.NORTH],
        "Q": [Length.x, Heading.EAST],
        "R": [Length.x13, Heading.SOUTH]
    },
    "Q": {
        "N": [Length.x16, Heading.NORTH],
        "P": [Length.x, Heading.WEST],
        "S": [Length.x13, Heading.SOUTH]
    },
    "R": {
        "P": [Length.x13, Heading.NORTH],
        "S": [Length.x, Heading.EAST]
    },
    "S": {
        "R": [Length.x, Heading.WEST],
        "Q": [Length.x13, Heading.NORTH],
        "T": [Length.x13, Heading.EAST]
    },
    "T": {
        "S": [Length.x13, Heading.WEST],
        "X": [Length.x13, Heading.SOUTH],
        "U": [Length.x16, Heading.EAST]
    },
    "U": {
        "T": [Length.x16, Heading.WEST],
        "Y": [Length.x13, Heading.SOUTH],
        "V": [Length.x16, Heading.EAST]
    },
    "V": {
        "U": [Length.x16, Heading.WEST],
        "Z": [Length.x13, Heading.SOUTH],
        "W": [Length.x16, Heading.EAST]
    },
    "W": {
        "AA": [Length.x13, Heading.SOUTH],
        "O": [Length.x12, Heading.NORTH],
        "V": [Length.x16, Heading.WEST]
    },
    "X": {
        "T": [Length.x13, Heading.NORTH]
    },
    "Y": {
        "U": [Length.x13, Heading.NORTH]
    },
    "Z": {
        "V": [Length.x13, Heading.NORTH]
    },
    "AA": {
        "W": [Length.x13, Heading.NORTH]
    },
}
XHeightY = 0
AHeightY = 1 + (2/3)
EHeightY = 1 + (1/3)
MHeightY = (1/2) + (1/3)
PHeightY = 2/3
RHeightY = 1/3
KHeightY = (1/2) + (2/3)

RVerticalX = 0
FVerticalX = 1/6
GVerticalX = 2/6
HVerticalX = 3/6
SVerticalX = 1
XVerticalX = 1 + (1/3)
YVerticalX = 1 + (1/3) + (1/6)
ZVerticalX = 1 + (1/3) + (2/6)
AAVerticalX = 1 + (1/3) + (3/6)

plottingData = {
    "A": (RVerticalX, AHeightY),
    "B": (FVerticalX, AHeightY),
    "C": (GVerticalX, AHeightY),
    "D": (HVerticalX, AHeightY),
    "E": (RVerticalX, EHeightY),
    "F": (FVerticalX, EHeightY),
    "G": (GVerticalX, EHeightY),
    "H": (HVerticalX, EHeightY),
    "I": (SVerticalX, EHeightY),
    "J": (AAVerticalX, EHeightY),
    "K": (SVerticalX, KHeightY),
    "L": (AAVerticalX, KHeightY),
    "M": (RVerticalX, MHeightY),
    "N": (SVerticalX, MHeightY),
    "O": (AAVerticalX, MHeightY),
    "P": (RVerticalX, PHeightY),
    "Q": (SVerticalX, PHeightY),

    "R": (RVerticalX, RHeightY),
    "S": (SVerticalX, RHeightY),
    "T": (XVerticalX, RHeightY),
    "U": (YVerticalX, RHeightY),
    "V": (ZVerticalX, RHeightY),
    "W": (AAVerticalX, RHeightY),

    "X": (XVerticalX, XHeightY),
    "Y": (YVerticalX, XHeightY),
    "Z": (ZVerticalX, XHeightY),
    "AA": (AAVerticalX, XHeightY)
}

class Graph:

    blockWeight = 1e25
    def __init__(self, nodes):
        self.nodes = nodes

    def getBlocks(self) -> list[str]:
        blockedNodes = []
        for node in self.nodes:
            for neighbor, (weight, _) in self.nodes[node].items():
                if weight >= self.blockWeight:
                    t1, t2 = f"{node}-{neighbor}", f"{neighbor}-{node}"
                    if t1 not in blockedNodes or t2 not in blockedNodes:
                        blockedNodes.append(t1)

        return blockedNodes

    def setBlocks(self, blocks: str | list[str]):
        if isinstance(blocks, str):
            blocks = [blocks]

        for block in blocks:
            A,B = block.split("-")
            self.nodes[A][B] = self.blockWeight
            self.nodes[B][A] = self.blockWeight

    def dijkstra(self, start, end):
        # Initialize distances and paths to all nodes
        distances = {node: float('inf') for node in self.nodes}
        distances[start] = 0
        paths = {node: [] for node in self.nodes}

        # Initialize priority queue with start node
        pq = [(0, start)]

        while pq:
            current_distance, current_node = heapq.heappop(pq)

            # If we have already found a shorter path to this node, skip it
            if current_distance > distances[current_node]:
                continue

            # Explore neighbors of the current node
            for neighbor, (weight, _) in self.nodes[current_node].items():
                distance = current_distance + weight

                # If this new path is shorter than previously known, update the distance and path
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    paths[neighbor] = paths[current_node] + [current_node]
                    heapq.heappush(pq, (distance, neighbor))

        # Return the shortest distance and the path taken
        shortest_distance = distances[end]
        shortest_path = paths[end] + [end]

        direction_list: list[Heading] = []
        for x in range(0, len(shortest_path)-1):
            A = shortest_path[x]
            B = shortest_path[x+1]
            direction_list.append(nodes[A][B][1])

        return shortest_distance, shortest_path, direction_list

    def clearBlocks(self, nodes):
        self.nodes = nodes

# graph = Graph(nodes)

# Example usage: find the shortest distance between nodes "A" and "Z"
# start_node = "A"
# end_node = "Z"
# shortest_distance, shortest_path = graph.dijkstra(start_node, end_node)


# print(f"The shortest distance between {start_node} and {end_node} is {shortest_distance}.")
# print(f"The shortest path is: {' -> '.join(shortest_path)}")
# print(f"The Directions for the shortest path are: {' -> '.join(direction_list)}")
# #

#
# # Extract x and y coordinates
# x_coords = [coord[0] for coord in plottingData.values()]
# y_coords = [coord[1] for coord in plottingData.values()]
#
# # Plotting
# plt.figure(figsize=(8, 6))
#
# # Plot points
# plt.scatter(x_coords, y_coords, color='blue')
#
# # Annotate points
# for letter, (x, y) in plottingData.items():
#     plt.text(x+1/20, y+1/20, letter, fontsize=12, ha='center', va='center')
#
# xB, yB = [], []
# for node in shortest_path:
#     x, y = plottingData[node]
#     xB.append(x)
#     yB.append(y)
# # xB.append(plottingData["Z"][0])
# # yB.append(plottingData["Z"][1])
#
# plt.plot(xB, yB, marker='o', linestyle='-', color='red', label='Line ABCD')
#
# plt.xlabel('Horizontal Axis')
# plt.ylabel('Vertical Axis')
# plt.title('Plot of Points')
# plt.grid(True)
# plt.tight_layout()
#
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
#
# # Extract x and y coordinates
# x_coords = [coord[0] for coord in plottingData.values()]
# y_coords = [coord[1] for coord in plottingData.values()]
#
# # Plotting
# # fig, ax = plt.subplot()
# fig = plt.figure()
# # Plot points
# plt.scatter(x_coords, y_coords, color='blue')
# for letter, (x, y) in plottingData.items():
#     plt.text(x + 1/20, y + 1/20, letter, fontsize=12, ha='center', va='center')
#
# # Initialize an empty plot for the robot's path
# robot_path, = plt.plot(xB[0], yB[0], marker='o', linestyle='-', color='green')
#
# def update(frame):
#     robot_path.set_data(xB[:frame], yB[:frame])  # Update robot path
#     return robot_path,
#
# # Create animation
# ani = FuncAnimation(fig, update, frames=len(xB)+1, interval=100, blit=True, repeat=False)
#
# plt.grid(True)
# plt.show()
