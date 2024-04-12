import heapq


class Heading:
    NORTH = "N"
    SOUTH = "S"
    WEST = "W"
    EAST = "E"


class Length:
    x = 1
    x12 = 1 / 2
    x13 = 1 / 3
    x16 = 1 / 6


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
        "M": [Length.x12, Heading.NORTH],
        "A": [Length.x13, Heading.SOUTH]
    },
    "F": {
        "B": [Length.x13, Heading.SOUTH],
        "E": [Length.x16, Heading.WEST],
        "G": [Length.x16, Heading.EAST]
    },
    "G": {
        "F": [Length.x16, Heading.WEST],
        "C": [Length.x13, Heading.SOUTH],
        "H": [Length.x16, Heading.EAST]
    },
    "H": {
        "D": [Length.x13, Heading.SOUTH],
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
        "E": [Length.x12, Heading.SOUTH],
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
XHeightY = 1/3
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
    "B": (RVerticalX, AHeightY),
    "C": (RVerticalX, AHeightY),
    "D": (RVerticalX, AHeightY),
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
    "R": (RVerticalX, RVerticalX),
    "S": (SVerticalX, RVerticalX),
    "T": (XVerticalX, RVerticalX),
    "U": (YVerticalX, RVerticalX),
    "V": (ZVerticalX, RVerticalX),
    "W": (AAVerticalX, RVerticalX),
    "X": (XVerticalX, XHeightY),
    "Y": (YVerticalX, XHeightY),
    "Z": (ZVerticalX, XHeightY),
    "AA": (AAVerticalX, XHeightY)
}

class Graph:
    def __init__(self, nodes):
        self.nodes = nodes

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
        return shortest_distance, shortest_path

    def BlockConnection(self, A: str, B: str):
        self.nodes[A][B][0] = 1e25
        self.nodes[B][A][0] = 1e25

graph = Graph(nodes)

graph.BlockConnection("K", "N")
graph.BlockConnection("N", "Q")
graph.BlockConnection("J", "L")
graph.BlockConnection("O", "W")
graph.BlockConnection("E", "M")
graph.BlockConnection("R", "S")

# Example usage: find the shortest distance between nodes "A" and "Z"
start_node = "A"
end_node = "Z"
shortest_distance, shortest_path = graph.dijkstra(start_node, end_node)
print(f"The shortest distance between {start_node} and {end_node} is {shortest_distance}.")
print(f"The shortest path is: {' -> '.join(shortest_path)}")

# for col, neightbs in nodes.items():
#     for i, (distance, direction) in neightbs.items():
#         print(f"{col}-{i} = {distance} [{direction}]")
import matplotlib.pyplot as plt

#
