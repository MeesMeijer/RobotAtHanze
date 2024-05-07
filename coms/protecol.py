from statemachine import States

# Sensors update


# Mapping: Update
#   Start pos and End pos
#   Output of the ddijkstra algs
#   Current pos, heading
#   desired pos, Heading,
#

# Statemachine Update
#   Current State
#   Next State
#   Pid output




class Update:
    pass


class SensorUpdate:
    IR_SENSOR_DATA: list[int, int, int, int, int]
    IR_SENSOR_PLACEMENT: str

    DISTANCE_SENSOR_RAW: int
    DISTANCE_SENSOR_FILTERD: int

    MAGNET_STATUS: bool

    LEFT_MOTOR_STATE: tuple[int, int]
    RIGHT_MOTOR_STATE: tuple[int,int]
    LEFT_MOTOR_SPEED: float
    RIGHT_MOTOR_SPEED: float

    COLOR_SENSOR_RAW: list[int,int,int]
    COLOR_DETECTED: str | None

    I2C_SCAN_RESULTS: list[list, list]

    def get(self):
        return

#   IR line sensor and SENSOR_PLACEMENT
#   distance sensor filterd and non filterd
#   Magnet status
#   left/right motor status
#   Color sensor
#   Scan of the I2C modules




class MappingUpdate:
    pass


class StatemachineUpdate:
    pass


class Response:
    pass


class DecodedData:
    startPos: str  # max len 2 "A..AA"
    endPos: str  # max len 2 "A..AA"
    currentPos: str  # max len 2 "A..AA"

    currentState: str  # one of the States. max len 12
    nextState: str # one of the States. max len 12




class EncodedData:
    pass


class Protecol:
    def __init__(self, data):
        self.data = data

    def encode(self):
        pass

    def decode(self):
        pass
