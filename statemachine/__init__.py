class States:
    STRAIGHT = "STRAIGHT"
    LEFT_CORNER = "LEFT_CORNER"
    RIGHT_CORNER = "RIGHT_CORNER"
    CROSS_POINT = "CROSS_POINT"
    T_CROSS = "T_CROSS"
    OBSTACLE = "OBSTACLE"
    STOP = "STOP"
    TURN_AROUND = "TURN_AROUND"
    PICK_UP_BOX = "PICK_UP_BOX"

    statesByBooleans = {
        (False, False, True,  False, False):  STRAIGHT,      # (0, 0, 1, 0, 0)
        (False, True,  False, False, False):  STRAIGHT,      # (0, 1, 0, 0, 0)
        (False, False, False, True,  False):  STRAIGHT,      # (0, 0, 0, 1, 0)
        (False, True,  True,  False, False):  STRAIGHT,      # (0, 1, 1, 0, 0)
        (False, False, True,  True,  False):  STRAIGHT,      # (0, 0, 1, 1, 0)
        (False, True,  True,  True,  False):  STRAIGHT,      # (0 ,1, 1, 1, 0)
        (False, False, False, True,  True):   STRAIGHT,      # (0, 0, 0, 1, 1)
        (True,  True,  False, False, False):  STRAIGHT,      # (1, 1, 0, 0, 0)
        (True,  True,  True,  False, False):  STRAIGHT,      # (1, 1, 1, 0, 0)
        (False, False, True,  True,  True):   STRAIGHT,      # (0, 0, 1, 1, 1)
        (False, True,  True,  True,  True):   RIGHT_CORNER,  # (0, 1, 1, 1, 1)
        (True,  True,  True,  True,  False):  LEFT_CORNER,   # (1, 1, 1, 1, 0)
        (False, False, False, False, False):  STOP           # (0, 0, 0, 0, 0)
    }

    currentState = STOP

    def __init__(self, startState: str):
        self.currentState = startState

    def __str__(self):
        return f"{self.currentState}"

    def __getitem__(self, item: list[bool]):
        if type(item) is type(str()):
            return self.statesByBooleans.get(tuple(item), __default=self.STOP)
