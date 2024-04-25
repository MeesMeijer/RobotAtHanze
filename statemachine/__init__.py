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
    GO_TO_GOAL = "GO_TO_GOAL"
    AT_GOAL = "AT_GOAL"

    statesByBooleans = {
        (0, 0, 1, 0, 0):  STRAIGHT,      # (0, 0, 1, 0, 0)
        (0, 1, 0, 0, 0):  STRAIGHT,      # (0, 1, 0, 0, 0)
        (0, 0, 0, 1, 0):  STRAIGHT,      # (0, 0, 0, 1, 0)
        (0, 1, 1, 0, 0):  STRAIGHT,      # (0, 1, 1, 0, 0)
        (0, 0, 1, 1, 0):  STRAIGHT,      # (0, 0, 1, 1, 0)
        (0, 1, 1, 1, 0):  STRAIGHT,      # (0 ,1, 1, 1, 0)
        (0, 0, 0, 1, 1):   STRAIGHT,
        (1, 1, 0, 0, 0): STRAIGHT,  # (0, 0, 0, 1, 1)
        (0, 0, 1, 1, 1):  RIGHT_CORNER,      # (1, 1, 0, 0, 0)
        (1,  1,  1,  0, 0):  LEFT_CORNER,      # (1, 1, 1, 0, 0)
        (1,0,0,0,0): STRAIGHT,
        (0,0,0,0,1): STRAIGHT,
        # (0, 0, 1, 1, 1):   RIGHT_CORNER,      # (0, 0, 1, 1, 1)
        (0, 1, 1, 1, 1):   RIGHT_CORNER,  # (0, 1, 1, 1, 1)
        (1, 1, 1, 1, 0):  LEFT_CORNER,   # (1, 1, 1, 1, 0)
        # (1, 1, 1, 0, 0): LEFT_CORNER,
        (0, 0, 0, 0, 0):  STOP,           # (0, 0, 0, 0, 0),
        (1, 1, 1, 1, 1): T_CROSS
    }

    currentState = STOP

    def __init__(self, startState: str):
        self.currentState = startState

    def __str__(self):
        return f"{self.currentState}"

    def __getitem__(self, item ):
        # if type(item) is type(str()):
        state = self.statesByBooleans.get(tuple(item))
        if not state:
            return self.STOP
        return state

