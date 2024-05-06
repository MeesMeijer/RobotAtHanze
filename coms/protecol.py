from statemachine import States

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
