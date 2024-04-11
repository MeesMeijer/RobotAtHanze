import time

from . import DCMotor

class MotorDriver:
    _leftMotor: DCMotor
    _rightMotor: DCMotor

    __maxSpeed = 100.0  # the max % the motors can spin.
    __kickstartThreshold = 50.0  # Every speed value below this value needs a short 'kickstart' to start turning the motors
    __kickStartTime = 10  # the time for the motor to start spinning in MS

    def __init__(self, leftMotor: DCMotor, rightMotor: DCMotor):
        self._leftMotor = leftMotor
        self._rightMotor = rightMotor

        self._leftMotor.setState(DCMotor.STOP)
        self._leftMotor.setSpeed(0)

        self._rightMotor.setState(DCMotor.STOP)
        self._rightMotor.setSpeed(0)

    def drive(self, leftSpeed: float, rightSpeed: float) -> None:
        """ leftSpeed: 'float' 0-100% motor power
            rightSpeed: 'float' 0-100% motor power """

        leftState = self.__getState(leftSpeed)
        rightState = self.__getState(rightSpeed)

        # if int(leftSpeed) != 0 and int(rightSpeed) != 0:
        leftSpeed, rightSpeed = self.__checkSaturation(abs(leftSpeed), abs(rightSpeed))

        self._leftMotor.setState(leftState)
        # TODO: Check of this works...
        if self.__kickstartThreshold > leftSpeed > 0:
            self._leftMotor.setSpeed(70)
            time.sleep_ms(self.__kickStartTime)
        self._leftMotor.setSpeed(leftSpeed)

        self._rightMotor.setState(rightState)
        #TODO: Same for here..
        if self.__kickstartThreshold > rightSpeed > 0:
            self._rightMotor.setSpeed(70)
            time.sleep_ms(self.__kickStartTime)
        self._rightMotor.setSpeed(rightSpeed)

    def __sign(self, x: float) -> int:
        if x < 0: return -1
        elif int(x) == 0: return 0
        elif x > 0: return 1

    def __getState(self, speed: float ) -> tuple[int, int]:
        if int(speed) == 0: return DCMotor.STOP
        if speed < 0: return DCMotor.BACKWARDS
        return DCMotor.FORWARDS

    def __checkSaturation(self, leftSpeed: float, rightSpeed: float) -> tuple[float, float]:
        """ Checks and corrects with ratio if the desired left/right speed is over 100% """
        if leftSpeed > self.__maxSpeed or rightSpeed > self.__maxSpeed:
            ratio = rightSpeed / leftSpeed
            if ratio > 1.0:
                rightSpeed = self.__sign(rightSpeed) * self.__maxSpeed
                leftSpeed = self.__sign(leftSpeed) * self.__maxSpeed / ratio
                return leftSpeed, rightSpeed

            rightSpeed = self.__sign(rightSpeed) * self.__maxSpeed * ratio
            leftSpeed = self.__sign(leftSpeed) * self.__maxSpeed
            return leftSpeed, rightSpeed

        return leftSpeed, rightSpeed
