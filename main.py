from motors import DCMotor
import machine
import time
from machine import Pin, PWM
from MCP23017 import MCP23017
from HSR04 import HCSR04
from MPU6050 import MPU6050
from AS5600 import AS5600, ASMethods
from TCS3200 import TCS3200
from pins import *
from IRlineSensor import LineSensor
from asyncws import AsyncWebsocketClient
import gc
from random import randint
from PID import PID

frequency = 15000

I2CA = machine.I2C(0, sda=Pin(SDA), scl=Pin(SCL))
I2CB = machine.I2C(1, sda=Pin(SDA2), scl=Pin(SCL2))

MCP = MCP23017(I2CA, 0x20)
MPU = MPU6050(I2CA)

ASL = AS5600(I2CB, 0x36)
ASR = AS5600(I2CA, 0x36)

SONIC = HCSR04(echo_pin=4, trigger_pin=2)
LINE = LineSensor(Pin(A1, Pin.IN), Pin(A2, Pin.IN), Pin(A3, Pin.IN), Pin(A4, Pin.IN), Pin(A5, Pin.IN))

SWITCH = Pin(LIMIT_SWITCH, Pin.IN)

MPU.wake()
MCP.init()

MCP.pin(IN1, mode=0, value=0)  # in1
MCP.pin(IN2, mode=0, value=0)  # in2
MCP.pin(IN3, mode=0, value=0)  # in3
MCP.pin(IN4, mode=0, value=0)  # in4
MCP.pin(S0, mode=0, value=1)  # S0
MCP.pin(S1, mode=0, value=0)  # S1
MCP.pin(S2, mode=0, value=0)  # S2
MCP.pin(S3, mode=0, value=0)  # S3
MCP.pin(LED, mode=0, value=0)  # LED
MCP.pin(DIRA, mode=0, value=0)
MCP.pin(DIRB, mode=0, value=1)

motorL = DCMotor(MCP, IN1, IN2, PWM(Pin(ENA), freq=frequency))
motorR = DCMotor(MCP, IN4, IN3, PWM(Pin(ENB), freq=frequency))

# Color sensor
TCS = TCS3200(MCP, S2, S3, LED, Pin(OUT, Pin.IN, Pin.PULL_UP))


def checkI2CDevices():
    busB = [0x36]
    busA = [0x36, 0x68, 0x20]

    devicesA = I2CA.scan()
    devicesB = I2CB.scan()

    # TODO: Check if this works
    print("DEBUGA", [hex(d) for d in devicesA])
    print("DEBUGB", [hex(dc) for dc in devicesB])

    found = 0
    for addra in busA:
        if addra not in devicesA:
            print(hex(addra), "Expected, but not found in busA")
        else:
            found += 1

    for addrb in busB:
        if addrb not in devicesB:
            print(hex(addrb), "Expected, but not found in busB")
        else:
            found += 1

    return True if found == 4 else False

# def driveToBoxAndConnect():
#     while not SWITCH.value():
#         print(".")
#         time.sleep(0.1)
#         # TODO: drive straight on the line to the box.
#         pass
#
#
# # # Blocking when magnet is not detected by sensors.
# # #TODO: Check if this works
# while not ASL.MD or not ASR.MD:
#     # L = 1 if ASL.MD else 0
#     print("Magnet not detected by AS5600 L/R", ASL.MD, ASR.MD)
#     time.sleep(0.3)

MASL = ASMethods(ASL.RAWANGLE)
MASR = ASMethods(ASR.RAWANGLE)

a, b = False, False


pidR = PID(5, 0, 0)
pidL = PID(5, 0, 0)


while True:
    degAngleL = MASL.toDeg(ASL.RAWANGLE)
    totalAngleL = MASL.checkQuadrant(degAngleL)
    posL = totalAngleL / 0.45

    degAngleR = MASR.toDeg(ASR.RAWANGLE)
    totalAngleR = MASR.checkQuadrant(degAngleR)
    posR = totalAngleR / 0.45

    setpoint = 2000
    errL = (setpoint - posL)
    errR = (setpoint - posR)


    print(
        f" \n\
        Deg: {degAngleL} \t {degAngleR} \n\
        Angle: {totalAngleL} \t {totalAngleR}\n\
        Pos: {posL} \t {posR} \n\
        "
    )

    # time.sleep(0.2)

# import uasyncio as a
# import network
#
# ws = AsyncWebsocketClient(3000)
#
# lock = a.Lock()
# data_from_ws = []

# async def connectToWifi():
#     wifi = network.WLAN(network.STA_IF)
#     wifi.active(1)
#
#     while not wifi.isconnected():
#         print("Wifi connecting.. ")
#
#         if wifi.status() != network.STAT_CONNECTING:
#             wifi.connect(SSID, PASSWORD)
#
#         await a.sleep(0.4)
#
#     if wifi.isconnected():
#         print("ifconfig: {}".format(wifi.ifconfig()))
#     else:
#         print("Wifi not connected.")
#
#     return wifi

# p2 = Pin(2, Pin.OUT)
# async def blink_sos():
#     global p2
#
#     async def blink(on_ms: int, off_ms: int):
#         p2.on()
#         await a.sleep_ms(on_ms)
#         p2.off()
#         await a.sleep_ms(off_ms)
#
#     await blink(200, 50)
#     await blink(200, 50)
#     await blink(200, 50)
#     await blink(400, 50)
#     await blink(400, 50)
#     await blink(400, 50)
#     await blink(200, 50)
#     await blink(200, 50)
#     await blink(200, 50)
#
#
# async def blink_loop():
#     global lock
#     global data_from_ws
#     global ws
#
#     # Main "work" cycle. It should be awaitable as possible.
#     while True:
#         # await blink_sos()
#         if ws is not None:
#             if await ws.open():
#                 await ws.send('SOS!')
#                 print("SOS!", end=' ')
#
#             # lock data archive
#             await lock.acquire()
#             if data_from_ws:
#                 for item in data_from_ws:
#                     print("\nData from ws: {}".format(item))
#                 data_from_ws = []
#             lock.release()
#             gc.collect()
#
#         await a.sleep_ms(400)
#
# async def read_loop():
#     global config
#     global lock
#     global data_from_ws
#
#     # may be, it
#     wifi = await connectToWifi()
#     while True:
#         gc.collect()
#         if not wifi.isconnected():
#             wifi = await connectToWifi()
#             if not wifi.isconnected():
#                 await a.sleep_ms(4000)
#                 continue
#         try:
#             print("Handshaking...")
#
#             # connect to test socket server with random client number
#             if not await ws.handshake("{}{}".format(f"ws://{wifi.ifconfig()[2]}:8000/", randint(1, 100))):
#                 raise Exception('Handshake error.')
#             print("...handshaked.")
#             mes_count = 0
#             while await ws.open():
#                 data = await ws.recv()
#                 print("Data: " + str(data) + "; ")
#                 # ?lose socket for every 10 messages (even ping/pong)
#                 if mes_count == 10:
#                     await ws.close()
#                     print("ws is open: " + str(await ws.open()))
#                 mes_count += 1
#                 if data is not None:
#                     await lock.acquire()
#                     data_from_ws.append(data)
#                     lock.release()
#
#                 await a.sleep_ms(50)
#         except Exception as ex:
#             print("Exception: {}".format(ex))
#             await a.sleep(1)
# async def main():
#     tasks = [read_loop(), blink_loop()]
#     await a.gather(*tasks)

# s