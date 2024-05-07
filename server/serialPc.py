import serial, threading, time
import json

# Initialize serial communication
ser = serial.Serial('COM4', 115200)  # Replace 'COM1' with the correct port for your system


def userInput():
    while True:
        t = ",".join([str(1) for _ in range(400)])
        # user_input = input("Enter message to send to ESP32: ")
        ser.write(t.encode() + b'\n')
        time.sleep(10)

def res():
    while True:
        # Wait for data from ESP32
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            print("Received temperature:", received_data)

        time.sleep(0.01)
        # Send data to ESP32


t1 = threading.Thread(None, userInput, daemon=None)
t2 = threading.Thread(None, res, daemon=True)


try:
    t1.start()
    t2.start()
except KeyboardInterrupt:
    pass


# mac = b'\xd4\x8a\xfc\xa5\x80\xe8'

# import espnow
# import machine
# import network

# sta = network.WLAN(network.STA_IF)
# sta.active(True)

# esp = espnow.ESPNow()
# esp.active(True)

# esp.add_peer(mac)

# while  True:

#     if esp.any():
#         mac, data = esp.recv(0)
#         print(data.decode("utf-8")+ "\n")


# mac = b'\xd4\x8a\xfc\xa5\x80\xe8'

# import espnow
# import machine
# import network
# from machine import UART
# sta = network.WLAN(network.STA_IF)
# sta.active(True)

# esp = espnow.ESPNow()
# esp.active(True)

# esp.add_peer(mac)
# uart = UART(1, 115200, tx=1, rx=3)

# while  True:

#     if esp.any():
#         mac, data = esp.recv(0)
#         print(data.decode("utf-8")+ "\n")
#         uart.write(data.decode("utf-8")+ "\n")

#     if uart.any():
#         strs = uart.readline().decode().strip()
#         print("Got: "+ strs)
#         esp.send(strs)


import serial, threading, time
import json

# Initialize serial communication
ser = serial.Serial('COM4', 115200)  # Replace 'COM1' with the correct port for your system


def userInput():
    while True:
        t = ",".join([str(1) for _ in range(400)])
        # user_input = input("Enter message to send to ESP32: ")
        ser.write(t.encode() + b'\n')
        time.sleep(10)

def res():
    while True:
        # Wait for data from ESP32
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            print("Received temperature:", received_data)

        time.sleep(0.01)
        # Send data to ESP32


t1 = threading.Thread(None, userInput, daemon=None)
t2 = threading.Thread(None, res, daemon=True)


try:
    t1.start()
    t2.start()
except KeyboardInterrupt:
    pass


# mac = b'\xd4\x8a\xfc\xa5\x80\xe8'

# import espnow
# import machine
# import network

# sta = network.WLAN(network.STA_IF)
# sta.active(True)

# esp = espnow.ESPNow()
# esp.active(True)

# esp.add_peer(mac)

# while  True:

#     if esp.any():
#         mac, data = esp.recv(0)
#         print(data.decode("utf-8")+ "\n")


# mac = b'\xd4\x8a\xfc\xa5\x80\xe8'

# import espnow
# import machine
# import network
# from machine import UART
# sta = network.WLAN(network.STA_IF)
# sta.active(True)

# esp = espnow.ESPNow()
# esp.active(True)

# esp.add_peer(mac)
# uart = UART(1, 115200, tx=1, rx=3)

# while  True:

#     if esp.any():
#         mac, data = esp.recv(0)
#         print(data.decode("utf-8")+ "\n")
#         uart.write(data.decode("utf-8")+ "\n")

#     if uart.any():
#         strs = uart.readline().decode().strip()
#         print("Got: "+ strs)
#         esp.send(strs)








import serial, threading, time
import json

# Initialize serial communication
ser = serial.Serial('COM4', 115200)  # Replace 'COM1' with the correct port for your system


def userInput():
    while True:
        t = ",".join([str(1) for _ in range(400)])
        # user_input = input("Enter message to send to ESP32: ")
        ser.write(t.encode() + b'\n')
        time.sleep(10)

def res():
    while True:
        # Wait for data from ESP32
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            print("Received temperature:", received_data)

        time.sleep(0.01)
        # Send data to ESP32


t1 = threading.Thread(None, userInput, daemon=None)
t2 = threading.Thread(None, res, daemon=True)


try:
    t1.start()
    t2.start()
except KeyboardInterrupt:
    pass


# mac = b'\xd4\x8a\xfc\xa5\x80\xe8'

# import espnow
# import machine
# import network

# sta = network.WLAN(network.STA_IF)
# sta.active(True)

# esp = espnow.ESPNow()
# esp.active(True)

# esp.add_peer(mac)

# while  True:

#     if esp.any():
#         mac, data = esp.recv(0)
#         print(data.decode("utf-8")+ "\n")


# mac = b'\xd4\x8a\xfc\xa5\x80\xe8'

# import espnow
# import machine
# import network
# from machine import UART
# sta = network.WLAN(network.STA_IF)
# sta.active(True)

# esp = espnow.ESPNow()
# esp.active(True)

# esp.add_peer(mac)
# uart = UART(1, 115200, tx=1, rx=3)

# while  True:

#     if esp.any():
#         mac, data = esp.recv(0)
#         print(data.decode("utf-8")+ "\n")
#         uart.write(data.decode("utf-8")+ "\n")

#     if uart.any():
#         strs = uart.readline().decode().strip()
#         print("Got: "+ strs)
#         esp.send(strs)





