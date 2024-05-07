mac = b'\xd4\x8a\xfc\xa5\x80\xe8'

import espnow
import machine
import network
from machine import UART
sta = network.WLAN(network.STA_IF)
sta.active(True)

esp = espnow.ESPNow()
esp.active(True)

esp.add_peer(mac)
uart = UART(1, 115200, tx=1, rx=3)

while  True:

    if esp.any():
        mac, data = esp.recv(0)
        print(data.decode("utf-8")+ "\n")
        uart.write(data.decode("utf-8")+ "\n")

    if uart.any():
        strs = uart.readline().decode().strip()
        print("Got: "+ strs)
        esp.send(strs)


