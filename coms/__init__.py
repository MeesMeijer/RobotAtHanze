import espnow
import network


class Coms:

    peers: [bytes] = []
    esp: espnow.ESPNow

    def __init__(self, peer: bytes):
        sta = network.WLAN(network.STA_IF)  # Or network.AP_IF
        sta.active(True)
        sta.disconnect()      # For ESP8266

        # Initialize ESP-NOW
        esp = espnow.ESPNow()
        esp.active(True)

        # Define the MAC address of the receiving ESP32 (ESP32 B)
        self.esp = esp
        self.add_peer(peer)

    def add_peer(self, mac: bytes):
        self.esp.add_peer(mac)
        self.peers.append(mac)

    def send(self, msg: str):
        # print(msg, peers)
        if len(msg) > 250:
            raise Exception("Msg len to big.. max 250")

        for p in self.peers:
            # print(f"Sending data packet to {bytes(p)}")
            self.esp.send(p, msg)

    def write(self, *args):
        return self.send(str(args))

    def available(self):
        return self.esp.any()

    def recv(self, timeout: int = 0):
        return self.esp.recv(timeout)
