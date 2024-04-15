import network

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

def do_connect():
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect('WE2-AD2BBB', '')
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())
    return True

do_connect()
