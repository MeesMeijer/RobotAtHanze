from ws import AsyncWebsocketClient
import uasyncio as a
from machine import Pin
import time, gc, json, random, network


pi = Pin(2, Pin.OUT)

lock = a.Lock()
dataFromWs = []

ws = AsyncWebsocketClient(5)
count = 0

async def blinkLoop():
    global ws, lock, dataFromWs, count
    while True:
        await a.sleep_ms(50)

        if await ws.open():
            await ws.send("SOS+"+ str(count))
        await lock.acquire()

        if dataFromWs:
            for item in dataFromWs:
                print("\nData from ws: {}".format(item))
            dataFromWs = []

        lock.release()
        gc.collect()


async def readLoop():
    global ws, lock, dataFromWs, count

    if do_connect():
        while True:
            gc.collect()
            do_connect()

            try:
                print("handsaking")

                if not await ws.handshake("{}{}".format("ws://192.168.2.7:5000/", random.randint(1, 100))):
                    raise Exception('Handshake error.')
                print("...handshaked.")

                mes_count = 0
                while await ws.open():
                    data = await ws.recv()
                    print("Data: " + str(data) + "; " + str(mes_count))
                    #close socket for every 10 messages (even ping/pong)
                    #if mes_count == 10:
                    #await ws.close()
                    #print("ws is open: " + str(await ws.open()))
                    mes_count += 1
                    count = mes_count
                    if data is not None:
                        await lock.acquire()
                        dataFromWs.append(data)
                        lock.release()

                    await a.sleep_ms(50)

            except Exception as ex:
                print("Exception: {}".format(ex))
                await a.sleep(1)


async def main():
    tasks = [readLoop(), blinkLoop()]
    await a.gather(*tasks)

a.run(main())

