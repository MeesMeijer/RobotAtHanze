
# Test websocket server.
# Created to test keepalive websocket connections with ESP32 controllers.

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Depends, status
from contextlib import asynccontextmanager

from fastapi.exceptions import HTTPException
import json
import asyncio
from datetime import datetime
import logging

async def startup():
    app.wss = {}

    app.logger = logging.getLogger('ws_test')
    app.logger.setLevel(logging.INFO)

    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)

    # create formatter
    formatter = logging.Formatter('%(asctime)s: %(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    app.logger.addHandler(ch)

    app.logger.info("Test websocket server started. Every message from client will be broadcasted to all other clients.")


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Load the ML model
    await startup()
    yield

    app.logger.info("Server stopped.")

    # Clean up the ML models and release the resources

app = FastAPI(lifespan=lifespan)

@app.websocket("/{id}")
async def websocket_endpoint(websocket: WebSocket, id: int):

    try:
        if id in app.wss.keys():
            await websocket.close(code=status.WS_1001_GOING_AWAY)
            app.logger.info("Client {} already exists.".format(id))
            return

        await websocket.accept()
        app.wss[id] = websocket

        app.logger.info("New websocket client: {}; client: {}".format(id, websocket.headers.get("X-SSL-Client")))

        while True:
            # print(websocket.)
            data = await websocket.receive_text()

            app.logger.info("Data from client {}: {}".format(id, data))

            for client, ws in app.wss.items():
                # if client == id:
                #     continue
                await ws.send_text("Mes from {}: {}".format(id, data))
                app.logger.info("Data {} have been sent to {}.".format(data, client))

    except Exception as ex:
        app.logger.info("Client {} is disconnected: {}".format(id, ex))
        app.wss.pop(id, None)


