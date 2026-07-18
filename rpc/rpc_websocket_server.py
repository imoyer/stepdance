import time
import json
import argparse
import asyncio
from websockets.asyncio.server import serve, broadcast
import serial_asyncio

# Install packages:
# pip install websockets
# pip install pyserial-asyncio

# Run:
# - first make sure a compatible sketch is loaded on the stepdance board (eg p5_ui.ino)
# - then run the python script:
# python rpc_websocket_server.py --arduino-port "/dev/cu.usbmodem184303901" --serial-send-delay 0.1
# replacing the arduino port by the correct port for your board, and changing the delay if you wish to (default = 0.1)


parser = argparse.ArgumentParser()
parser.add_argument(
    "--arduino-port", default="COM4", help="Arduino port"
)
parser.add_argument(
    "--serial-send-delay", default=0.1, type=float, help="The amount of time we wait between each send of a command through serial"
)

args = parser.parse_args()



# ===== SERIAL (connection to stepdance board) =====

# This queue is where we store the commands received from UI, while waiting for them to be sent
# to stepdance through serial
serial_send_queue = []

async def stepdance_listener(r : asyncio.StreamReader):
    while True:
        # print("stepdance loop")
        message_from_serial = await r.readuntil(b'\n')
        # print + send through websocket to frontend
        print(f'received: {message_from_serial.rstrip().decode()}')
        ws_send(f'{message_from_serial.rstrip().decode()}')

async def stepdance_send(w : asyncio.StreamWriter, msg : bytes):
    w.write(msg)
    await w.drain()


# ===== WEBSOCKET (connection to frontend) =====
websocket_connections = set()
def ws_send(msg):
    broadcast(websocket_connections, msg)
    

async def ws_handler(websocket):
    print("run ws_handler")
    websocket_connections.add(websocket)
    while True:
        # print("ws loop")
        message = await websocket.recv()
        print("ws rcv:", message)

        # m_data = json.loads(message)

        # add the message to the queue that is consumed by the loop that sends to serial
        message_to_send = message.encode('utf-8') + b'\n'
        serial_send_queue.append(message_to_send)
        

async def wait_fixed_time(prev_exec_time, wait_time):
    while time.perf_counter() < prev_exec_time + wait_time:
        pass

async def main_loop_fixed_time(stepdance_writer, period_s):
    prev_exec_time = time.perf_counter()
    while True:
        await asyncio.create_task(wait_fixed_time(prev_exec_time, period_s))

        # Execute below code every time period_s has elapsed
        # print("running fixed time loop, time:", time.perf_counter())

        if (len(serial_send_queue) > 0):
            serial_message = serial_send_queue.pop(0)
            print(f"Sending to serial: {serial_message}")
            await stepdance_send(stepdance_writer, serial_message)

        prev_exec_time = time.perf_counter()



async def main():
    reader, writer = await serial_asyncio.open_serial_connection(url=args.arduino_port, baudrate=115200)

    ws_server = await serve(ws_handler, "localhost", 8001)
    # print(ws_server)

    # await ws_server.serve_forever()

    await asyncio.gather(
        asyncio.create_task(ws_server.serve_forever()),
        asyncio.create_task(stepdance_listener(reader)),
        asyncio.create_task(main_loop_fixed_time(writer, args.serial_send_delay))
    )

   

loop = asyncio.get_event_loop()
loop.run_until_complete(main())
