# backend_ws.py
import asyncio
import websockets
import json
import socket
from collections import deque
import threading
from datetime import datetime
import uuid
tcp_client_socket = None  # STM32_1 client socket
tcp_client_socket_2 = None  # STM32_2 client socket
connected_clients = set()
msg_queue = asyncio.Queue()
clients = {
    "client_1": None,
    "client_2": None,
}

# ===Frontend WebSocket Server===


# def frontend_to_STM1(msg):
#     tcp_client_socket.send(msg.encode('utf-8'))


# def frontend_to_STM2(msg):
#     tcp_client_socket_2.send(msg.encode('utf-8'))


async def frontend_to_STM1(msg):
    reader, writer = clients["client_1"]
    if writer:
        writer.write(msg.encode('utf-8'))
        await writer.drain()
    else:
        print("client_1 not connected")


async def frontend_to_STM2(msg):
    reader, writer = clients["client_2"]
    if writer:
        writer.write(msg.encode('utf-8'))
        await writer.drain()
    else:
        print("client_2 not connected")


async def handler(websocket):
    # Handle incoming messages from the frontend
    connected_clients.add(websocket)
    print(f"Client connected: {websocket.remote_address}")
    try:
        async for message in websocket:
            data = json.loads(message)
            payload = data.get("payload", {})
            type = data.get("type", "unknown")

            if (type == "getLog"):
                print("Frontend: Received log request")
                with open("log.json", "r", encoding="utf-8") as f:
                    loadData = json.load(f)
                    print(loadData)
                    await websocket.send(json.dumps({"type": "getLog", "payload": loadData}))
                print("Sent log data to frontend")

            elif (type == "setOut"):
                print("Frontend: Received setOut request")

                await websocket.send(json.dumps({"type": "setOut", "payload": {"success": True}}))

            elif (type == "sendNum"):
                num = payload.get("num", 0)
                print(f"Received number: {num}")
                await frontend_to_STM2(f"start out: {num}")
                await frontend_to_STM1(f"start out: {num}")

            else:
                print(f"Unknown message type: {type}")

            # print(f"[Frontend] says: {data}")
            # await websocket.send(f"Got your message: {data}")
    except websockets.ConnectionClosed:
        print("Client disconnected")
    finally:
        connected_clients.remove(websocket)

# === TCP to WebSocket Forwarding ===


async def send_to_frontend(msg):
    for client in connected_clients.copy():
        try:
            print(f"Forwarding message to {client.remote_address}: {msg}")
            await client.send(msg)
        except:
            connected_clients.remove(client)


def find_id_in_log(uid):
    try:
        with open("RFIDCard.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            for entry in data:
                if entry["uid"] == uid:
                    return entry["type"]
    except FileNotFoundError:
        print("Log file not found.")
    return None


def append_log(uid, action):
    type = find_id_in_log(uid)
    log_entry = {
        "time": datetime.now().isoformat(),
        "uid": uid,
        "type": type,
        "action": action,
        "id": str(uuid.uuid4())
    }
    with open("log.json", "r+", encoding="utf-8") as f:
        data = json.load(f)
        data.append(log_entry)
        f.seek(0)
        f.write(json.dumps(data, ensure_ascii=False, indent=2))
        f.truncate()
    print(f"Appended to log: {log_entry}")
    return log_entry


def storage_record(logEntry):
    log_entry = {
        "time": logEntry["time"],
        "uid": logEntry["uid"],
        "type": logEntry["type"],
    }
    with open("storage.json", "r+", encoding="utf-8") as f:
        data = json.load(f)
        if (logEntry["action"] == "out"):
            data = [entry for entry in data if not entry["uid"]
                    == logEntry["uid"]]
        else:
            data.append(log_entry)
        f.seek(0)  # Move to the beginning of the file
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.truncate()  # Remove any leftover data
    print(f"Appended to storage: {log_entry}")
    return log_entry


async def forward_tcp_to_frontend():
    while True:
        msg = await msg_queue.get()
        if (msg == "storing start"):
            print("Received 'storing start' command")
            await frontend_to_STM2("storing start")
            await send_to_frontend(json.dumps({"type": "setIn", "payload": {"start": True}}))
        elif (msg == "stop storing"):
            await frontend_to_STM2("stop")
            await send_to_frontend(json.dumps({"type": "setIn", "payload": {"start": False}}))
        elif (msg == "stop out"):
            await frontend_to_STM2("stop")
            await send_to_frontend(json.dumps({"type": "endOut", "payload": {}}))
        else:
            [action, uid] = msg.split(": ")
            logEntry = append_log(uid, action)
            storage_record(logEntry)
            if (action == "storing"):
                await frontend_to_STM2(f"storing: {logEntry['type']}")
            await send_to_frontend(json.dumps({"type": "updateLog", "payload": logEntry}))

# === STM32_1 Server ===


async def handle_client(reader, writer, queue):
    addr = writer.get_extra_info('peername')
    print(f"New connection from {addr}")
    if clients["client_1"] is None and addr[0] == "172.20.10.5":
        clients["client_1"] = [reader, writer]
        name = "client_1"
    elif clients["client_2"] is None and addr[0] == "172.20.10.6":
        clients["client_2"] = [reader, writer]
        name = "client_2"
    else:
        print("Too many clients, rejecting connection.")
        writer.close()
        await writer.wait_closed()
        return

    print(f"{name} connected: {addr}")

    try:
        while name == "client_1":
            data = await reader.read(1024)
            if not data:
                break
            message = data.decode('utf-8').strip()
            print(f"Received from {name}: {message}")
            await msg_queue.put(message)
            # writer.write(b"ACK\n")
            # await writer.drain()
        else:
            print(f"{name} does not send messages. Just connected.")
            await asyncio.Future()  # suspend forever

    except Exception as e:
        print(f"Error with {name}: {e}")
    finally:
        print(f"{name} disconnected")
        clients[name] = None
        writer.close()
        await writer.wait_closed()


async def start_tcp_server(queue):
    host = '172.20.10.2'
    port = 8002

    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, queue),
        host, port
    )

    addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
    print(f"Server listening on {addrs}")

    async with server:
        await server.serve_forever()
# def tcp_server(queue):
#     global tcp_client_socket
#     global tcp_client_socket_2
#     HOST = '172.20.10.2'  # Listen on all available interfaces(Mine)
#     PORT = 8002       # Port to listen on
#     BUFFER_SIZE = 1024  # Max amount of data received at once

#     # Set up socket server
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.bind((HOST, PORT))
#     server_socket.listen(2)
#     print(f"Listening on {HOST}:{PORT}...")
#     # block until a client connects
#     while (True):
#         if (tcp_client_socket is None):
#             tcp_client_socket, client_address = server_socket.accept()
#             print(f"Connection from {client_address}")
#         elif (tcp_client_socket_2 is None):
#             tcp_client_socket_2, client_address2 = server_socket.accept()
#             print(f"Connection from {client_address2}")
#         else:
#             break
#     print(f"Connection from {client_address}")

#     try:
#         while True:
#             print("Waiting for data from client...")
#             data = tcp_client_socket.recv(BUFFER_SIZE).decode('utf-8').strip()
#             print(data)
#             queue.put_nowait(data)
#             # tcp_client_socket.send("ACK: Received\n".encode('utf-8'))

#     except KeyboardInterrupt:
#         print("Exiting gracefully...")
#     except Exception as e:
#         print(f"Error: {e}")

#     finally:
#         print("Closing sockets...")
#         tcp_client_socket.close()
#         server_socket.close()


async def main():
    #  daemon=True allows the thread to exit when the main program exits
    # threading.Thread(target=tcp_server, args=(msg_queue,), daemon=True).start()
    queue = asyncio.Queue()
    asyncio.create_task(start_tcp_server(queue))
    async with websockets.serve(handler, "0.0.0.0", 4000):
        print("WebSocket server started on port 4000")
        await forward_tcp_to_frontend()
        # await asyncio.Future()  # Run forever

asyncio.run(main())
