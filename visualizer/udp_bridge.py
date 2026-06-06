import asyncio
import socket
import websockets
import json

connected_clients = set()

async def ws_handler(websocket):
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            pass  # We only send telemetry, no need to receive client messages
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.remove(websocket)

async def udp_listener():
    # Bind UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    sock.setblocking(False)
    
    print("UDP listener started on port 5005...")
    
    loop = asyncio.get_running_loop()
    while True:
        try:
            data, addr = await loop.sock_recvfrom(sock, 65535)
            message = data.decode("utf-8")
            if connected_clients:
                # Forward to all connected websocket clients
                await asyncio.gather(
                    *[client.send(message) for client in connected_clients],
                    return_exceptions=True
                )
        except Exception as e:
            await asyncio.sleep(0.01)

async def main():
    server = await websockets.serve(ws_handler, "0.0.0.0", 8765)
    print("WebSocket server started on port 8765...")
    
    await asyncio.gather(server.wait_closed(), udp_listener())

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Bridge stopped.")
