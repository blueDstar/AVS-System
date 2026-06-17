import asyncio
import sys

try:
    import websockets
except ImportError:
    print("Python websockets package not found. Installing it for test...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets"])
    import websockets

async def test():
    uri = "ws://localhost:8000/ws"
    print(f"Connecting to {uri}...")
    try:
        async with websockets.connect(uri) as websocket:
            print("Connection established successfully!")
            for _ in range(5):
                msg = await websocket.recv()
                print(f"Received telemetry: {msg[:100]}... (Total length: {len(msg)})")
    except Exception as e:
        print(f"WebSocket client error: {e}")

if __name__ == "__main__":
    asyncio.run(test())
