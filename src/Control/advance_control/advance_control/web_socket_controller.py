import asyncio
import websockets
import json
import threading
import time

# Constant
DEFAULT_COMMANDS = {
    'x': 0.0, 'y': 0.0, 'z': 0.0, 
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'record': False, 'pause': False
}

class WebSocketController:
    """
    Manages the WebSocket server to receive control inputs and exposes a 
    thread-safe API for ROS2 nodes to read the latest commands.
    """
    def __init__(self, host="0.0.0.0", port=8765, enable_logging=False):
        self._host = host
        self._port = port
        self._enable_logging = enable_logging
        
        self._latest_commands = DEFAULT_COMMANDS.copy()
        self._is_running = False
        self._is_connected = False
        self._server_thread = None
        self._lock = threading.Lock() # Protects _latest_commands
        # Note: We create the loop inside the thread to ensure thread-safety
        
    def _update_commands(self, data):
        """Thread-safe update of control data."""
        with self._lock:
            self._latest_commands.update(data)
            
    def _get_commands_copy(self):
        """Thread-safe retrieval of control data."""
        with self._lock:
            return self._latest_commands.copy()

    def get_latest_commands(self):
        """Public API method for ROS2 node to retrieve the current controller state."""
        return self._get_commands_copy()

    def is_connected(self):
        """Returns connection status."""
        return self._is_connected

    async def _websocket_handler(self, ws):
        """Handles incoming WebSocket connection and message flow."""
        client_ip = ws.remote_address[0]
        self._is_connected = True
        if self._enable_logging:
            print(f"📱 Phone connected: {client_ip}")

        try:
            async for msg in ws:
                try:
                    data = json.loads(msg)
                    self._update_commands(data)
                    
                    if self._enable_logging:
                        pause_status = f"PAUSE" if (data['record'] and data['pause']) else ""
                        print(f"🎮 Forward:{data['x']:> 6.2f} Left:{data['y']:> 6.2f} Up:{data['z']:> 6.2f} | Roll:{data['roll']:> 6.2f} Pitch:{data['pitch']:> 6.2f} Yaw:{data['yaw']:> 6.2f} | {'REC' if data['record'] else '   '} {pause_status}")

                    # # Simulated Drone Telemetry Feedback
                    # state = {"armed": True, "battery": 95}
                    # await ws.send(json.dumps(state))

                except json.JSONDecodeError:
                    pass

        except websockets.exceptions.ConnectionClosedOK:
            pass 
        except Exception as e:
            if self._enable_logging:
                print(f"❌ Connection error: {e}")
        
        finally:
            self._is_connected = False
            # Reset commands to zero when disconnected for safety
            self._update_commands(DEFAULT_COMMANDS.copy())
            if self._enable_logging:
                print(f"👋 Phone disconnected: {client_ip}")

    async def _run_server(self):
        """Runs the WebSocket server asynchronously."""
        async with websockets.serve(
            self._websocket_handler, 
            self._host, 
            self._port, 
            ping_interval=None # Critical: Disable pings to prevent mobile disconnects
        ):
            if self._enable_logging:
                print(f"🚀 WebSocket API running on ws://{self._host}:{self._port}")
            self._is_running = True
            await asyncio.Future() # Run forever

    def _start_asyncio_loop(self):
        """Entry point for the dedicated server thread."""
        # Create a new event loop strictly for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._run_server())
        except Exception as e:
            print(f"Server Thread Error: {e}")
        finally:
            loop.close()

    def start(self):
        """Starts the WebSocket server in a background thread."""
        if not self._is_running:
            self._server_thread = threading.Thread(target=self._start_asyncio_loop, daemon=True)
            self._server_thread.start()
            
    def stop(self):
        """Stops the WebSocket server."""
        if self._is_running:
            # Note: Hard stopping a thread loop is tricky, usually we let daemon handle it
            # or use an asyncio Event to stop. For this use case, daemon exit is fine.
            self._is_running = False
            print("🛑 WebSocket Controller Stopping...")

if __name__ == "__main__":
    # Enable logging when running standalone
    controller = WebSocketController(enable_logging=True)
    controller.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller.stop()