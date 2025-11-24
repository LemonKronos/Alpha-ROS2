import asyncio
import websockets
import json
import threading
import time

LOGGER = False

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
    def __init__(self, host="0.0.0.0", port=8765):
        self._host = host
        self._port = port
        self._latest_commands = DEFAULT_COMMANDS.copy()
        self._is_running = False
        self._is_connected = False
        self._server_thread = None
        self._lock = threading.Lock() # Protects _latest_commands
        self._loop = asyncio.new_event_loop()
        
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
        print(f"📱 Phone connected: {client_ip}")

        try:
            async for msg in ws:
                try:
                    data = json.loads(msg)
                    self._update_commands(data)
                    
                    if LOGGER:
                        pause_status = f"PAUSE" if (data['record'] and data['pause']) else ""
                        print(f"🎮 Forward:{data['x']:> 6.2f} Left:{data['y']:> 6.2f} Up:{data['z']:> 6.2f} | Roll:{data['roll']:> 6.2f} Pitch:{data['pitch']:> 6.2f} Yaw:{data['yaw']:> 6.2f} | {'REC' if data['record'] else '   '} {pause_status}")

                    # # Simulated Drone Telemetry Feedback
                    # state = {"armed": True, "battery": 95}
                    # await ws.send(json.dumps(state))

                except json.JSONDecodeError:
                    print(f"⚠️ Received invalid JSON from {client_ip}")

        except websockets.exceptions.ConnectionClosedOK:
            pass # Handled in finally
        except Exception as e:
            print(f"❌ Phone connection lost due to error: {e}")
        
        finally:
            self._is_connected = False
            # Reset commands to zero when disconnected
            self._update_commands(DEFAULT_COMMANDS.copy())
            print(f"👋 Phone disconnected: {client_ip}")

    async def _run_server(self):
        """Runs the WebSocket server asynchronously."""
        async with websockets.serve(
            self._websocket_handler, 
            self._host, 
            self._port, 
            ping_interval=None # Disable aggressive pings for mobile devices
        ):
            print(f"🚀 WebSocket API running on ws://{self._host}:{self._port}")
            self._is_running = True
            await asyncio.Future()

    def _start_asyncio_loop(self):
        """Entry point for the dedicated server thread."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._run_server())

    def start(self):
        """Starts the WebSocket server in a background thread."""
        if not self._is_running:
            self._server_thread = threading.Thread(target=self._start_asyncio_loop, daemon=True)
            self._server_thread.start()
            
    def stop(self):
        """Stops the WebSocket server."""
        if self._is_running:
            print("🛑 Stopping WebSocket API...")
            self._loop.stop()
            self._is_running = False

# --- Example Usage ---
if __name__ == "__main__":
    LOGGER = True
    controller = WebSocketController()
    controller.start()

    # --- SIMULATING ROS2 NODE LOOP ---
    try:
        print("\n--- Starting simulated ROS2 read loop (Ctrl+C to exit) ---")
        while True:
            cmds = controller.get_latest_commands()
            if cmds['x'] != 0 or cmds['roll'] != 0 or cmds['record']:
                 print(f"ROS2 Read: X={cmds['x']:.2f}, Roll={cmds['roll']:.2f}, REC={cmds['record']}, PAUSE={cmds['pause']}")
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        controller.stop()
        print("Server shutdown complete.")