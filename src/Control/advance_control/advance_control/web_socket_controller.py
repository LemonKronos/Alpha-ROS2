import asyncio
import websockets
import json
import threading
import time

# --- SPLIT DATA STRUCTURES ---
DEFAULT_CONTROL = {
    'control_state': False,
    'x': 0.0, 'y': 0.0, 'z': 0.0, 
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
}

DEFAULT_RECORD = {
    'record': False, 'pause': False
}

class WebSocketController:
    """
    Manages WebSocket server with DUAL LOCKING.
    Splits Flight Control data from UI/State data to prevent lock contention.
    """
    def __init__(self, host="10.54.227.129", port=8765, enable_logging=False):
        self._host = host
        self._port = port
        self._enable_logging = enable_logging
        
        # Separate storage
        self._control_data = DEFAULT_CONTROL.copy()
        self._record_data = DEFAULT_RECORD.copy()
        
        # Separate locks
        self._lock_control = threading.Lock() 
        self._lock_record = threading.Lock()

        self._is_running = False
        self._is_connected = False
        self._server_thread = None

    def _update_commands(self, data):
        """Sorts incoming JSON into the correct bucket and locks only what is needed."""
        
        # 1. Update Control Data
        control_update = {k: v for k, v in data.items() if k in DEFAULT_CONTROL}
        if control_update:
            with self._lock_control:
                self._control_data.update(control_update)
        
        # 2. Update Record Data
        record_update = {k: v for k, v in data.items() if k in DEFAULT_RECORD}
        if record_update:
            with self._lock_record:
                self._record_data.update(record_update)

    # --- API FOR ROS NODE ---

    def get_latest_control_cmd(self):
        """Returns x, y, z, roll, pitch, yaw. Thread-safe."""
        with self._lock_control:
            return self._control_data.copy()

    def get_latest_record_cmd(self):
        """Returns record, pause. Thread-safe."""
        with self._lock_record:
            return self._record_data.copy()
        
    def check_is_connected(self):
        return self._is_connected

    # --- CONNECTION HANDLING ---

    def is_connected(self):
        return self._is_connected

    async def _websocket_handler(self, ws):
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
                        rec = data.get('record', False)
                        pse = data.get('pause', False)
                        arm = data.get('control_state', False)
                        print(f"{'🎮' if arm else '❌'} F:{data.get('x',0):>5.2f} L:{data.get('y',0):>5.2f} U:{data.get('z',0):>5.2f} | {'REC' if rec else '   '} {'PAUSE' if pse else '     '} |")

                except json.JSONDecodeError:
                    pass

        except Exception as e:
            if self._enable_logging:
                print(f"❌ Connection error: {e}")
        
        finally:
            self._is_connected = False
            # Reset BOTH to defaults on disconnect
            with self._lock_control:
                self._control_data = DEFAULT_CONTROL.copy()
            with self._lock_record:
                self._record_data = DEFAULT_RECORD.copy()
                
            if self._enable_logging:
                print(f"👋 Phone disconnected: {client_ip}")

    async def _run_server(self):
        async with websockets.serve(
            self._websocket_handler, 
            self._host, 
            self._port, 
            ping_interval=None 
        ):
            if self._enable_logging:
                print(f"🚀 WebSocket API running on ws://{self._host}:{self._port}")
            self._is_running = True
            await asyncio.Future() 

    def _start_asyncio_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._run_server())
        except Exception:
            pass
        finally:
            loop.close()

    def start(self):
        if not self._is_running:
            self._server_thread = threading.Thread(target=self._start_asyncio_loop, daemon=True)
            self._server_thread.start()
            
    def stop(self):
        self._is_running = False
        if self._enable_logging:
            print("🛑 WebSocket Controller Stopping...")

if __name__ == "__main__":
    controller = WebSocketController(enable_logging=True)
    controller.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller.stop()