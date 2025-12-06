import json
import socket
import threading
import time
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global state
robot_socket = None
reader_thread = None
stop_event = threading.Event()
robot_connected = False

DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 8080
SOCKET_TIMEOUT = 2.0
SOCKET_RECV_BYTES = 4096

class TelemetryReader(threading.Thread):
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.sock = None

    def run(self):
        global robot_socket, robot_connected
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=SOCKET_TIMEOUT)
            self.sock.settimeout(SOCKET_TIMEOUT)
            robot_socket = self.sock
            robot_connected = True
            socketio.emit('status', {'msg': f'Connected to {self.host}:{self.port}', 'connected': True})
        except Exception as e:
            socketio.emit('status', {'msg': f'Connection failed: {e}', 'connected': False})
            return

        buffer = ""
        while not stop_event.is_set():
            try:
                chunk = self.sock.recv(SOCKET_RECV_BYTES)
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        try:
                            data = json.loads(line)
                            socketio.emit('telemetry', data)
                        except json.JSONDecodeError:
                            pass
            except socket.timeout:
                continue
            except Exception as e:
                socketio.emit('status', {'msg': f'Error: {e}', 'connected': False})
                break
        
        robot_connected = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        socketio.emit('status', {'msg': 'Disconnected', 'connected': False})

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect_robot')
def handle_connect_robot(data):
    global reader_thread, stop_event
    host = data.get('host', DEFAULT_HOST)
    try:
        port = int(data.get('port', DEFAULT_PORT))
    except:
        emit('status', {'msg': 'Invalid port', 'connected': False})
        return

    if reader_thread and reader_thread.is_alive():
        stop_event.set()
        reader_thread.join()

    stop_event.clear()
    reader_thread = TelemetryReader(host, port)
    reader_thread.start()

@socketio.on('disconnect_robot')
def handle_disconnect_robot():
    global stop_event
    stop_event.set()

@socketio.on('send_command')
def handle_command(data):
    global robot_socket
    if robot_socket and robot_connected:
        try:
            cmd = data.get('command', '')
            if cmd:
                robot_socket.sendall((cmd + "\n").encode('utf-8'))
        except Exception as e:
            emit('status', {'msg': f'Send error: {e}', 'connected': False})

@socketio.on('set_threshold')
def handle_threshold(data):
    global robot_socket
    if robot_socket and robot_connected:
        try:
            # {"cmd":"set_threshold","sensor":idx,"value":value}
            cmd_json = json.dumps(data)
            robot_socket.sendall((cmd_json + "\n").encode('utf-8'))
        except Exception as e:
            emit('status', {'msg': f'Send error: {e}', 'connected': False})

if __name__ == '__main__':
    print("Starting Web Viewer on 0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
