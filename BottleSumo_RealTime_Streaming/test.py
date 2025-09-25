import socket, json
import time
s = socket.socket()
s.connect(('ip', 4242))
while True:
    data = s.recv(1024)
    if data:
        try:
            json_data = json.loads(data.decode())
            print(f"Action: {json_data['robot_state']['action']}")
            print(f"Sensors: {json_data['sensors']['voltage']}")
            time.sleep(0.5)
        except:
            pass