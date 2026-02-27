import zmq
from socketio import Client
sio = Client()

# Connect to the local Flask/SocketIO server
sio.connect('http://localhost:5000')

context = zmq.Context()
socket = context.socket(zmq.SUB)

# Connect to the publisher (Raspberry Pi) and receive SpO2 updates
socket.connect("tcp://10.1.150.45:5556")
socket.setsockopt_string(zmq.SUBSCRIBE, "Vitals/spo2")

while True:
	# Receive a message
	string = socket.recv_string()
	topic, spo2 = string.split()
	
	# Forward the value directly to the website
	sio.emit('new_spo2', {'value': spo2})
	print("SpO2 poslat na sajt:", spo2)
