import zmq
import json
from flask import Flask, render_template
from flask_socketio import SocketIO
from signal_processing import SignalProcessor

# Initialize Flask & SocketIO
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ppg_tajna'
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize signal processor (normalize for plotting)
procesor = SignalProcessor(sampling_rate=50)

# ZMQ setup to receive raw IR data from Raspberry Pi
context = zmq.Context()
zmq_socket = context.socket(zmq.SUB)

# Make sure to check the IP matches of your Raspberry Pi
zmq_socket.connect("tcp://10.1.150.45:5556")
zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "Raw/ir")

print("Web Server i Raw Subscriber pokrenuti...")

# Keep a small rolling buffer for plotting
moj_bafer = []

# Background function that reads from ZMQ
def background_zmq_loop():
    global moj_bafer
    while True:
        try:
            # Receive message from RPi
            string = zmq_socket.recv_string()

            # Split topic from JSON payload
            parts = string.split(" ", 1)
            if len(parts) < 2:
                continue

            topic, json_data = parts
            data_dict = json.loads(json_data)

            novi_podaci = data_dict["niz"] 
            #moj_bafer.extend(novi_podaci)
            moj_bafer=novi_podaci[-200:] if len(novi_podaci) >= 200 else novi_podaci

            if len(moj_bafer) > 200:
                moj_bafer = moj_bafer[-200:]

            # Filter only if there is enough data (200 samples)
            if len(moj_bafer) >= 200:
                cisti_niz = procesor.filter_for_plot(moj_bafer)

                # Send the filtered PPG signal to the website
                socketio.emit('new_ppg_data', {'niz': cisti_niz})
                print("IR: ", cisti_niz[0])
            else:
                print(f"Punjenje bafera... {len(moj_bafer)}/200", end="\r")

        except Exception as e:
            print(f"Greška u subRaw: {e}")

# Handlers for messages coming from subBPM and subSPO2
@socketio.on('new_bpm')
def handle_bpm(data):
    socketio.emit('update_bpm', data)

@socketio.on('new_spo2')
def handle_spo2(data):
    socketio.emit('update_spo2', data)

# Home page route
@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    socketio.start_background_task(background_zmq_loop)
    socketio.run(app, host='0.0.0.0', port=5000)