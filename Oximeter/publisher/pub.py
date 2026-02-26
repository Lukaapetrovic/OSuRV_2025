import time
import zmq
import json
import numpy as np
from max30102_lib.max30102.heartrate_monitor import HeartRateMonitor

# Create a PUB socket that broadcasts messages to all SUB clients
# SUB clients subscribe to topics (Raw/ir, Vitals/bpm)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

# Give the socket time to stabilize before sending the first message
time.sleep(1)

monitor = HeartRateMonitor()
monitor.start_sensor()

# History for moving average (smoothing out noise)
bpm_history = []
spo2_history = []
HISTORY_SIZE = 15

# Send vitals regularly every 15 seconds
poslednje_slanje_vitals = 0
INTERVAL_SLANJA = 15.0

# Removed - alarm ON
# Returned - alarm OFF
prethodno_stanje_prsta = False

print("Publisher pokrenut: Pametni alarm (Instant ON/OFF)...")

try:
    while True:
        # Current timestamp (seconds)
        trenutno_vreme = time.time()

        # Raw readings from the monitor
        stvarni_bpm = monitor.bpm
        stvarni_spo2 = round(monitor.spo2, 2)
        ir_niz = monitor.plot_buffer

        prst_prisutan = monitor.finger

        # If there is no finger, reset everything
        if not prst_prisutan:
            prikaz_bpm = 0
            prikaz_spo2 = 0
            ir_niz = [0] * 200

            # Clear histories so the old average does not carry over
            bpm_history = []
            spo2_history = []
        else:
            # If there is a finger, store only valid values in history
            # The sensor may output 0 while it is stabilizing
            if stvarni_bpm > 0:
                bpm_history.append(stvarni_bpm)
                if len(bpm_history) > HISTORY_SIZE:
                    bpm_history.pop(0)

            if stvarni_spo2 > 0:
                spo2_history.append(stvarni_spo2)
                if len(spo2_history) > HISTORY_SIZE:
                    spo2_history.pop(0)

            # Moving average for a more stable display
            # If the history is empty, fall back to the raw value
            prikaz_bpm = round(np.mean(bpm_history), 1) if bpm_history else stvarni_bpm
            prikaz_spo2 = round(np.mean(spo2_history), 1) if spo2_history else stvarni_spo2

        # Send the graph every 0.1s
        podaci = {"niz": ir_niz}
        socket.send_string(f"Raw/ir {json.dumps(podaci)}")

        # Finger returned - OFF
        # Finger removed - ON
        upravo_izvadjen = (prethodno_stanje_prsta == True and prst_prisutan == False)
        upravo_vracen = (prethodno_stanje_prsta == False and prst_prisutan == True)
        proslo_vreme = (trenutno_vreme - poslednje_slanje_vitals >= INTERVAL_SLANJA)

        if proslo_vreme or upravo_izvadjen or upravo_vracen:
            # If the finger has been returned, send at least 0.1 (or the real BPM)        
            slanje_bpm = prikaz_bpm if prst_prisutan else 0
            if upravo_vracen and slanje_bpm == 0:
                # subBPM turns off the sound as soon as it receives anything > 0
                slanje_bpm = 0.1

            # BPM and SpO2 are sent on separate topics
            socket.send_string(f"Vitals/bpm {slanje_bpm}")
            socket.send_string(f"Vitals/spo2 {prikaz_spo2}")

            # Reset the last send timer
            poslednje_slanje_vitals = trenutno_vreme

            # Console log
            if upravo_izvadjen: print("ALARM: Prst izvadjen!")
            elif upravo_vracen: print("ALARM UGASEN: Prst vracen!")
            else: print(f"Redovni izvestaj: {slanje_bpm} BPM")

        prethodno_stanje_prsta = prst_prisutan
        time.sleep(0.1)

except KeyboardInterrupt:
    monitor.stop_sensor()