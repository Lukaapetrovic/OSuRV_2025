import zmq
import time
import pygame
from socketio import Client

# Audio initialization
pygame.mixer.init()
try:
    # Load the alarm sound made in Audacity
    alarm_zvuk = pygame.mixer.Sound("alarm.wav")
    print(" Ton za ravnu liniju (alarm.wav) uspesno ucitan.")
except:
    print(" Greska: alarm.wav nije pronadjen! Proveri da li je u istom folderu.")
    alarm_zvuk = None

# Keep the alarm silent until BPM > 0 at least once
puls_je_nekad_postojao = False

# Connections (SocketIO & ZMQ)
sio = Client()
try:
    sio.connect('http://localhost:5000')
    print(" Povezan na Flask server (subRaw.py).")
except:
    print(" Upozorenje: Server na portu 5000 nije dostupan.")

# ZMQ subscriber: receive BPM form the publisher
context = zmq.Context()
socket = context.socket(zmq.SUB)

socket.connect("tcp://10.1.150.45:5556")
socket.setsockopt_string(zmq.SUBSCRIBE, "Vitals/bpm")

print(" subBPM aktivan. Cekam prvi kontakt sa senzorom...")
print("-" * 50)

# Main loop
try:
    while True:
        # Receive message from ZMQ
        string = socket.recv_string()
        topic, bpm = string.split()
        bpm_float = float(bpm)

        # Smart alarm logic
        if bpm_float > 0:
            # First time BPM > 0 enable alarm from now on
            if not puls_je_nekad_postojao:
                puls_je_nekad_postojao = True
                print("\n>>> Senzor detektovao prst. Alarm je sada AKTIVAN.")
            
            # If we have a pulse, the alarm must be OFF
            pygame.mixer.stop()
            print(f" BPM: {bpm_float}", end="\r")

        elif bpm_float == 0:
            # Alarm soundz only if finger is previously detected (BPM > 0 at least once)
            if puls_je_nekad_postojao:
                # Play in a loop, only if it is not already playing
                if alarm_zvuk and not pygame.mixer.get_busy():
                    alarm_zvuk.play(-1)         # loop forever
                    print("\n!!! ALARM: FLATLINE (0 BPM) !!!")
            else:
                # If we have not detected a finger yet, ignore zero readings and keep silent
                print("Čekam prvi kontakt sa prstom...", end="\r")

        # Send data to the website
        # Forward BPM to the SocketIO server for web display
        try:
            sio.emit('new_bpm', {'value': bpm})
        except:
            pass

except KeyboardInterrupt:
    print("\n Gašenje subBPM skripte...")
finally:
    # Clean shutdown of audio
    pygame.mixer.stop()
    pygame.mixer.quit()