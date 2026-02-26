import os
import warnings
import logging

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

warnings.filterwarnings("ignore")
logging.getLogger("absl").setLevel(logging.ERROR)

import numpy as np
import librosa
import tensorflow as tf
from collections import Counter
from model.mfcc_utils import MFCCProcessor
from tqdm import tqdm
import pygame as pg
import socket
from pathlib import Path
import pathlib


import threading
import time

# ===== CONFIG =====
folder_path = Path("raw_data")
MODEL_PATH = "model/best_country_model.h5"
LABELS_PATH = "model/labels.npy"
SR = 22050
CHUNK_DURATION = 3.0
FRAMES_PER_CHUNK = 130

# Define the server IP address and port
SERVER_IP = "10.1.149.209"
SERVER_PORT = 5005
BUFFER_SIZE = 1024
MESSAGE = ""


def menu():
    print("=== Country Prediction Menu ===")
    print("\nChoose a song for prediction:")

    for i, file in enumerate(folder_path.rglob("*.wav")):
        print(f"{i+1}. {file.name}")
    choice = int(input("\nEnter the number of the song: "))
    if(choice < 1 or choice > len(list(folder_path.rglob("*.wav")))):
        selected_file = "exit"
    else:
        selected_file = list(folder_path.rglob("*.wav"))[choice-1]
    return selected_file

# ===== LOAD MODEL =====
model = tf.keras.models.load_model(MODEL_PATH)
labels = np.load(LABELS_PATH, allow_pickle=True).item()
inverse_labels = {v: k for k, v in labels.items()}

chunk_len = int(SR * CHUNK_DURATION)
mfcc_proc = MFCCProcessor(sample_rate=SR)



# ===== MFCC EXTRACT =====
def extract_features(y):
    frame_size = 512
    hop = 256
    frames = []

    for i in range(0, len(y) - frame_size, hop):
        frame = y[i:i+frame_size]
        mfcc = mfcc_proc.process_frame(frame)
        frames.append(mfcc)

    if len(frames) == 0:
        frames.append(np.zeros(13))

    mfcc = np.array(frames).T
    delta = librosa.feature.delta(mfcc)
    delta2 = librosa.feature.delta(mfcc, order=2)
    feat = np.stack([mfcc, delta, delta2], axis=1)
    feat = np.transpose(feat, (2,1,0))
    return feat

# ===== PREDICT SONG =====
def predict_song(path):
    y, _ = librosa.load(path, sr=SR)
    num_chunks = max(1, len(y) // chunk_len)
    predictions = []

    print(f"\nProcessing song ...")
    print(f"Total chunks: {num_chunks}\n")

    for i in tqdm(range(num_chunks), desc="Chunk prediction"):
        chunk = y[i*chunk_len:(i+1)*chunk_len]
        features = extract_features(chunk)

        # padding/trimming
        if features.shape[0] < FRAMES_PER_CHUNK:
            pad = FRAMES_PER_CHUNK - features.shape[0]
            features = np.pad(features, ((0,pad),(0,0),(0,0)))
        else:
            features = features[:FRAMES_PER_CHUNK]

        features = features.reshape(1, FRAMES_PER_CHUNK, 39, 1)
        pred = np.argmax(model.predict(features, verbose=0))
        predictions.append(pred)

    vote = Counter(predictions).most_common(1)[0][0]
    print(f"\nPredicted country: {inverse_labels[vote]}")
    return inverse_labels[vote]

# ===== RUN =====
if __name__ == "__main__":

    while(True):
        if(pg.mixer.get_init()):
            pg.mixer.music.stop()
        selected_file = menu()
        if selected_file == "exit":
            print("Exiting the program.")
            break
        pg.mixer.init()
        pg.mixer.music.load(selected_file)
        pg.mixer.music.play()    
        MESSAGE = predict_song(selected_file)
        MESSAGE = MESSAGE.strip()  # Remove any leading/trailing whitespace

    

        # Create a UDP socket at the client side
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Send a message to the server
            client_socket.sendto(MESSAGE.encode('utf-8'), (SERVER_IP, SERVER_PORT))
            print(f"Sent UDP packet to {SERVER_IP}:{SERVER_PORT}: {MESSAGE}")

            # Receive response from the server
            data, server_address = client_socket.recvfrom(BUFFER_SIZE)
            print(f"Received response from server: {data.decode('utf-8')}")



        except socket.error as e:
            print(f"Socket error: {e}")
            
        finally:
            client_socket.close()
            print("UDP Client closed")

