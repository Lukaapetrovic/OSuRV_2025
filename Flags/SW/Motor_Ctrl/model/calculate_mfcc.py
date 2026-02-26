import os
import numpy as np
import librosa
from tqdm import tqdm
from mfcc_utils import MFCCProcessor

# ================= CONFIG =================
DATASET_FOLDER = "augmented_dataset"
SR = 22050
FRAME_SIZE = 512  # sample frame size for MFCC extraction
# ================= INIT =================
mfcc_processor = MFCCProcessor(sample_rate=SR, n_mfcc=13)

X_list = []
y_list = []
labels = {}  # mapping folder -> integer

label_counter = 0

# ================= COLLECT FILES =================
all_files = []
for root, dirs, files in os.walk(DATASET_FOLDER):
    for f in files:
        if f.lower().endswith(".wav"):
            all_files.append(os.path.join(root, f))

print(f"[INFO] Found {len(all_files)} files")

# ================= PROCESS FILES =================
for file_path in tqdm(all_files, desc="Processing files"):
    label_name = os.path.basename(os.path.dirname(file_path))
    
    # mapping label to int
    if label_name not in labels:
        labels[label_name] = label_counter
        label_counter += 1
    
    y_label = labels[label_name]

    # load audio
    y_audio, _ = librosa.load(file_path, sr=SR)

    # cut into frames of 512 samples
    num_frames = len(y_audio) // FRAME_SIZE
    for i in tqdm(range(num_frames), desc=f"Frames {os.path.basename(file_path)}", leave=False):
        frame = y_audio[i*FRAME_SIZE:(i+1)*FRAME_SIZE]
        
        mfcc_feat = mfcc_processor.process_frame(frame)  # shape = (3*n_mfcc,)
        X_list.append(mfcc_feat)
        y_list.append(y_label)

# convert to numpy array
X = np.stack(X_list)  # shape: (num_frames_total, 3*n_mfcc)
y = np.array(y_list)  # shape: (num_frames_total,)

print(f"[INFO] X shape: {X.shape}, y shape: {y.shape}")
print(f"[INFO] Labels mapping: {labels}")

# opcionalno: sacuvaj kao .npz fajl
np.savez("mfcc_dataset.npz", X=X, y=y, labels=labels)
