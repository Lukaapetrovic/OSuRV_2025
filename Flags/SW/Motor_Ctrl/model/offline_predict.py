import numpy as np
import librosa
import tensorflow as tf
from collections import Counter
from mfcc_utils import MFCCProcessor
from tqdm import tqdm

# ===== CONFIG =====
MODEL_PATH = "best_country_model.h5"
LABELS_PATH = "labels.npy"
SR = 22050
CHUNK_DURATION = 3.0
FRAMES_PER_CHUNK = 130

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

    print(f"\nProcessing song: {path}")
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

# ===== RUN =====
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Offline country prediction from audio.")
    parser.add_argument("path", type=str, help="Path to song for prediction")
    args = parser.parse_args()
    predict_song(args.path)
