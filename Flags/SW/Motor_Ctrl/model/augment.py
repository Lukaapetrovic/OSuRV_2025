import os
import numpy as np
import librosa
import soundfile as sf
from tqdm import tqdm
from mfcc_utils import AudioAugmentor  

# ================= CONFIG =================
PROCESSED_FOLDER = "raw_data"
OUTPUT_FOLDER = "augmented_dataset"
SR = 22050
CHUNK_DURATION = 3.0 
TARGET_RMS = 0.1     
SUM_FILE = "sum.wav"  

chunk_len = int(CHUNK_DURATION * SR)

# ================= INIT =================
augmentor = AudioAugmentor(sr=SR)

# load sum.wav
sum_audio, _ = librosa.load(SUM_FILE, sr=SR)

def rms_normalize(y, target_rms=TARGET_RMS):
    rms = np.sqrt(np.mean(y**2))
    if rms > 0:
        y = y * (target_rms / rms)
    return y

def add_sum_noise(y, sum_audio):
    """
    Overlay the input audio with mic noise from sum_audio. If sum_audio is shorter than y, it will be looped to match the length of y.
    """
    if len(sum_audio) < len(y):
        repeats = int(np.ceil(len(y) / len(sum_audio)))
        sum_audio_extended = np.tile(sum_audio, repeats)[:len(y)]
    else:
        sum_audio_extended = sum_audio[:len(y)]
    
    y_noisy = y + sum_audio_extended
    y_noisy = np.clip(y_noisy, -1.0, 1.0)
    return y_noisy

# ================= PREPARE OUTPUT =================
if not os.path.exists(OUTPUT_FOLDER):
    os.makedirs(OUTPUT_FOLDER)

# ================= COLLECT FILES =================
all_files = []
for root, dirs, files in os.walk(PROCESSED_FOLDER):
    for f in files:
        if f.lower().endswith(".wav"):
            all_files.append(os.path.join(root, f))

print(f"[INFO] Found {len(all_files)} files")

# ================= PROCESS =================
for file_path in tqdm(all_files, desc="Processing all files"):
    # output folder based on original folder
    cls_out_path = os.path.join(OUTPUT_FOLDER, os.path.basename(os.path.dirname(file_path)))
    os.makedirs(cls_out_path, exist_ok=True)

    y, _ = librosa.load(file_path, sr=SR)
    num_chunks = len(y) // chunk_len
    f = os.path.basename(file_path)

    for i in range(num_chunks):
        start = i * chunk_len
        chunk = y[start:start+chunk_len]

        for step in [-2, -1, 1, 2]:
            chunk_aug = augmentor.pitch_shift(chunk.copy(), n_steps=step)
            chunk_aug = rms_normalize(chunk_aug)
            out_name = f"{os.path.splitext(f)[0]}_chunk{i}_pitch{step}.wav"
            out_path = os.path.join(cls_out_path, out_name)
            sf.write(out_path, chunk_aug, SR)


        for factor in [0.95, 1.05]:
            chunk_aug = augmentor.time_stretch(chunk.copy(), rate=factor)
            chunk_aug = rms_normalize(chunk_aug)
            out_name = f"{os.path.splitext(f)[0]}_chunk{i}_tempo{factor:.2f}.wav"
            out_path = os.path.join(cls_out_path, out_name)
            sf.write(out_path, chunk_aug, SR)

        for gain in [-2, 2]:
            chunk_aug = augmentor.change_gain(chunk.copy(), gain_db=gain)
            chunk_aug = rms_normalize(chunk_aug)
            out_name = f"{os.path.splitext(f)[0]}_chunk{i}_gain{gain}.wav"
            out_path = os.path.join(cls_out_path, out_name)
            sf.write(out_path, chunk_aug, SR)

        chunk_aug = add_sum_noise(chunk.copy(), sum_audio)
        chunk_aug = rms_normalize(chunk_aug)
        out_name = f"{os.path.splitext(f)[0]}_chunk{i}_sum.wav"
        out_path = os.path.join(cls_out_path, out_name)
        sf.write(out_path, chunk_aug, SR)

print("[INFO] Processing completed!")
