import numpy as np
import librosa
import librosa.effects
try:
    import reson  # type: ignore
except Exception:  # reson is an optional native extension
    reson = None

# ================= MFCC PROCESSOR =================
class MFCCProcessor:
    """
    MFCC Processor.

    Preferred backend: native `reson` (fast).
    Fallback backend: `librosa` (works everywhere).

    `process_frame()` returns MFCC vector of shape (n_mfcc,).
    """

    def __init__(self, sample_rate=22050, n_mels=40, n_fft=512, n_mfcc=13):
        self.sample_rate = sample_rate
        self.n_mfcc = n_mfcc
        self.n_mels = n_mels
        self.n_fft = n_fft
        self.mfcc_pipeline = None
        if reson is not None:
            self.mfcc_pipeline = reson.features.MFCCPipeline512(
                sample_rate=sample_rate,
                n_mels=n_mels,
                n_fft=n_fft,
                n_mfcc=n_mfcc,
            )

    def process_frame(self, frame):
        """
        Processing audio frame (512 samples) and returns MFCC (n_mfcc,).
        Padding/trunc if frame is not 512 samples.
        """
        frame_len = len(frame)
        if frame_len < 512:
            frame = np.pad(frame, (0, 512 - frame_len))
        elif frame_len > 512:
            frame = frame[:512]

        if self.mfcc_pipeline is not None:
            frame_obj = reson.core.Frame512()
            for i in range(512):
                frame_obj[i] = float(frame[i])

            mfccs = self.mfcc_pipeline.process(frame_obj)
            mfccs = np.asarray(mfccs, dtype=np.float32)
            return mfccs.reshape(-1)

        # librosa fallback: compute a single MFCC column for this 512-sample frame
        frame = np.asarray(frame, dtype=np.float32)
        mfcc = librosa.feature.mfcc(
            y=frame,
            sr=self.sample_rate,
            n_mfcc=self.n_mfcc,
            n_fft=self.n_fft,
            hop_length=512,
            n_mels=self.n_mels,
            center=False,
        )
        return np.asarray(mfcc[:, 0], dtype=np.float32)

# ================= AUDIO AUGMENTOR =================
class AudioAugmentor:
    def __init__(self, sr=22050):
        self.sr = sr

    def time_stretch(self, y, rate=1.0):
        D = librosa.stft(y)
        D_stretch = librosa.phase_vocoder(D, rate=rate)
        y_stretch = librosa.istft(D_stretch)
        return y_stretch

    def pitch_shift(self, y, n_steps=0):
        return librosa.effects.pitch_shift(y, sr=self.sr, n_steps=n_steps)

    def add_background_noise(self, y, noise_level=0.005):
        noise = np.random.randn(len(y)) * noise_level
        return np.clip(y + noise, -1.0, 1.0)

    def change_gain(self, y, gain_db=0.0):
        gain = 10**(gain_db / 20)
        return np.clip(y * gain, -1.0, 1.0)
