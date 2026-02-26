import neurokit2 as nk
import numpy as np

class SignalProcessor:
    def __init__(self, sampling_rate=50):
        # Sampling rate of the incoming signal [Hz]
        self.sampling_rate = sampling_rate

    def filter_for_plot(self, ir_niz):
        try:
            # If the signal is flat, there is nothing to clear or normalize
            if max(ir_niz) == min(ir_niz):
                # Return zeros so the plot stays stable
                return [0.0] * len(ir_niz)
            
            # Convert input list to a NumPy for processing
            signal = np.array(ir_niz)

            # Clean the full PPG signal (noise/artifacts removal)
            cleaned = nk.ppg_clean(signal, sampling_rate=self.sampling_rate, method='elgendi')

            # Normalize so the plot always looks consistent
            norm_signal = (cleaned - np.min(cleaned)) / (np.max(cleaned) - np.min(cleaned))

            # Convert back to a plain Python list (for plotting)
            return norm_signal.tolist()

        except Exception:
            # If anything goes wrong, return an empty list
            return []