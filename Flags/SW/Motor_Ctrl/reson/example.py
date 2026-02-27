#!/usr/bin/env python3
"""
Example usage of the reson Python module.

Make sure the reson.cpython-312-x86_64-linux-gnu.so is in your Python path.
You can run this script from the build directory.
"""

import sys
import os
import math

# Add the build directory to the path if running from project root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'build'))

import reson

def main():
    print("Testing reson Python bindings")

    # Parameters
    sample_rate = 16000
    freq = 440  # 440 Hz sine wave
    amplitude = 1.0
    frame_size = 512  # Use 512-point frame

    # Generate sine wave
    frame = reson.core.Frame512()
    for i in range(frame.length()):
        frame[i] = amplitude * math.sin(2 * math.pi * freq * i / sample_rate)

    print(f"Generated {frame.length()} samples of {freq} Hz sine wave")
    print(f"First 5 samples: {[frame[i] for i in range(5)]}")

    # Create MFCC pipeline
    mfcc_pipeline = reson.features.MFCCPipeline512(
        sample_rate=sample_rate,
        n_mels=40,
        n_fft=512,
        n_mfcc=13
    )

    # Process the frame
    mfccs = mfcc_pipeline.process(frame)

    for i in range(len(mfccs)):
        print(f"MFCC[{i}]: {mfccs[i]} \n")
    
    print(f"Number of MFCC coefficients: {len(mfccs)}")

if __name__ == "__main__":
    main()