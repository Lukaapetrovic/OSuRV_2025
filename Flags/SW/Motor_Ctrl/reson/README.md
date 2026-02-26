# reson

Header-only C++ MFCC pipeline with Python bindings (pybind11) for audio feature extraction.

## Goal

This project implements a modular MFCC (Mel-Frequency Cepstral Coefficients) pipeline. The idea is to transform a signal frame into a compact coefficient vector that can be used in downstream tasks (classification, clustering, detection).

The implementation is intentionally minimal and separated into stages (windowing, FFT, Mel, DCT) so each part is easy to understand and test.

## Features

- Header-only, template-based C++ implementation
- FFT + power spectrum
- Mel filter bank projection + optional normalization
- Log compression
- Orthonormal DCT-II
- Python bindings via pybind11
- Small C++ test executables and signal generators for validation

## Project structure

- `include/core/`
	- Core data types (`Frame<N>`, `Spectre<N>`) and shared constants/types
- `include/dsp/`
	- DSP steps: windowing, FFT, Mel filter bank, helpers (power spectrum, log compression, DCT)
- `include/features/`
	- High-level feature pipelines (MFCC)
- `bindings/`
	- pybind11 module exposing the C++ API to Python
- `tests/`
	- GoogleTest-based unit tests for FFT, windowing, Mel filters, and MFCC pipeline (11 tests total)

## MFCC pipeline overview

The main pipeline is implemented as `MFCCPipeline<N>` (header-only), where `N` is the input frame size.

Conceptual flow per call to `process(frame)`:

1. Windowing (Hann)
2. FFT
3. Power spectrum (typically use the positive spectrum: `N/2 + 1` bins)
4. Mel filter bank
5. Log compression
6. DCT (keep first `n_mfcc` coefficients)

User-controlled parameters:

- `sample_rate`: sampling rate (Hz)
- `n_mels`: number of Mel filters
- `n_fft`: FFT size (recommended: `n_fft == N`)
- `n_mfcc`: number of MFCC coefficients returned
- `fmin_hz`, `fmax_hz`: frequency range in Hz (`fmax_hz = -1` means Nyquist)

Limitations (by design):

- Processes a single frame (no framing/hop/overlap)
- No pre-emphasis, liftering, delta/delta-delta

## Build

### Dependencies

Option A (automatic):

- Run `setup.sh` (installs CMake, toolchain, Python dev headers, and pybind11)

Option B (manual):

- CMake >= 3.20
- A C++17 compiler
- Python3 + `python3-dev`
- pybind11

### Build commands

```bash
mkdir -p build
cmake -S . -B build
cmake --build build -j
```

Build outputs:

- test executables: `fft_test`, `window_test`, `pipeline_test`
- Python module: `reson*.so` (name depends on Python version/platform)

## Running tests

Tests are implemented with GoogleTest and automatically discovered by CTest during build.

### Run all tests

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

You should see all 11 tests pass:
- 6 tests in `fft_test` (CoreFrameSpectre, FFT, Helpers)
- 3 tests in `window_test` (Window)
- 2 tests in `pipeline_test` (MelFilterBank, MFCCPipeline)

### Useful CTest commands

```bash
# List all discovered tests
ctest --test-dir build -N

# Run a subset by regex match
ctest --test-dir build -R FFT

# Verbose output
ctest --test-dir build -V
```

### Run test executables directly with GoogleTest

```bash
# List all tests in a binary
./build/fft_test --gtest_list_tests

# Run all tests in a binary
./build/fft_test

# Run specific test suite
./build/fft_test --gtest_filter=FFT.*

# Run specific test case
./build/window_test --gtest_filter=Window.HannHasZeroEndpointsOnOnes
```

## Python usage

The easiest way is to run the example script; it adds `build/` to `sys.path`:

```bash
python3 example.py
```

Manual alternative:

```bash
export PYTHONPATH="$PWD/build:$PYTHONPATH"
python3 example.py
```

Quick API sketch:

- Create a frame: `reson.core.Frame512()`
- Run pipeline: `reson.features.MFCCPipeline512(sample_rate=16000, n_mels=40, n_fft=512, n_mfcc=13, fmin_hz=0, fmax_hz=-1)`
