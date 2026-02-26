# reson

Header-only C++ MFCC pipeline with Python bindings (pybind11).

## Modules

- `reson::core`: core data types (`Frame<N>`, `Spectre<N>`, common typedefs)
- `reson::dsp`: DSP building blocks (windowing, FFT, Mel filter bank, helpers)
- `reson::features`: higher-level feature extraction (MFCC pipeline)

## Generating the docs

### With CMake

- Configure: `cmake -S . -B build`
- Build docs: `cmake --build build --target docs`

HTML output will be generated into `build/doxygen/html/index.html`.

### Dependencies

- `doxygen` (required)
- `graphviz` (optional, for diagrams)
