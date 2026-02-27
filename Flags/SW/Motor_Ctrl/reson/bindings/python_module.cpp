#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>

#include "../include/core/frame.hpp"
#include "../include/core/spectre.hpp"
#include "../include/dsp/fft.hpp"
#include "../include/dsp/window.hpp"
#include "../include/dsp/mel.hpp"

#include "../include/features/mfcc_pipeline.hpp"

namespace py = pybind11;

// Macros to simplify binding
#define BIND_ARRAY_CLASS(module, cls, name, value_type) \
  py::class_<cls>(module, name) \
      .def(py::init<>()) \
      .def("__getitem__", [](const cls& obj, size_t i) { \
          if (i >= obj.length()) throw py::index_error(); \
          return obj[i]; \
      }) \
      .def("__setitem__", [](cls& obj, size_t i, value_type v) { \
          if (i >= obj.length()) throw py::index_error(); \
          obj[i] = v; \
      }) \
      .def("length", &cls::length)

#define BIND_WINDOW_CLASS(module, cls, name) \
  py::class_<cls>(module, name) \
      .def(py::init<reson::dsp::WindowType>()) \
      .def("apply_window", &cls::apply_window)

#define BIND_FFT_CLASS(module, cls, name) \
  py::class_<cls>(module, name) \
      .def(py::init<>()) \
      .def("process", &cls::process)

#define BIND_MFCC_PIPELINE(module, cls, name) \
  py::class_<cls>(module, name) \
      .def(py::init<int, int, int, int, int, int>(), py::arg("sample_rate"), py::arg("n_mels"), py::arg("n_fft"), py::arg("n_mfcc"), py::arg("fmin_hz")=0, py::arg("fmax_hz")=-1) \
      .def("process", &cls::process)


PYBIND11_MODULE(reson, m) {
  auto dsp = m.def_submodule("dsp", "Digital Signal Processing utilities");
  auto core = m.def_submodule("core", "Core data structures");
  auto features = m.def_submodule("features", "Feature extraction pipelines");

  // Bind enums
  py::enum_<reson::dsp::WindowType>(dsp, "WindowType")
      .value("Hann", reson::dsp::WindowType::Hann)
      .value("Hamming", reson::dsp::WindowType::Hamming);

  // Bind Frame<128>
  BIND_ARRAY_CLASS(core, reson::core::Frame<128>, "Frame128", float);
  
  //Bind Spectre<128>
  BIND_ARRAY_CLASS(core, reson::core::Spectre<128>, "Spectre128", std::complex<float>);
  

  //Bind Window<128>
  BIND_WINDOW_CLASS(dsp, reson::dsp::Window<128>, "Window128");
  
  //Bind FFT<128>
  BIND_FFT_CLASS(dsp, reson::dsp::FFT<128>, "FFT128");

  // Bind Frame<256>
  BIND_ARRAY_CLASS(core, reson::core::Frame<256>, "Frame256", float);
  
  //Bind Spectre<256>
  BIND_ARRAY_CLASS(core, reson::core::Spectre<256>, "Spectre256", std::complex<float>);
  

  //Bind Window<256>
  BIND_WINDOW_CLASS(dsp, reson::dsp::Window<256>, "Window256");
  
  //Bind FFT<256>
  BIND_FFT_CLASS(dsp, reson::dsp::FFT<256>, "FFT256");

  // Bind Frame<512>
  BIND_ARRAY_CLASS(core, reson::core::Frame<512>, "Frame512", float);
  
  //Bind Spectre<512>
  BIND_ARRAY_CLASS(core, reson::core::Spectre<512>, "Spectre512", std::complex<float>);
  

  //Bind Window<512>
  BIND_WINDOW_CLASS(dsp, reson::dsp::Window<512>, "Window512");
  
  //Bind FFT<512>
  BIND_FFT_CLASS(dsp, reson::dsp::FFT<512>, "FFT512");

  // Bind Frame<1024>
  BIND_ARRAY_CLASS(core, reson::core::Frame<1024>, "Frame1024", float);
  
  //Bind Spectre<1024>
  BIND_ARRAY_CLASS(core, reson::core::Spectre<1024>, "Spectre1024", std::complex<float>);
  

  //Bind Window<1024>
  BIND_WINDOW_CLASS(dsp, reson::dsp::Window<1024>, "Window1024");
  
  //Bind FFT<1024>
  BIND_FFT_CLASS(dsp, reson::dsp::FFT<1024>, "FFT1024");

  // Bind MelFilterBank
  py::class_<reson::dsp::MelFilterBank>(dsp, "MelFilterBank")
      .def(py::init<int, int, int, float, float, bool>(), py::arg("sample_rate"), py::arg("n_fft"), py::arg("n_mels"), py::arg("fmin_hz")=0.0f, py::arg("fmax_hz")=-1.0f, py::arg("normalize_by_sum")=true)
      .def("apply", &reson::dsp::MelFilterBank::apply)
      .def("get_filterbank", &reson::dsp::MelFilterBank::get_filterbank);

  // Bind MFCCPipeline<128>
  BIND_MFCC_PIPELINE(features, MFCCPipeline<128>, "MFCCPipeline128");

  // Bind MFCCPipeline<256>
  BIND_MFCC_PIPELINE(features, MFCCPipeline<256>, "MFCCPipeline256");

  // Bind MFCCPipeline<512>
  BIND_MFCC_PIPELINE(features, MFCCPipeline<512>, "MFCCPipeline512");

  // Bind MFCCPipeline<1024>
  BIND_MFCC_PIPELINE(features, MFCCPipeline<1024>, "MFCCPipeline1024");
  

}
