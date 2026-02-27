cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0148 NEW)
project(reson LANGUAGES CXX)

# --- C++ standard ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # za shared library

# --- Pybind11 ---
find_package(pybind11 REQUIRED)

# --- Include directories ---
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- Source files ---
set(SRC_FILES
    bindings/python_module.cpp
)

# --- Build pybind11 module ---
pybind11_add_module(reson ${SRC_FILES})

# --- Optional: target include dirs (modern CMake) ---
target_include_directories(reson PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- GoogleTest setup ---
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

# --- Build tests ---
add_executable(fft_test tests/fft_test.cpp)
target_include_directories(fft_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(fft_test GTest::gtest_main)
gtest_discover_tests(fft_test)

add_executable(pipeline_test tests/pipeline_test.cpp)
target_include_directories(pipeline_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(pipeline_test GTest::gtest_main)
gtest_discover_tests(pipeline_test)

add_executable(window_test tests/window_test.cpp)
target_include_directories(window_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(window_test GTest::gtest_main)
gtest_discover_tests(window_test)

# --- Optional: Doxygen documentation ---
find_package(Doxygen QUIET)

if(DOXYGEN_FOUND)
    set(DOXYFILE_IN  ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    if(EXISTS ${DOXYFILE_IN})
        configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    else()
        message(STATUS "Doxygen config template not found at ${DOXYFILE_IN}")
    endif()
else()
    message(STATUS "Doxygen not found; 'docs' target will be unavailable")
endif()
