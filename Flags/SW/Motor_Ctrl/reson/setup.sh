#!/bin/bash

# Setup script for the reson project
# This script installs the necessary dependencies for building the C++ project with Python bindings

set -e  # Exit on any error

echo "Updating package list..."
sudo apt update

echo "Installing build dependencies..."
sudo apt install -y build-essential cmake python3 python3-dev python3-pip

echo "Installing pybind11..."
sudo apt install -y python3-pybind11

echo "Verifying installations..."
echo "CMake version:"
cmake --version
echo "Python version:"
python3 --version
echo "g++ version:"
g++ --version

echo "Setup complete! You can now build the project with:"
echo "mkdir build && cd build && cmake .. && make"