#!/bin/bash

# Update system
sudo apt update
sudo apt full-upgrade -y  # Recommended for a clean install

# Install Python and pip
sudo apt install -y python3 python3-pip

# Create virtual environment (highly recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install core libraries
pip3 install tensorflow onnx numpy pynetworktables

# Install camera libraries
pip3 install picamera[array]  # For Raspberry Pi camera modules
pip3 install opencv-python     # For general camera support (including USB cameras)

# Install GPIO library
pip3 install RPi.GPIO

# Install USB library (if needed for specific USB device control)
pip3 install pyusb

# Upgrade pip (optional)
python3 -m pip install --upgrade pip

echo "Installation complete. Activate the virtual environment:"
echo "source .venv/bin/activate"
