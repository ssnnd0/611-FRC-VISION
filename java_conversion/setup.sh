#!/bin/bash

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install OpenCV dependencies
echo "Installing OpenCV dependencies..."
sudo apt-get install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev libpng-dev \
    libtiff-dev libatlas-base-dev gfortran python3-dev

# Install OpenCV
echo "Installing OpenCV..."
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install

# Install Java (if not already installed)
echo "Checking for Java installation..."
if ! command -v java &> /dev/null
then
    echo "Java not found. Installing OpenJDK..."
    sudo apt-get install -y openjdk-11-jdk
else
    echo "Java is already installed."
fi

# Install Maven (optional, if you want to manage dependencies)
echo "Installing Maven..."
sudo apt-get install -y maven

# Print instructions for importing files
echo ""
echo "Setup complete!"
echo "Please ensure you have the following files in your project directory:"
echo "1. yolov3.cfg - YOLO configuration file"
echo "2. yolov3.weights - YOLO weights file"
echo "3. coco.names - Class names file"
echo ""
echo "You can place your own detection file in the project directory as well."
echo "Make sure to update the ObjectDetection class to use your detection file."
