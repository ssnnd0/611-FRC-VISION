#!/bin/bash

# Function to print messages in a fancy way
print_message() {
    echo ""
    echo "========================================"
    echo "      611 Vision; Sandro Thornton"
    echo "========================================"
    echo ""
}

# Update package list
print_message "Updating package list..."
sudo apt-get update

# Install OpenCV dependencies
print_message "Installing OpenCV dependencies..."
sudo apt-get install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev libpng-dev \
    libtiff-dev libatlas-base-dev gfortran python3-dev

# Install OpenCV
print_message "Installing OpenCV..."
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install

# Install Java (OpenJDK 11)
print_message "Checking for Java installation..."
if ! command -v java &> /dev/null
then
    print_message "Java not found. Installing OpenJDK 11..."
    sudo apt-get install -y openjdk-11-jdk
else
    print_message "Java is already installed."
fi

# Verify Java installation
if java -version &> /dev/null; then
    print_message "Java installation verified."
else
    print_message "Java installation failed. Please check your system."
    exit 1
fi

# Install Maven (optional, if you want to manage dependencies)
print_message "Installing Maven..."
sudo apt-get install -y maven

# Install additional libraries for DNN module
print_message "Installing additional libraries for DNN module..."
sudo apt-get install -y libopencv-dnn-dev

# Print instructions for importing files
print_message "Setup complete!"
echo "Please ensure you have the following files in your project directory:"
echo "1. yolov3.cfg - YOLO configuration file"
echo "2. yolov3.weights - YOLO weights file"
echo "3. coco.names - Class names file"
echo ""
echo "You can place your own detection file in the project directory as well."
echo "Make sure to update the ObjectDetection class to use your detection file."
echo ""
echo "You can now compile and run your Java application!"
