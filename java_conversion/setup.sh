#!/bin/bash

# Function to print messages in a fancy way
print_message() {
    echo "
 .----------------.  .----------------.  .----------------.    
| .--------------. || .--------------. || .--------------. |   
| |    ______    | || |     __       | || |     __       | |   
| |  / ____  \   | || |    /  |      | || |    /  |      | |   
| |   '  ___) |  | || |   \`| |      | || |   \`| |      | |   
| |  | (____) |  | || |     | |      | || |     | |      | |   
| |   \______.'  | || |    _| |_     | || |    _| |_     | |   
| |              | || |   |_____|    | || |   |_____|    | |   
| '--------------' || '--------------' || '--------------' |   
 '----------------'  '----------------'  '----------------'    
 .-----------------. .----------------.  .----------------.  .----------------.  .----------------.  .-----------------. 
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
| |  ____  ____  | || |     __       | || |    _______   | || |     __       | || |     ___      | || |    _______   | |
| | |_  _||_  _| | || |    /  |      | || |   /  ___  |  | || |    /  |      | || |   .'   \`.   | || |   /  ___  |  | |
| |   \ \  / /   | || |   \`| |      | || |  |  (__ \_|  | || |   \`| |      | || |  /  .--.  \  | || |  |  (__ \_|  | |
| |    \ \/ /    | || |     | |      | || |   '.___\`-.  | || |     | |      | || | |  |    | |  | || |   '.___\`-.  | |
| |    _|  |_    | || |    _| |_     | || |  |\`\____) | | || |    _| |_     | || | \  \`--'  /  | || |  |\`\____) | | |
| |   |______|   | || |   |_____|    | || |  |_______.'  | || |   |_____|    | || |  \`.____.'   | || |  |_______.'  | |
| |              | || |              | || |              | || |              | || |              | || |              | |
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'  '----------------' 
MADE BY Sandro Thornton
"
}

# Update package list
print_message "Updating package list..."
sudo apt-get update

# Upgrade existing packages
print_message "Upgrading existing packages..."
sudo apt-get upgrade -y

# Install OpenCV dependencies
print_message "Installing OpenCV dependencies..."
sudo apt-get install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libjpeg-dev libpng-dev \
    libtiff-dev libatlas-base-dev gfortran python3-dev libv4l-dev

# Install OpenCV
print_message "Installing OpenCV..."
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_V4L=ON -D WITH_GTK=ON -D WITH_TBB=ON -D WITH_OPENGL=ON ..
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
echo ""
echo "For Arducam and other camera support, ensure your camera is connected and recognized by the system."
echo "You can test camera access using the following command:"
echo "  v4l2-ctl --list-devices"
echo "If your camera is listed, it should work with OpenCV."
