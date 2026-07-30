#!/bin/bash
# ==========================================
# Automated Setup Script for Raspberry Pi 5
# System: Cartridge Feeding + Robot + Vision
# ==========================================

set -e

echo "🚀 Starting System Setup for Raspberry Pi 5..."

# 1. Update system
echo "📦 Updating system packages..."
sudo apt-get update && sudo apt-get upgrade -y

# 2. Install ROS 2 Humble (if not installed)
if ! command -v ros2 &> /dev/null; then
    echo "⚙️ Installing ROS 2 Humble..."
    sudo apt-get install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt-get update && sudo apt-get install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions ros-dev-tools python3-rosdep
    
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
else
    echo "✅ ROS 2 is already installed."
fi

# 3. Install Python Dependencies
echo "🐍 Installing Python dependencies..."
sudo apt-get install -y python3-pip python3-venv
pip3 install pymodbus pydantic pyserial pyyaml

# 4. Install Qt5 for GUI
echo "🖼️ Installing Qt5 Dependencies..."
sudo apt-get install -y qtbase5-dev qtdeclarative5-dev qml-module-qtquick-controls2 qml-module-qtquick-layouts

# 5. HailoRT / Vision dependencies
echo "📷 Installing Vision Dependencies..."
sudo apt-get install -y libcamera-dev libopencv-dev python3-opencv v4l-utils
# Note: HailoRT PCIe driver usually requires specific apt repo setup or .deb install provided by Hailo/Raspberry Pi.
# Ensure `hailort` is installed.
sudo apt-get install -y hailort || echo "⚠️ Please install HailoRT driver manually if not available in apt."

# 6. Build Workspace
echo "🔨 Building ROS 2 Workspace..."
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "✅ Setup Complete! Please reboot the Raspberry Pi."
