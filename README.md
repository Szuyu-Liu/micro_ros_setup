# 🔧 micro-ROS on Teensy with Raspberry pi Setup
## 📌 Overview
This documentation is for setting up micro-ROS a Teensy and connect it to ROS2 running on a Raspberry Pi.
Official tutorial: https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/

## Hardware and Software
- Raspberry pi 4 (Ubuntu 24.04)
- ROS2 - jazzy
- Teensy 4.0
- AppImageLauncher 

## 🔹 Core Steps
1. Install micro-ROS on Raspberry pi.
2. Install micro-ROS agent on Raspberry pi.
3. Download AppImageLauncher
4. Download Arduino IDE
5. Add Teensy to Arduino IDE 2.0
6. Add USB Permission
7. Add micro-ROS on Teensy and patch the micro-ROS environment on Arduino

## ① Install micro-ROS on Raspberry pi
- Install the ROS first, we use ROS2 jazzy. [Offical Tutorial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Install the micro-ROS agent
```
# Source the environment
source /opt/ros/jazzy/setup.bash

# Create a workspace and download tools for micro-ROS
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build and source micro-ROS tools
colcon build
source install/local_setup.bash
```
## ② Install micro-ROS agent on Raspberry pi
- Download micro-ROS agent packages
```
ros2 run micro_ros_setup create_agent_ws.sh
```
- Build agent and source
```
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
- Run for test
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

## ③ Download AppImageLauncher 
- Install dependencies for AppImage and then restart the system
```
sudo apt-get install zlib1g-dev libfuse-dev libfuse2t64
sudo apt install software-properties-common
sudo reboot
```
- Download the arm64 version for ubuntu24.04 on Raspberry pi: [appimagelauncher_2.2.0-travis995.0f91801.bionic_arm64.deb](https://github.com/TheAssassin/AppImageLauncher/releases/tag/v2.2.0)
- Navigate to the directory.
```
sudo dpkg -i appimagelauncher_2.2.0-travis995.0f91801.bionic_arm64.deb
```
## ④ Download Arduino IDE
- Download to the same directory with AppImageLauncher. [Linux_arm64_app_image.zip](https://github.com/koendv/arduino-ide-raspberrypi/releases)
- Double click the file, if the AppImageLauncher is installed successfully, the Arduino file can be run.
- Select Integrate & Run

## ⑤ Add Teensy to Arduino IDE 2.0

1. Add the packages for Teensy
- File &#8594; Preferences &#8594; Additional boards manager URLs
- add the link: https://www.pjrc.com/teensy/package_teensy_index.json
2. Install Teensy
- Tools &#8594; Board &#8594; Board Manager
- Search 'Teensy'. Download Teensy 1.59.0

## ⑥ Add USB Permission
```
# Navigate to the directory
cd /etc/udev/rules.d/

# Get the required rules
sudo wget https://www.pjrc.com/teensy/00-teensy.rules

# Reload and restart the udev
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

# Grant user permissions
sudo usermod -a -G dialout "your-username"
```
## ⑦ Add micro-ROS on Teensy and patch the micro-ROS environment on Arduino
- Install dependencies 
```
sudo apt-get install git
sudo apt-get install curl
```
- clone the micro-ROS library
```
cd ~/Arduino/libraries/
git clone https://github.com/micro-ROS/micro_ros_arduino.git
```
- Patch the Arduino environment for micro-ROS
```
cd /home/pi/.arduino15/packages/teensy/hardware/avr/1.59.0
curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
```
## Test with the example publisher
- On Arduino: File &#8594; Examples &#8594; micro_ros_arduino, select micro-ros_publisher
- Make sure the proper board and port is selected.
- Verified and Upload the code, reset of the Teensy board required
----
- On Raspberry pi
```
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
# Open a new terminal to see if there is micro_ros_arduino node
ros2 topic list
```
