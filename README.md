# micro_ros_setup

download the arduio-ide:
https://github.com/koendv/arduino-ide-raspberrypi/releases

download appimagelauncher
https://github.com/TheAssassin/AppImageLauncher/releases/tag/v2.2.0
appimagelauncher_2.2.0-travis995.0f91801.bionic_arm64.deb 
arm64 for raspberry Pi

sudo dpkg -i appimagelauncher_2.2.0-travis995.0f91801.bionic_arm64.deb

tutorial:
https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/

cd /home/pi/.arduino15/packages/teensy/hardware/avr/1.59.0

curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
