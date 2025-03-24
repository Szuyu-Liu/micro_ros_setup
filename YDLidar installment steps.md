# Steps to install YDLidar:
Prerequisite: ros2 installed, we use jazzy in this case. Iron should work as well.

## 1. Clone and Install YDLIDAR SDK
YDLidar-SDK is a dependency library of YDLidar

```
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## 2. Clone and Build YDLIDAR ROS 2 Package
```
source /opt/ros/jazzy/setup.bash
cd ~/ydlidar_ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ydlidar_ros2_ws
```
do:
```
colcon build --packages-select ydlidar_ros2_driver
```
This command will build ydlidar_ros2_driver.

Or:
```
colcon build --symlink-install
```
This command will build all the packages in the workspace.


An error may occur, if jazzy or newer version ros2 is used: 
```
no matching function for call to 'rclcpp::Node:: declare_parameter(const char [5])' node->declare_parameter("port");
```
The reason:
In ROS 2 Jazzy, you must provide the parameter name and a default value when using declare_parameter. This means that older code (meant for previous ROS 2 versions) will throw an error if the default value is missing.

The approach:

Use the library from Hartmut or change the source code like:
```
node->declare_parameter("port", str_optvalue);
```
## 3. Make sure YDLidar was installed successfully
```
ros2 pkg list | grep ydlidar
```
ydlidar_ros2_driver can be seen.

launch the node
```
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

ros2 topic list
ros2 topic echo /scan
```
Here you can see the output of the laser_frame.
```
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_client
```
It'll give a certain degree with a certain distance.

Visualization
```
rviz2
```

