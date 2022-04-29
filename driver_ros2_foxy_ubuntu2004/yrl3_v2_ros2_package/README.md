Installing YRLROS2 driver 
=============

## Install ROS2 Foxy Fitzroy
### Add the ROS 2 apt repository
```
$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
### Install ROS2 Foxy Fitzroy
```
$ sudo apt update
$ sudo apt install ros-foxy-desktop
```
### Install ROS2 development tools
```
$ sudo apt update
$ sudo apt install -y python3-pip
$ sudo apt install -y build-essential
$ sudo apt install -y python3-colcon-common-extensions
$ pip3 install -U argcomplete
```
### Environment setup
```
$ echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
### Try example nodes
```
Open a new terminal and run 'talker'
$ ros2 run demo_nodes_cpp talker

Open another terminal and run 'listener'
ros2 run demo_nodes_py listener
```

## Build YRL3V2 ROS2 Package
### Create a workspace
```
$ mkdir -p ~/colcon_ws/src
$ cd ~/colcon_ws
$ colcon build --symlink-install
```
### Environment setup for the worksapce
```
$ echo ". ~/colcon_ws/install/local_setup.bash" >> ~/.bashrc
```
### Clone 'yujin_lidar_v2' repository
```
$ git clone https://github.com/yujinrobot/yujin_lidar_v2.git

Copy 'yrl3_v2_ros2_package' of 'yujin_lidar_v2/driver_ros2_foxy_ubuntu2004' into ~/colcon_ws/src
```
### Build the YRL3V2 ROS2 Package
``` 
Open a new terminal and build the package

$ cd ~/colcon_ws
$ colcon build
```

## How to use ROS2 and YRL3V2 ROS2 Package
### Package execution (roslaunch) : recommended
```
Launch files are located in '~/colcon_ws/src/yrl3_v2_ros2_package/launch'
1) yrl3_v2_ros2.launch.py (Python) 2) yrl3_v2_ros2.launch (XML)

Open a new terminal and run the launch file

$ ros2 launch < package_name > < launch_file_name >

# Example
$ ros2 launch yrl3_v2_ros2_package yrl3_v2_ros2.launch.py 
OR
$ ros2 launch yrl3_v2_ros2_package yrl3_v2_ros2.launch
```
