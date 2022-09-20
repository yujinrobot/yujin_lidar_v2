Installing YRL3V2 ROS2 Driver 
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
$ ros2 run demo_nodes_py listener
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

Copy 'yrl3_v2_ros2_package' in 'yujin_lidar_v2/driver_ros2_foxy_ubuntu2004' to ~/colcon_ws/src
```
### Build the YRL3V2 ROS2 package
``` 
***copy 'lib_yujinrobot_yrldriver', you created in 'YRL3V2 Driver Package-Build YRL3V2 Driver Package' above, to '~/colcon_ws/src/yrl3_v2_ros2_package'
***every time you made changes in driver, you should apply changed driver library to ros package.
Open a new terminal and build the package

$ cd ~/colcon_ws
$ colcon build
```

## How to Use ROS2 and YRL3V2 ROS2 Package
### Package execution (ros2 launch) : recommended
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
### Package execution (ros2 run)
```
$ ros2 run < package_name > < node_name >

# Example
$ ros2 run yrl3_v2_ros2_package yrl3_v2_ros2_node

# For visualizing point data from yrl3_v2_ros2_node
$ rviz2 -d ~/colcon_ws/src/yrl3_v2_ros2_package/config/yrl3_v2_rviz2.rviz
```
### Parameter list of a running node
```
$ ros2 param list /< node_name >

# Example
$ ros2 param list /yrl3_v2_ros2_node
```
### Getting a parameter value in runtime
```
$ ros2 param get /< node_name > < parameter_name >

# Example
$ ros2 param get /yrl3_v2_ros2_node lidar_ip
```
### Setting a parameter value in runtime
```
$ ros2 param set /< node_name > < parameter_name > < parameter_value >

# Example
$ ros2 param set /yrl3_v2_ros2_node scan_mode 1
$ ros2 param set /yrl3_v2_ros2_node extrinsic_transform "[0.0, 0.0, 0.07, 0.0, 0.0, 0.0]"
```
### Loading parameters from a file to a running node
```
$ ros2 param load /< node_name > < path_to_yaml_file >

# Example
$ ros2 param load /yrl3_v2_ros2_node ~/colcon_ws/src/yrl3_v2_ros2_package/config/lidar_params.yaml
```
### Loading a parameter file when starting a node
```
$ ros2 run < package_name > < node_name > --ros-args --params-file < path_to_yaml_file >

# Example
$ ros2 run yrl3_v2_ros2_package yrl3_v2_ros2_node --ros-args --params-file ~/colcon_ws/src/yrl3_v2_ros2_package/config/lidar_params.yaml
```
### Setting parameters when starting a node
```
$ ros2 run < package_name > < node_name > --ros-args -p < parameter_name >:=< parameter_value >

# Example
$ ros2 run yrl3_v2_ros2_package yrl3_v2_ros2_node --ros-args -p lidar_ip:="192.168.1.250" -p scan_mode:=1 -p extrinsic_transform:="[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]"
```
