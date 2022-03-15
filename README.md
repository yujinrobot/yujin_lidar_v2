# Yujin LiDAR V2
Official Website: 
http://lidar.yujinrobot.com/
https://yujinrobot.com/autonomous-mobility-solutions/components/lidar/

## About Yujin LiDAR

YRL series LiDAR is designed to detect objects, measure distances from surroundings and collect data as point clouds. Yujin LiDAR is an optimized solution for indoor mapping, navigation, localization and other applications in a variety of industries including robotics, safety and security.

## YRL3V2 Scanning
![](cafe2.jpeg)
![](cafe.gif)
![](4F.gif)

## ROS1 Package

- ROS Version: Noetic
- Maintainer Status: Developed
- License: BSD
![](ros1driverRviz.gif)

## Supported Hardware
- YRL3V2-05 (3D, 5m)
- YRL3V2-10 (3D, 10m)
- YRL3V2-25 (3D, 25m)

## ROS Package Installation

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone https://
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
rospack profile
```

## ROS API
- Package Name: yujin_yrl_package
- Node Name: yrl_pub
- Publisher Name : yrl_pub
- Topic Name : yrl_cloud
### Parameters
YRL ROS driver imports YRL Linux driver. To get and set parameters of YRL ROS driver, please use APIs explained in the manual.

## QUICK START
```bash
/// 1st terminal
roscore

/// 2nd terminal 1st option
roslaunch yujin_yrl_v2_package yujin_yrl_v2.launch

/// 2nd terminal 2nd option
rosrun yujin_yrl_v2_package yrl_pub
rostopic echo /yrl_pub/yrl_cloud
rosrun rviz rviz
```
## Additional Software
### Viewer
- Ubuntu 20.04 is required.
#### For Linux: Dependency Installation
```bash
sudo apt-get install qt5-default
```
#### For Linux: Quick Start For Viewer
```bash
sudo -H ./Yujin_Lidar_Viewer.sh
```
