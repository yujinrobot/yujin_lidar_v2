# YRL3V2 LeGO-LOAM
LeGO-LOAM: https://github.com/RobustFieldAutonomyLab/LeGO-LOAM

Tested with ROS Noetic and YUJIN LiDAR YRL3V2

## Build YRL3V2 ROS Package
### Create a workspace
```
$ mkdir -p ~/catkin_ws/src/
$ cd ~/catkin_ws
$ catkin_make
```
### Environment setup for the worksapce
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
### Clone 'yujin_lidar_v2' repository
```
$ cd ~/Downloads
$ git clone https://github.com/yujinrobot/yujin_lidar_v2.git

Copy 'yujin_yrl_v2_package' in '~/Downloads/yujin_lidar_v2/driver_ros1_noetic_ubuntu2004' to '~/catkin_ws/src'
```
### Build the YRL3V2 ROS package
``` 
$ cd ~/catkin_ws
$ catkin_make
```

## Environment Set-up for YRL3V2 LeGO-LOAM
### Install dependencies 
```
Open a new terminal 

Install laser_assembler 
$ sudo apt install ros-noetic-laser-assembler
$ sudo apt install python-is-python3

Install gtsam for LeGO-LOAM
$ wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
$ cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
$ cd ~/Downloads/gtsam-4.0.0-alpha2/
$ mkdir build && cd build
$ cmake ..
$ sudo make install 

Install libparmetis
$ sudo apt-get install libparmetis-dev 
```
### Download 'LeGO-LOAM'
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
```
### Copy package
```
$ cd ~/catkin_ws/src/

Copy 'yrl_to_cloud' in 'yujin_lidar_v2/LeGO_LOAM_ros1_noetic_TestPackage_ubuntu2004' to '~/catkin_ws/src'
Replace 'LeGO-LOAM' in '~/catkin_ws/src/LeGO-LOAM' with 'yujin_lidar_v2/LeGO_LOAM_ros1_noetic_TestPackage_ubuntu2004/LeGO-LOAM'
```
### Modify yrl_pub.cpp
```
$ cd ~/catkin_ws/src/yujin_yrl_v2_package/src
$ gedit ./yrl_pub.cpp

Change code in line 308
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "yrl_cloud_id"));
=>
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "yrl_cloud_id"));
 
Save and build again
$ cd ~/catkin_ws
$ catkin_make
```
### Modify PCL
```
$ cd /usr/include/pcl-1.10/pcl/filters 
$ sudo gedit voxel_grid.h

In line 340 and 669,
Change 'Eigen::Index' to 'int' and save
```
### Modify utility.h
```
$ cd ~/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include
$ gedit utility.h

In line 57,
Change 'fileDirectory' to your directory for saving pcd files

ex) extern const string fileDirectory = "/home/tof-hjkim2/catkin_ws/";
```
### Build the YRL3V2 LeGO-LOAM
```
$ cd ~/catkin_ws/ 
$ catkin_make
```

## Map your environment
Default scanning mode we support is mode 2, so please change LiDAR's scanning mode to 2 through viewer before trying mapping.
When obtaining point cloud data for mapping, data collection should be carried out by moving 0.5 meters and stopping for 2~3 seconds, and so on.

If you want to use LiDAR scanning mode 1, 3 and 4, 
you should modify the value of 'max_clouds' parameter in ~/catkin_ws/src/yrl_to_cloud/launch/assemble.launch
and values of 'N_SCAN', 'ang_res_y', 'ang_bottom' in ~/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h

'max_clouds': As the vertical field of view increases, the buffer size must be increased. Because, to explore point clouds between consecutive frames for the full range of field of view, proportional amount of point clouds are needed.

'N_SCAN': This variable represents the number of channels for multichannel LiDAR. For YRL3V2 mode 2, 24 is the optimum value. You can also adjust this value in proportion to the size of the vertical field of view.

'ang_res_y': The resolution of the vertical axis, i.e. the y-axis. [Vertical FoV / (Number of channels-1)]

'ang_bottom': The lower vertical angle value for LiDAR's vertical field of view. For example, in case of mode 2, 'ang_bottom' is 5 because vertical field of view is from -5 degrees to 35 degrees.

All parameters mentioned above are carefully optimized for mode 2. If you would like to use other scanning mode, please refer to explanations above and change the variable values.

### Execute yrl_to_cloud package 
```
<Terminal 1>
$ cd ~/catkin_ws/src/yrl_to_cloud/src
$ chmod +x ./yrl2pc.py
$ roslaunch yrl_to_cloud assemble.launch

<Terminal 2>
$ cd ~/catkin_ws/
$ rosbag record /assemble_yrl

When your mapping is done, stop rosbag record.
```
### Run YRL3V2 LeGO-LOAM
```
<Terminal 1> 
$ roslaunch lego_loam run.launch

<Terminal 2>
$ cd ~/catkin_ws/
$ rosbag play < bagfile_name >.bag --clock --topic /assemble_yrl

<Terminal 3> 
$ cd ~/catkin_ws/
$ rosrun pcl_ros pointcloud_to_pcd input:=/laser_cloud_surround

Commands for terminal 3 is optional. If you want to get a map, run them, then you will get pcd files created (in your path set in utility.h) after LeGo-LOAM is finished. (ex. cornerMap.pcd, finalCloud.pcd, surfaceMap.pcd, trajectory.pcd and etc.)
```
