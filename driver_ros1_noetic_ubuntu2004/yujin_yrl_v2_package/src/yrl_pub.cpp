/*********************************************************************
*  Copyright (c) 2022, YujinRobot Corp.
*  
*  Hyeon Jeong Kim, hjkim2@yujinrobot.com
*
*  Software License Agreement (BSD License)
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  - Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above
*  copyright notice, this list of conditions and the following
*  disclaimer in the documentation and/or other materials provided
*  with the distribution.
*  - Neither the name of {copyright_holder} nor the names of its
*  contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "../../lib_yujinrobot_yrldriver/include/yujinrobot_yrldriver/yujinrobot_yrldriver.hpp"
#include <iostream>
#include <dlfcn.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
  std::string IPAddress;
  float SensorX;
  float SensorY;
  float SensorZ;
  float SensorRX;
  float SensorRY;
  float SensorRZ;
  float MinX;
  float MaxX;
  float MinY;
  float MaxY;
  float MinZ;
  float MaxZ;
  float MinRange;
  float MaxRange;
  float MinVertiAngle;
  float MaxVertiAngle;
  float MinHoriAngle;
  float MaxHoriAngle;
  float NoiseFilterLevel;

  /// initialize a node
  ros::init(argc, argv, "yrl_pub");

  /// NodeHandle
  ros::NodeHandle nh("~");
  nh.param<std::string>("/yrl_pub/ip_address", IPAddress, "192.168.1.250");
  nh.param<float>("/yrl_pub/sensor_x", SensorX, 0.0);
  nh.param<float>("/yrl_pub/sensor_y", SensorY, 0.0);
  nh.param<float>("/yrl_pub/sensor_z", SensorZ, 0.07);
  nh.param<float>("/yrl_pub/sensor_rx", SensorRX, 0);
  nh.param<float>("/yrl_pub/sensor_ry", SensorRY, 0);
  nh.param<float>("/yrl_pub/sensor_rz", SensorRZ, 0);
  nh.param<float>("/yrl_pub/min_x", MinX, 0);
  nh.param<float>("/yrl_pub/max_x", MaxX, 0);
  nh.param<float>("/yrl_pub/min_y", MinY, 0);
  nh.param<float>("/yrl_pub/max_y", MaxY, 0);
  nh.param<float>("/yrl_pub/min_z", MinZ, 0);
  nh.param<float>("/yrl_pub/max_z", MaxZ, 0);
  nh.param<float>("/yrl_pub/min_range", MinRange, 0);
  nh.param<float>("/yrl_pub/max_range", MaxRange, 0);
  nh.param<float>("/yrl_pub/min_vertical_angle", MinVertiAngle, 0);
  nh.param<float>("/yrl_pub/max_vertical_angle", MaxVertiAngle, 0);
  nh.param<float>("/yrl_pub/min_horizontal_angle", MinHoriAngle, 0);
  nh.param<float>("/yrl_pub/max_horizontal_angle", MaxHoriAngle, 0);
  nh.param<float>("/yrl_pub/noise_filter_level", NoiseFilterLevel, 0);

  /// Publisher yrl_pub
  ros::Publisher yrl_pub = nh.advertise<sensor_msgs::PointCloud2>("yrl_cloud", 2);

  ros::Rate loop_rate(25);

  //== 1. CREATE DRIVER INSTANCE ===========================================
  YujinRobotYrlDriver* instance = new YujinRobotYrlDriver();
  //========================================================================
  
  //== 2. SET IP ===========================================================
  // THIS MUST BE SET BEFOR CALLING Start().
  // THIS WILL BE USED TO CONNECT LIDAR
  instance->SetIPAddrParam(IPAddress);
  //========================================================================

  //== 3. START DRIVER =====================================================
  int ret = instance->ConnectTOYRL3V2();
  if (ret == -1)
  {
    std::string IpAddress = instance->GetIPAddrParam();
    int PortNumber = instance->GetPortNumParam ();
    LOGPRINT(main, YRL_LOG_USER, ("CANNOT START COMMUNICATION WITH LIDAR.\n"));
    LOGPRINT(main, YRL_LOG_USER, ("CONNECT TO [IP:%s PORT:%d] FAILED. CHECK YOUR NETWORK CONNECTION.\n", IpAddress.c_str(), PortNumber));
    delete instance;
    return -1;
  }
  
  instance->StartThreads();
  //========================================================================

  //== 4. APPLY LiDAR POSE =================================================
  // APPLY POSITION AND ORIENTATION OF LiDAR
  // YOU CAN USE GET/SET FUNCTION OF LiDAR POSE
  //
  // GetExtrinsicTransformParam (float &x, float &y, float &z, float &rx, float &ry, float &rz)
  // SetExtrinsicTransformMatParam (const float x, const float y, const float z, const float rx, const float ry, const float rz)
  //
  // x, y, z ARE POSITION OF LiDAR
  // rx, ry, rz ARE ORIENTATION OF LiDAR

  instance->SetExtrinsicTransformMatParam ( SensorX, SensorY, SensorZ, SensorRX, SensorRY, SensorRZ );
  instance->GetExtrinsicTransformParam (SensorX, SensorY, SensorZ, SensorRX, SensorRY, SensorRZ);
  std::cout << "SensorX : " << SensorX << std::endl;
  std::cout << "SensorY : " << SensorY << std::endl;
  std::cout << "SensorZ : " << SensorZ << std::endl;
  std::cout << "SensorRX : " << SensorRX << std::endl;
  std::cout << "SensorRY : " << SensorRY << std::endl;
  std::cout << "SensorRZ : " << SensorRZ << std::endl;
  //========================================================================

  //== 5. OTHER SET PARAMETER FUNCTIONS ====================================
  // SetMinZParam (const float z_min)
  // SetMaxZParam (const float z_max)
  // SetMinYParam (const float y_min)
  // SetMaxYParam (const float y_max)
  // SetMinXParam (const float x_min)
  // SetMaxXParam (const float x_max)
  // SetMinRangeParam (const float range_min)
  // SetMaxRangeParam (const float range_max)
  // SetMaxVertiAngleParam (const float verti_angle_max)
  // SetMinVertiAngleParam (const float verti_angle_min)
  // SetMaxHoriAngleParam (const float hori_angle_max)
  // SetMinHoriAngleParam (const float hori_angle_min)
  // SetNoiseFilterLevelParam (const float filter_level)

  instance->SetMinXParam ( MinX );
  instance->SetMaxXParam ( MaxX );
  instance->SetMinYParam (MinY);
  instance->SetMaxYParam ( MaxY);
  instance->SetMinZParam ( MinZ );
  instance->SetMaxZParam ( MaxZ );
  instance->SetMinRangeParam (MinRange);
  instance->SetMaxRangeParam (MaxRange);
  instance->SetMinVertiAngleParam ( Degree2Radian<double>()( MinVertiAngle ) );
  instance->SetMaxVertiAngleParam ( Degree2Radian<double>()( MaxVertiAngle ) );
  instance->SetMinHoriAngleParam ( Degree2Radian<double>()( MinHoriAngle ) );
  instance->SetMaxHoriAngleParam ( Degree2Radian<double>()( MaxHoriAngle ) );
  instance->SetNoiseFilterLevelParam (NoiseFilterLevel);
  //========================================================================

  //== 6. OTHER GET PARAMETER FUNCTIONS ====================================
  // GetIPAddrParam()
  // GetPortNumParam ()
  // GetMinZParam ()
  // GetMaxZParam ()
  // GetMinYParam ()
  // GetMaxYParam ()
  // GetMinXParam ()
  // GetMaxXParam ()
  // GetMinRangeParam ()
  // GetMaxRangeParam ()
  // GetMaxVertiAngleParam ()
  // GetMinVertiAngleParam ()
  // GetMaxHoriAngleParam ()
  // GetMinHoriAngleParam ()
  // GetNoiseFilterLevelParam ()

  std::cout << "inputIpAddress : " << instance->GetIPAddrParam() << std::endl;
  std::cout << "PortNumber : " << instance->GetPortNumParam () << std::endl;
  std::cout << "MinX : " << instance->GetMinXParam() << std::endl;
  std::cout << "MaxX : " << instance->GetMaxXParam() << std::endl;
  std::cout << "MinY : " << instance->GetMinYParam() << std::endl;
  std::cout << "MaxY : " << instance->GetMaxYParam() << std::endl;
  std::cout << "MinZ : " << instance->GetMinZParam() << std::endl;
  std::cout << "MaxZ : " << instance->GetMaxZParam() << std::endl;
  std::cout << "MinRange : " << instance->GetMinRangeParam() << std::endl;
  std::cout << "MaxRange : " << instance->GetMaxRangeParam() << std::endl;
  std::cout << "MinVertiAngle : " << instance->GetMinVertiAngleParam() << std::endl;
  std::cout << "MaxVertiAngle : " << instance->GetMaxVertiAngleParam() << std::endl;
  std::cout << "MinHoriAngle : " << instance->GetMinHoriAngleParam() << std::endl;
  std::cout << "MaxHoriAngle : " << instance->GetMaxHoriAngleParam() << std::endl;
  std::cout << "NoiseFilterLevel : " << instance->GetNoiseFilterLevelParam() << std::endl;
  //========================================================================

  //== 7. START GETTING SENSOR DATA ========================================
  instance->StartGettingDataStream();
  //
  // YOU CAN GET SW DATA PACKET RATE THROUGH GetDPR()
  // void GetDPR(float &dpr)
  //
  // THERE ARE 2 OUTPUT FUNCTIONS.
  // YOU SHOULD USE ONLY ONE OF THEM.
  //
  // 1. int GetCartesianOutputsWithIntensity( double _SystemTime,
  //                                     std::vector <float>& _IntensityArray,
  //                                     std::vector <float>& _XCoordArray,
  //                                     std::vector <float>& _YCoordArray,
  //                                     std::vector <float>& _ZCoordArray);
  //
  // 2. int GetSphericalOutputsWithIntensity( double _SystemTime,
  //                                     std::vector <float>& _IntensityArray,
  //                                     std::vector <float>& _RangeArray,
  //                                     std::vector <float>& _HorizontalAngleArray,
  //                                     std::vector <float>& _VerticalAngleArray);
  //========================================================================

  const uint32_t point_size = 16;

  /// tf
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation(q);

  /// Buffers for getting output data
  double systemTime;
  std::vector<float> buffer_x, buffer_y, buffer_z, buffer_i;
  float data_packet_rate(0);

  while(ros::ok())
  {
    int ret = instance->GetCartesianOutputsWithIntensity (systemTime, buffer_i, buffer_x, buffer_y, buffer_z);
    if (ret == -1)
    {
      // std::cout << "empty queue\n";
      continue;
    }

    int size_buffer(static_cast<int>(buffer_x.size()));
    if(size_buffer == 0)
    {
      continue;
    }

    instance->GetDPR(data_packet_rate);
    std::cout << "Data_Packet_Rate : " << data_packet_rate << " points per sec"<<std::endl;

    /// Publish a group of point clouds
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = std::string("yrl_cloud_id");
    cloud.header.stamp = ros::Time::now();

    cloud.fields.resize(4);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    cloud.data.resize(std::max(1, size_buffer) * point_size, 0x00);
    cloud.point_step = point_size;
    cloud.row_step = cloud.data.size();
    cloud.height = 1;
    cloud.width = cloud.row_step / point_size;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    uint8_t *ptr = cloud.data.data();
    for (int i(0); i < size_buffer; i++)
    {
      *(reinterpret_cast<float*>(ptr +  0)) = buffer_x[i];
      *(reinterpret_cast<float*>(ptr +  4)) = buffer_y[i];
      *(reinterpret_cast<float*>(ptr +  8)) = buffer_z[i];
      *(reinterpret_cast<float*>(ptr + 12)) = buffer_i[i];
      ptr += point_size;
    }

    yrl_pub.publish(cloud);
    buffer_x.clear();
    buffer_y.clear();
    buffer_z.clear();
    buffer_i.clear();

    /// Send Transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "yrl_cloud_id"));
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete instance;
  return 0;
}
