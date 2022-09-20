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
#include "../lib_yujinrobot_yrldriver/include/yujinrobot_yrldriver/yujinrobot_yrldriver.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#define DELAY_LONG  10
#define DELAY_SHORT 3     /* In sec, used for sleep(); */

using namespace std::chrono_literals;

class Yujin_Lidar : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr yrl_pub;
  OnSetParametersCallbackHandle::SharedPtr cb_handle;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_br;
  tf2::Quaternion quat;
  tf2::Transform transform;
  sensor_msgs::msg::PointCloud2 cloud;
  geometry_msgs::msg::TransformStamped trans_msg;
  float data_pkt_rate;

  std::shared_ptr<YujinRobotYrlDriver> instance;
  double systemTime;
  std::vector<float> buffer_x, buffer_y, buffer_z, buffer_i;
  const uint32_t point_size;

  std::vector<double> param_extrinsicTF;
  std::string param_IP;
  float param_minZ;
  float param_maxZ;
  float param_minY;
  float param_maxY;
  float param_minX;
  float param_maxX;
  float param_minRange;
  float param_maxRange;
  float param_minVertiAngle;
  float param_maxVertiAngle;
  float param_minHoriAngle;
  float param_maxHoriAngle;
  float param_filterLevel;
  int param_scanmode;

public:
  Yujin_Lidar();
  ~Yujin_Lidar();

private:
  void initParamSrv();
  void lidar_CB();
  void startDriver();
  void setTimerForLidarCB();
  void changeScanMode();
  rcl_interfaces::msg::SetParametersResult
    params_CB(const std::vector<rclcpp::Parameter> &);
};

Yujin_Lidar::Yujin_Lidar(): Node("yrl3_v2_ros2_node"), point_size(16)
{
  /* 1. Initialize ROS publisher related objects */
  initParamSrv();
  quat.setRPY(0, 0, 0);
  transform.setOrigin( tf2::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation(quat);

  /* 2. Create a publisher to periodically generate pointcloud message */
  yrl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("yrl_scan", 2);
  tf_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  /* 3. A callback handler for the parameter server */
  cb_handle = this->add_on_set_parameters_callback(
    std::bind(&Yujin_Lidar::params_CB, this, std::placeholders::_1));

  /* 4. Start the Lidar driver */
  startDriver();

  /* 5. A timer to invoke the publisher callback every 40ms (25 Hz) */
  if (instance) 
  {
    setTimerForLidarCB();
  }
}

Yujin_Lidar::~Yujin_Lidar()
{
  instance.reset();
}

void Yujin_Lidar::setTimerForLidarCB()
{
  timer = this->create_wall_timer(
    40ms, std::bind(&Yujin_Lidar::lidar_CB, this));
}

/* This method initialize the internal parameter variables. */
void Yujin_Lidar::initParamSrv()
{
  /* 1. Declare parameter lists */
  this->declare_parameter("lidar_ip", (std::string)"192.168.1.250");
  this->declare_parameter("min_z", (double)0);
  this->declare_parameter("max_z", (double)5.0f);
  this->declare_parameter("min_y", (double)-35.0f);
  this->declare_parameter("max_y", (double)35.0f);
  this->declare_parameter("min_x", (double)-35.0f);
  this->declare_parameter("max_x", (double)35.0f);
  this->declare_parameter("min_range", (double)0);
  this->declare_parameter("max_range", (double)35.0f);
  this->declare_parameter("min_vertical_angle", (double)-40);
  this->declare_parameter("max_vertical_angle", (double)40);
  this->declare_parameter("min_horizontal_angle", (double)-180);
  this->declare_parameter("max_horizontal_angle", (double)180);
  this->declare_parameter("filter_level", (double)30.0);
  this->declare_parameter("extrinsic_transform", std::vector<double>{ 0, 0, 0.07f, 0, 0, 0});
  this->declare_parameter("scan_mode", (int) 1);

  param_IP = this->get_parameter("lidar_ip").as_string();
  param_minZ = (float)this->get_parameter("min_z").as_double();
  param_maxZ = (float)this->get_parameter("max_z").as_double();
  param_minY = (float)this->get_parameter("min_y").as_double();
  param_maxY = (float)this->get_parameter("max_y").as_double();
  param_minX = (float)this->get_parameter("min_x").as_double();
  param_maxX = (float)this->get_parameter("max_x").as_double();
  param_minRange = (float)this->get_parameter("min_range").as_double();
  param_maxRange = (float)this->get_parameter("max_range").as_double();
  param_minVertiAngle = (float)this->get_parameter("min_vertical_angle").as_double();
  param_maxVertiAngle = (float)this->get_parameter("max_vertical_angle").as_double();
  param_minHoriAngle = (float)this->get_parameter("min_horizontal_angle").as_double();
  param_maxHoriAngle = (float)this->get_parameter("max_horizontal_angle").as_double();
  param_filterLevel = (float)this->get_parameter("filter_level").as_double();
  param_extrinsicTF = this->get_parameter("extrinsic_transform").as_double_array();
  param_scanmode = this->get_parameter("scan_mode").as_int();
}

/* It generates and starts the driver instance */
void Yujin_Lidar::startDriver()
{
  float x, y, z, rx, ry, rz;

  /* 1. Create the driver instance */
  if (!instance)
    instance = std::make_shared<YujinRobotYrlDriver>();
  else
  {
    RCLCPP_INFO(this->get_logger(), "ERR: Duplicated driver instances.");
    return;
  }

  /* 2. Set IP */
  instance->SetIPAddrParam(param_IP);

  /* 3. Start the driver */
  int ret = instance->ConnectTOYRL3V2();
  if (ret == -1)
  {
    RCLCPP_INFO(this->get_logger(), "ERROR: CANNOT START COMMUNICATION WITH LIDAR.\n");
    RCLCPP_INFO(this->get_logger(), "ERROR: CONNECT TO [IP:%s] FAILED. CHECK YOUR NETWORK CONNECTION.\n", param_IP.c_str());
    instance.reset();
    return;
  }
  instance->StartThreads();

  /* 4. Apply the lidar parameters */
  x = (float)param_extrinsicTF[0];
  y = (float)param_extrinsicTF[1];
  z = (float)param_extrinsicTF[2];
  rx = (float)param_extrinsicTF[3];
  ry = (float)param_extrinsicTF[4];
  rz = (float)param_extrinsicTF[5];
  instance->SetExtrinsicTransformMatParam ( x, y, z, rx, ry, rz );
  instance->SetMinZParam ( param_minZ );
  instance->SetMaxZParam ( param_maxZ );
  instance->SetMinYParam ( param_minY );
  instance->SetMaxYParam ( param_maxY );
  instance->SetMinXParam ( param_minX );
  instance->SetMaxXParam ( param_maxX );
  instance->SetMinRangeParam ( param_minRange );
  instance->SetMaxRangeParam ( param_maxRange );
  instance->SetMinVertiAngleParam ( Degree2Radian<double>()(param_minVertiAngle) );
  instance->SetMaxVertiAngleParam ( Degree2Radian<double>()(param_maxVertiAngle) );
  instance->SetMinHoriAngleParam ( Degree2Radian<double>()(param_minHoriAngle) );
  instance->SetMaxHoriAngleParam ( Degree2Radian<double>()(param_maxHoriAngle) );
  instance->SetNoiseFilterLevelParam(param_filterLevel);

  /* 5. After the parameter initialization, start getting data */
  instance->StartGettingDataStream();
}

rcl_interfaces::msg::SetParametersResult
    Yujin_Lidar::params_CB(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters)
  {
    /* DEBUG */
    //RCLCPP_INFO(this->get_logger(), "PARAM SET (name ): %s", param.get_name().c_str());
    //RCLCPP_INFO(this->get_logger(), "PARAM SET (type ): %s", param.get_type_name().c_str());
    //RCLCPP_INFO(this->get_logger(), "PARAM SET (value): %s", param.value_to_string().c_str());
    /* ----- */

    std::string pname = param.get_name();
    if (pname == "min_z")
    {
      param_minZ = (float)param.as_double();
      instance->SetMinZParam ( param_minZ );
    }
    else if (pname == "max_z")
    {
      param_maxZ = (float)param.as_double();
      instance->SetMaxZParam ( param_maxZ );
    }
    else if (pname == "min_y")
    {
      param_minY = (float)param.as_double();
      instance->SetMinYParam ( param_minY );
    }
    else if (pname == "max_y")
    {
      param_maxY = (float)param.as_double();
      instance->SetMaxYParam ( param_maxY );
    }
    else if (pname == "min_x")
    {
      param_minX = (float)param.as_double();
      instance->SetMinXParam ( param_minX );
    }
    else if (pname == "max_x")
    {
      param_maxX = (float)param.as_double();
      instance->SetMaxXParam ( param_maxX );
    }
    else if (pname == "min_range")
    {
      param_minRange = (float)param.as_double();
      instance->SetMinRangeParam ( param_minRange );
    }
    else if (pname == "max_range")
    {
      param_maxRange = (float)param.as_double();
      instance->SetMaxRangeParam ( param_maxRange );
    }
    else if (pname == "min_vertical_angle")
    {
      param_minVertiAngle = (float)param.as_double();
      instance->SetMinVertiAngleParam ( Degree2Radian<double>()(param_minVertiAngle) );
    }
    else if (pname == "max_vertical_angle")
    {
      param_maxVertiAngle = (float)param.as_double();
      instance->SetMaxVertiAngleParam ( Degree2Radian<double>()(param_maxVertiAngle) );
    }
    else if (pname == "min_horizontal_angle")
    {
      param_minHoriAngle = (float)param.as_double();
      instance->SetMinHoriAngleParam ( Degree2Radian<double>()(param_minHoriAngle) );
    }
    else if (pname == "max_horizontal_angle")
    {
      param_maxHoriAngle = (float)param.as_double();
      instance->SetMaxHoriAngleParam ( Degree2Radian<double>()(param_maxHoriAngle) );
    }
    else if (pname == "filter_level")
    {
      param_filterLevel = (float)param.as_double();
      instance->SetNoiseFilterLevelParam (param_filterLevel);
    }
    else if (pname == "extrinsic_transform")
    {
      float x,y,z,rx,ry,rz;
      param_extrinsicTF = param.as_double_array();
      x = (float)param_extrinsicTF[0];
      y = (float)param_extrinsicTF[1];
      z = (float)param_extrinsicTF[2];
      rx = (float)param_extrinsicTF[3];
      ry = (float)param_extrinsicTF[4];
      rz = (float)param_extrinsicTF[5];
      instance->SetExtrinsicTransformMatParam ( x, y, z, rx, ry, rz );
    }
    else if (pname == "scan_mode")
    {
      if ( (int) param.as_int() >= 1 && (int) param.as_int() <= 4)
      {
        param_scanmode = (int) param.as_int();
        changeScanMode();
      }
    }
  }

  return result;
}

/* This method will be invoked when a user changes 'scan_mode' parameter */
void Yujin_Lidar::changeScanMode()
{
  RCLCPP_INFO(this->get_logger(), "change LiDAR scanning mode");

  /* Set new mode of lidar, MODE: 1, 2, 3, 4 */
  if ( param_scanmode >= 1 && param_scanmode <= 4 )
  {
    /* 1. Stop getting data from lidar and Start setting of lidar */
    instance->StopGettingDataStreamAndSet();

    /* 2. Set scanning mode of the lidar */
    instance->UserChangeMode (param_scanmode);

    /* 3. start getting data from lidar */
    instance->StartGettingDataStream();
  }

  RCLCPP_INFO(this->get_logger(), "Scanning mode has been changed.");
}

void Yujin_Lidar::lidar_CB()
{
  /* For data packet rate benchmarking */
  //instance->GetDPR(data_pkt_rate);
  //std::cout << "DPR: " << data_pkt_rate << std::endl;
  /* ================================= */

  int ret = instance->GetCartesianOutputsWithIntensity(systemTime, buffer_i, buffer_x, buffer_y, buffer_z);
  if (ret == -1)
  {
    return;
  }

  auto size_buffer(static_cast<int>(buffer_x.size()));

  cloud.header.frame_id = std::string("yrl_scan_id");
  cloud.header.stamp = this->get_clock()->now();
  cloud.fields.resize(4);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1;

  cloud.data.resize(std::max(1, size_buffer) * point_size, 0x00);
  cloud.point_step = point_size;
  cloud.row_step = cloud.data.size();
  cloud.height = 1;
  cloud.width = cloud.row_step / point_size;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  auto ptr = cloud.data.data();
  for (int i(0); i < size_buffer; i++)
  {
    *(reinterpret_cast<float*>(ptr +  0)) = buffer_x[i];
    *(reinterpret_cast<float*>(ptr +  4)) = buffer_y[i];
    *(reinterpret_cast<float*>(ptr +  8)) = buffer_z[i];
    *(reinterpret_cast<float*>(ptr + 12)) = buffer_i[i];
    ptr += point_size;
  }
  yrl_pub->publish(cloud);

  buffer_x.clear();
  buffer_y.clear();
  buffer_z.clear();
  buffer_i.clear();

  trans_msg.child_frame_id = "yrl_scan_id";
  trans_msg.header.frame_id = "map";
  trans_msg.header.stamp = this->get_clock()->now();
  trans_msg.transform.translation.x = transform.getOrigin().getX();
  trans_msg.transform.translation.y = transform.getOrigin().getY();
  trans_msg.transform.translation.z = transform.getOrigin().getZ();
  trans_msg.transform.rotation = toMsg(transform.getRotation());
  // Send Transform
  tf_br->sendTransform(trans_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Yujin_Lidar>());
  rclcpp::shutdown();
  return 0;
}
