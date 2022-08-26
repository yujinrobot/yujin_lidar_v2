/*********************************************************************
*  Copyright (c) 2022, YujinRobot Corp.
*
*  Non-monifiable freely redistributable software(FRS)
*
*  - Redistribution. Redistribution and use in binary form, without modification,
*    are permitted provided that the following conditions are met:
*  - Redistributions must reproduce the above copyright notice and the following
*    disclaimer in the documentation and/or other materials provided with the distribution.
*  - Neither the name of YujinRobot Corporation nor the names of its suppliers may be used
*    to endorse or promote products derived from this software without specific prior written permission.
*
*  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OFOR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*********************************************************************/

/*********************************************************************   
  Copyright 2011-2017 Hauke Strasdat           
          2012-2017 Steven Lovegrove

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to
  deal in the Software without restriction, including without limitation the
  rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
  sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.
*********************************************************************/

#ifndef EXTRA_CODE_HPP
#define EXTRA_CODE_HPP

#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <math.h>
#endif

#include <Eigen/Geometry>
#include <shield_sophus/se3.hpp>

#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <cmath>

enum RawDataGraphType
{
  typeNone = 0,
  typeRisingWidthGraph,
  typeRangeGraph,
  typeHorizontalCountsGraph,
  typeVerticalCountsGraph
};

typedef struct LidarParam
{
  uint32_t bias_volt;
  uint32_t gain_volt;
  uint32_t ld_volt;
  uint32_t ld_pulse;
  uint32_t adjust_mode; 
  uint32_t safety_rotation_ms;
  uint32_t vertical_lower; 
  uint32_t vertical_upper;
  uint32_t vertical_speed;
  int32_t vertical_offset_lower;
  int32_t vertical_mode;
  uint32_t table_size; 
  int32_t* width_table1; 
  int32_t* width_table2; 
  int32_t* comp_table1;
  int32_t* comp_table2;

  uint32_t serial_number1;
  uint32_t serial_number2;
  uint32_t serial_number3;
  uint32_t ip_address;
  uint32_t model_no;
  uint32_t horizontal_speed;
  int32_t horizontal_offset;
  int32_t vertical_offset;
  uint32_t MAC1;
  uint32_t MAC2;
  uint32_t eth_major;
  uint32_t eth_minor;
  uint32_t eth_revision;
  uint32_t tdc_major;
  uint32_t tdc_minor;
  uint32_t tdc_revision;
  uint32_t factory_mode;
  uint32_t main_model_no;
} LidarParam;

typedef struct ValueGroup
{
  double system_time;
  unsigned int TimeStamp;
  std::vector<unsigned int> DataArr;
} ValueGroup;

class PointDataArray
{
public:
  PointDataArray ()
  : mSystemTime(0)
  {}

public:
  double mSystemTime;
  std::vector<float> mRangeArray;
  std::vector<float> mHorizontalAngleArray;
  std::vector<float> mVerticalAngleArray;
  std::vector<float> mRisingArray;
  std::vector<float> mIntensityArray;
  std::vector<float> mXCoordArray;
  std::vector<float> mYCoordArray;
  std::vector<float> mZCoordArray;
};

class LidarStatusData
{
public:
  float V_pd;
  float V_ld;
  float T_pd;
  float T_mcu;
  float V_target_pd;
  float rpm;
  float vertical_speed;
  float sample_rate;
  unsigned int origin_info;
  
  LidarStatusData ()
  : V_pd(0)
  , V_ld(0)
  , T_pd(0)
  , T_mcu(0)
  , V_target_pd(0)
  , rpm(0)
  , vertical_speed(0)
  , sample_rate(0)
  , origin_info(0)
  {}
  
  ~LidarStatusData (){}
};

template<typename Tr, int _degree180 = 180>
class Degree2Radian
{
public:
  template<typename Td>
  Tr operator() ( const Td & degree )
  {
    return static_cast<Tr>( degree )
            * static_cast<Tr>(3.14159265358979)
            / static_cast<Tr>(180.0);
  }
};

template<typename Td, int _degree180 = 180>
class Radian2Degree
{
public:
  template<typename Tr>
  Td operator() ( const Tr & radian )
  {
    return static_cast<Td>( radian * static_cast<Tr>(180.0) / static_cast<Tr>(3.14159265358979) );
  }
};

class Parameters
{
public:
  Parameters ()
  : mIPAddrParam("192.168.1.250")
  , mPortNumParam(1234)
  , mMinZParam( 0.0f )
  , mMaxZParam( 5.0f )
  , mMinYParam( -35.0f )
  , mMaxYParam( 35.0f )
  , mMinXParam( -35.0f )
  , mMaxXParam( 35.0f )
  , mMinRangeParam(0.0) /// 0.1
  , mMaxRangeParam(35.0) /// 25.0
  , mSensorCoordX(0)
  , mSensorCoordY(0)
  , mSensorCoordZ(0.07f) /// default sensor height from the bottom is 0.07m
  , mSensorCoordRX(0)
  , mSensorCoordRY(0)
  , mSensorCoordRZ(0) /// actual value * 10
  //, mHoriAngleOffsetParam( Degree2Radian<double>()( 0.0 ) )
  , mVertiAngleOffsetParam(0.0)
  , mOutputModeParam(1)
  , mVerticalModeParam(1)
  , mMaxVertiAngleParam( Degree2Radian<double>()( +40.0 ) ) /// +40
  , mMinVertiAngleParam( Degree2Radian<double>()( -40.0 ) ) /// -40
  , mMaxHoriAngleParam( Degree2Radian<double>()( +180.0 ) ) ///+180
  , mMinHoriAngleParam( Degree2Radian<double>()( -180.0 ) ) ///-180
  , mNoiseFilterLevelParam (30)
  {} 
  
  Eigen::Matrix3f ArrayToExtrinsics (float px, float py, float pz, float prx, float pry, float prz)
  {
    mSensorCoordX = px;
    mSensorCoordY = py;
    mSensorCoordZ = pz;
    mSensorCoordRX = prx;
    mSensorCoordRY = pry;
    mSensorCoordRZ = prz;

    float rz = Degree2Radian<float>()( mSensorCoordRZ );
    float ry = Degree2Radian<float>()( mSensorCoordRY );
    float rx = Degree2Radian<float>()( mSensorCoordRX );

    Eigen::Matrix3f R = (Eigen::AngleAxis<float> (rz, Eigen::Vector3f::UnitZ()) *                              
                        Eigen::AngleAxis<float> (ry, Eigen::Vector3f::UnitY()) *
                        Eigen::AngleAxis<float> (rx, Eigen::Vector3f::UnitX()) ).matrix();
    mExtrinsicTransformMatParam = R;

    return mExtrinsicTransformMatParam;
  }
  
public:
  std::string mIPAddrParam;
  unsigned short int mPortNumParam;

  float mMinZParam;
  float mMaxZParam;

  float mMinYParam;
  float mMaxYParam;

  float mMinXParam;
  float mMaxXParam;

  /// minimum and maximum of range
  float mMinRangeParam;
  float mMaxRangeParam;

  /// data pose  
  //Sophus::SE3f mExtrinsicTransformMatParam;
  Eigen::Matrix3f mExtrinsicTransformMatParam; 
  float mSensorCoordX;
  float mSensorCoordY;
  float mSensorCoordZ;
  float mSensorCoordRX;
  float mSensorCoordRY;
  float mSensorCoordRZ;
  
  ///----------< read from Lidar >----------
  //float mHoriAngleOffsetParam; /// offset for horizontal angle and vertical angle
  float mVertiAngleOffsetParam;
  int mOutputModeParam;
  int mVerticalModeParam;
  ///----------< read from Lidar >----------

  /// maximum and minimum of vertical angle
  float mMaxVertiAngleParam;
  float mMinVertiAngleParam;

  /// maximum and minimum horizontal angle
  float mMaxHoriAngleParam;
  float mMinHoriAngleParam;

  float mNoiseFilterLevelParam;
};

class StopWatch
{
public:
  StopWatch ()
  : _start(std::chrono::system_clock::now())
  {}

  void Start ()
  {
    _start = std::chrono::system_clock::now();
  }

  int GetTimeElapsed ()
  {
    _end = std::chrono::system_clock::now();
    std::chrono::milliseconds mill = std::chrono::duration_cast<std::chrono::milliseconds>(_end - _start);
    return mill.count();
  }

  int GetTimeElapsedInSec ()
  {
    _end = std::chrono::system_clock::now();
    std::chrono::seconds secc = std::chrono::duration_cast<std::chrono::seconds>(_end - _start);
    //              std::chrono::duration<double> secc = _end - _start;
    return secc.count();
  }

  double GetCurrentTimeInSeconds ()
  {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
  }

protected:
  std::chrono::time_point<std::chrono::system_clock> _start;
  std::chrono::time_point<std::chrono::system_clock> _end;
};

template<typename T, int _Size>
class MedianFilter
{
public:
  MedianFilter()
  : initialised(false), running_counter(0)
  {}

  T operator()(const T & value)
  {
    if (!initialised)
    {
      initialised = true;
      for (int i(0); i < _Size; i++)
      {
        buffer[i] = value;
        sort_buffer.push_back(value);
      }
      running_counter = 0;
    }

    buffer[running_counter++] = value;
    running_counter %= _Size;

    for (int i(0); i < _Size; i++)
    {
      sort_buffer[i] = buffer[i];
    }

    std::sort(sort_buffer.begin(), sort_buffer.end());
    last_value = sort_buffer[_Size / 2];
    return last_value;
  }

  T operator()(const T & value, int noLocalSample)
  {
    if (!initialised)
    {
      initialised = true;
      for (int i(0); i < _Size; i++)
      {
        buffer[i] = value;
        sort_buffer.push_back(value);
      }
      running_counter = 0;
    }

    buffer[running_counter++] = value;
    running_counter %= _Size;

    for (int i(0); i < _Size; i++)
    {
      sort_buffer[i] = buffer[i];
    }

    std::sort(sort_buffer.begin(), sort_buffer.end());

    last_value = 0;
    for (int k(_Size / 2 - noLocalSample); k <= (_Size / 2 + noLocalSample); k++)
    {
      last_value += sort_buffer[k];
    }
    last_value /= (2 * noLocalSample + 1);
    return last_value;
  }

  void Clear(const T & clearValue = T())
  {
    sort_buffer.resize( _Size );
    for (int i(0); i < _Size; i++)
    {
      buffer[i] = clearValue;
      sort_buffer[i] = clearValue;
    }

    running_counter = 0;
  }

  T getValue()
  {
    return last_value;
  }

protected:
  bool initialised;
  T buffer[_Size];
  std::vector<T> sort_buffer;
  int running_counter;
  T last_value;
};

template<typename T>
void Parse ( T & destination, unsigned char *& source )
{
  unsigned int no_size = sizeof( T );
  std::memcpy( &destination, source, no_size );
  ////Copies the values of no_size bytes from the location pointed to by source directly to the memory block pointed to by destination.
  ////The underlying type of the objects pointed to by both the source and destination pointers are irrelevant for this function;
  /// The result is a binary copy of the data.
  source += no_size;
}

#endif // EXTRA_CODE_HPP