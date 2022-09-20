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
//#include "pch.h"
#include "../../include/yujinrobot_yrldriver/yujinrobot_yrldriver.hpp"

YujinRobotYrlDriver::YujinRobotYrlDriver()
: mbRunThread(false)
, mRecordPreHoriAngleCnt(0)
, mDPR(0)
, useRawDataGraph(false)
, mGraphType(typeNone)
, UDPErrorCase(0)
, cntCommThread(0)
, cntPrcsThread(0)
, cntFinalOutput(0)
, mNumContinuousReadingFailures(0)
{
  mParams = new Parameters();
  mLidarStatus = new LidarStatusData();
  mNI = std::make_shared< NetworkInterface >();
    
  mLidarParam = {};
  mLidarParam.width_table1 = nullptr;
  mLidarParam.width_table2 = nullptr;
  mLidarParam.comp_table1 = nullptr;
  mLidarParam.comp_table2 = nullptr;

  mLidarWidthTable = nullptr;
  mLidarCompTable = nullptr;

  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_TRACE, ("constructor called\n"));
}

YujinRobotYrlDriver::~YujinRobotYrlDriver()
{
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_TRACE, ("distructor called\n"));
  removeThreads();
  
  free(mLidarWidthTable);
  free(mLidarCompTable);
  
  mNI.reset();
  //mNI = nullptr;

  delete mLidarStatus;
  delete mParams;
}

void YujinRobotYrlDriver::StartThreads()
{
  if (!mbRunThread)
  {
    mbRunThread = true;
    mFirstThread = std::thread(&YujinRobotYrlDriver::threadedFunction1, this);
    mSecondThread = std::thread(&YujinRobotYrlDriver::threadedFunction2, this);
  }
}

void YujinRobotYrlDriver::removeThreads()
{
  if (mbRunThread)
  {
    mbRunThread = false;

    mCVInputData.notify_all();

    if (mFirstThread.joinable())
    {
      mFirstThread.join();
      LOGPRINT(YujinRobotYrlDriver, YRL_LOG_TRACE, ("mFirstThread joined\n"));
    }

    if (mSecondThread.joinable())
    {
      mSecondThread.join();
      LOGPRINT(YujinRobotYrlDriver, YRL_LOG_TRACE, ("mSecondThread joined\n"));
    }
  }
}

int YujinRobotYrlDriver::StartTCP()
{
  return mNI->ConnectTCP();
}

void YujinRobotYrlDriver::StopTCP()
{
  mNI->CloseTCP();
}

bool YujinRobotYrlDriver::IsTCPConnected()
{
  return mNI->TCPConnected;
}

bool YujinRobotYrlDriver::ReconnectTCP()
{
  if (!mNI->mTCPSocket.Init(mParams->mIPAddrParam, mParams->mPortNumParam))
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LOG_ERROR, ("FAILED TO RECONNECT TCP COMM\n"));
    return false;
  }

  return true;
}

void YujinRobotYrlDriver::RecoveryNetwork()
{
  removeThreads();
  deleteNetworkInterface();

#ifdef _WIN32
  timeBeginPeriod(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  timeEndPeriod(1);
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
#endif

  createNetworkInterface();
  StartThreads();
}

void YujinRobotYrlDriver::deleteNetworkInterface()
{
  mNI = nullptr;
}

void YujinRobotYrlDriver::createNetworkInterface()
{
  mNI = std::make_shared< NetworkInterface >();
}

int YujinRobotYrlDriver::ConnectTOYRL3V2()
{
  int result = StartTCP();
  if (result == -1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LOG_ERROR, ("FAILED TO START TCP COMM.\n"));
    return -1;
  }

#ifdef _WIN32
  timeBeginPeriod(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  timeEndPeriod(1);
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
#endif

  FWCMD(1, 14);
  return 1;
}

void YujinRobotYrlDriver::DisconnectTOYRL3V2()
{
  StopTCP();

  #ifdef _WIN32
  timeBeginPeriod(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  timeEndPeriod(1);
#else
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
#endif
}

void YujinRobotYrlDriver::UserChangeMode (int mode)
{
  FWSetYRLVerticalMode(mode);
  FWCMD(1, 9);
  FWCMD(1, 14);
  FWGetYRLVerticalMode();
  FWGetYRLVerticalSpeed();
  FWGetYRLVerticalAngleLower();
  FWGetYRLVerticalAngleUpper();
}

void YujinRobotYrlDriver::UserChangeIP(std::string new_ip)
{
  std::stringstream s(new_ip);
  int a,b,c,d;
  int ip_a, ip_b, ip_c, ip_d;
  char ch;
  s >> a >> ch >> b >> ch >> c >> ch >> d;
  FWSetYRLIPAddress(a, b, c, d);
  FWCMD(1, 12);
  FWGetYRLIPAddress(ip_a, ip_b, ip_c, ip_d);
}

void YujinRobotYrlDriver::StartGettingDataStream()
{
  FWCMD(1, 13);
  mNI->doCommunicationMode(RECV_DATA_MODE, 0, 0, 0);
}

void YujinRobotYrlDriver::StopGettingDataStreamAndSet ()
{
  FWCMD(1, 6);
  FWCMD(1, 14);
}

int YujinRobotYrlDriver::Start()
{
  //==================================================================================================
  // Start()
  //
  // Connect TCP socket to server
  // Start threads
  //
  // return -1: Failed to connect
  // return 0: normal case
  //
  // Before calling Start(), IP address of LiDAR you want to connect to must be set in advance.
  // At the very first time, working mode is set to sending command mode.
  // getScanningData() will not be running before the working mode is changed to receiving data mode.
  //==================================================================================================

  int result = StartTCP();
  if (result == -1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LOG_ERROR, ("FAILED TO START TCP COMM.\n"));
    return -1;
  }

  StartThreads();

  return 0;
}

void YujinRobotYrlDriver::threadedFunction1()
{
  while (mbRunThread)
  {
    cntCommThread++;

    if (!mNI->mbStopGettingScanData)
    {
      getScanningData();
    }
    else
    {
#ifdef _WIN32
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      timeEndPeriod(1);
#else
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
#endif
    }
  }
}

void YujinRobotYrlDriver::threadedFunction2()
{
  while (1)
  {
    std::shared_ptr< ValueGroup > valueGroup;
    {
      std::unique_lock<std::mutex> lock(mInputDataLocker);
      mCVInputData.wait(lock, [this]{return !mInputDataDeque.empty() || !mbRunThread;});
      if (!mbRunThread)
      {
          return;
      }

      int sizeInputDataDeque (mInputDataDeque.size());
      if (sizeInputDataDeque >= 60) /// optimization for low computational power environment
      {
        LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("clear mInputDataDeque :  %d\n", sizeInputDataDeque));
        LOGPRINT(YujinRobotYrlDriver, YRL_LOG_WARN, ("THERE IS A DELAY IN REAL-TIME INPUT DATA PROCESSING. PLEASE CHECK THE SYSTEM PERFORMANCE.\n"));
        mInputDataDeque.clear();

        continue;
      }
      else if (sizeInputDataDeque >= 30) /// 30 data processing = data for 1 rotation
      {
        LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("popfront mInputDataDeque :  %d\n", sizeInputDataDeque));
        mInputDataDeque.pop_front();
      }

      valueGroup = mInputDataDeque.front();
      mInputDataDeque.pop_front();
    }

    dataProcessing(valueGroup);
  }
}

void YujinRobotYrlDriver::getScanningData()
{ 
  //==================================================================================================
  // getScanningData()
  //
  // Only when 'mbStopGettingScanData' is false, it works. Otherwise, return.
  // This sends UDP message and receive UDP message from server.
  // Parse the received message and push it to input queue
  //==================================================================================================

  mNI->mUDPRecvBuffer.assign(4096 * 4, 0);
  int sizeRecvM = mNI->mUDPSocket.ReadWithTimeout( mNI->mUDPRecvBuffer.data(), mNI->mUDPRecvBuffer.size(), 100 );
  double time_now = StopWatch().GetCurrentTimeInSeconds();

  UDPErrorCase = sizeRecvM;
  //wrong case
  if ( sizeRecvM <= 0 )//in case of -1, -2, -3
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LOG_ERROR, ("DID NOT GET ANY DATA\n"));
    if ( ++mNumContinuousReadingFailures > 10 )
    {
      mNumContinuousReadingFailures = 0;
      LOGPRINT(YujinRobotYrlDriver, YRL_LOG_ERROR, ("COMMUNICATION FAILURE. PLEASE CHECK THE CONNECTION.\n"));
    }
  }
  else
  {       
    if (sizeRecvM != 1467)
    {
      LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("sizeRecvM: %d\n", sizeRecvM));
    }
    
    //mNI->printReceivedUDPMessage();

    std::shared_ptr< ValueGroup > valueGroup = std::make_shared< ValueGroup >();
    if (seeDataStructure( mNI->mUDPRecvBuffer.data(), sizeRecvM, valueGroup->TimeStamp, valueGroup->DataArr ) > 0)
    {
      valueGroup->system_time = time_now;

      {
        std::lock_guard<std::mutex> lock(mInputDataLocker);
        mInputDataDeque.push_back(valueGroup);
      }
      mCVInputData.notify_all();
    }
  }
}

int YujinRobotYrlDriver::seeDataStructure( unsigned char * ptr, unsigned int numData, unsigned int& time_stamp, std::vector< unsigned int > & values )
{
  //==================================================================================================
  // seeDataStructure()
  //
  // Parse message and get header and data
  //
  // return -1: same frame id
  // return 1: normal case
  //==================================================================================================

  unsigned int headerSignaure;        //4byte //not used
  unsigned short int fwVersion;       //2byte //not used
  unsigned char hwVersion;            //1byte //not used
  unsigned int timeStamp;             //4byte //not used
  unsigned char descriptor;           //1byte//not used
  unsigned int frameID;               //4byte total:16

  //copy data from ptr to each variables
  Parse( headerSignaure, ptr );
  Parse( fwVersion, ptr );
  Parse( hwVersion, ptr );
  Parse( timeStamp, ptr );
  Parse( descriptor, ptr );
  Parse( frameID, ptr );

  static unsigned int preFrameID = 0;
  if( preFrameID == frameID ) //if a new data packet has same id, skip it
  {
    return -1;
  }
  preFrameID = frameID;

  time_stamp = timeStamp;

  unsigned char statusID(0);          //1byte
  unsigned int statusData(0);         //4byte
  Parse( statusID, ptr );
  Parse( statusData, ptr );

  // update lidar status
  // Status data are sent one by one in order(0~8) and this sending sequence is repeated.
  switch( statusID )
  {
    case 0: 
      mLidarStatus->V_pd = static_cast<float>( statusData ) * 1e-3f;
      break;
    case 1:
      mLidarStatus->V_ld = static_cast<float>( statusData ) * 1e-3f;
      break;
    case 2: 
      mLidarStatus->T_pd = static_cast<float>( statusData ) * 1e-3f; 
      break;
    case 3: 
      mLidarStatus->T_mcu = static_cast<float>( statusData ) * 1e-3f;
      break;
    case 4: 
      mLidarStatus->V_target_pd = static_cast<float>( statusData ) * 1e-3f; 
      break;
    case 5: 
      mLidarStatus->rpm = static_cast<float>( statusData );
      break;
    case 6: 
      mLidarStatus->vertical_speed = static_cast<float>( statusData );
      break;
    case 7: 
      mLidarStatus->sample_rate = static_cast<float>( statusData ) * 1e-3f; 
      break;
    case 8: 
      mLidarStatus->origin_info = static_cast<int>( statusData );
      break;
    default:
      break;
  }

  // parsing error code
  int compare_no(0xffff & (mLidarStatus->origin_info >> 8));
  int binary_buffer[16]={0};
  int i = 0;
  while (compare_no > 0)
  {
    binary_buffer[i] = compare_no % 2;
    compare_no = compare_no / 2;
    i++;
  }

  if (binary_buffer[0] == 1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("ERROR A: POWER ERROR\n"));
  }
  if (binary_buffer[1] == 1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("ERROR B: HORIZONTAL ERROR\n"));
  }
  if (binary_buffer[2] == 1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("ERROR C: VERTICAL ERROR\n"));
  }
  if (binary_buffer[4] == 1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("ERROR D: SENSOR COMMNUNICATION ERROR\n"));
  }
  if (binary_buffer[5] == 1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("ERROR E: TEMPERATURE EXCEPTION ERROR\n"));
  }
  if (binary_buffer[12] == 1)
  {
    LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("ERROR F: USER COMMUNICATION ERROR\n"));
  }

  for ( int j(0); j < 241; j++ )//241: the number of points. Each point has 6 byte data.
  {
    //save data to 'values': horizontal angle, verticl angle, leading time, width time
    if (mParams->mOutputModeParam == 0) /// RAW SIGNAL MODE
    {
      unsigned short int d0, d2;
      unsigned char d1, d3;

      Parse( d0, ptr );
      Parse( d1, ptr );
      Parse( d2, ptr );
      Parse( d3, ptr );

      unsigned int angle_horizontal = 0;
      unsigned int time_leading     = d0 | (((unsigned int)d1 << 16) & 0xFF0000);
      unsigned int time_width       = d2 | (((unsigned int)d3 << 16) & 0xFF0000);
      unsigned int angle_vertical   = 0;

      values.push_back( angle_horizontal );
      values.push_back( angle_vertical );
      values.push_back( time_leading );
      values.push_back( time_width );
    }
    else /// NORMAL RANGING MODE
    {
      unsigned short int d0, d1, d2;

      Parse( d0, ptr );
      Parse( d1, ptr );
      Parse( d2, ptr );

      unsigned int angle_horizontal = (d0 & 0xFFF) << 7;
      unsigned int time_leading = d1;
      unsigned int time_width = ((((d0 >> 8) & 0xF0) | ((d2 >> 12) & 0xF)) << 8) & 0xFF00;
      unsigned int angle_vertical = d2 & 0xFFF;

      values.push_back( angle_horizontal );
      values.push_back( angle_vertical );
      values.push_back( time_leading );
      values.push_back( time_width );
    }
  }

  return 1;
}

void YujinRobotYrlDriver::dataProcessing(std::shared_ptr < ValueGroup > &ptr)
{
  //==================================================================================================
  // dataProcessing()
  //
  // When input data queue is not empty, get data from the queue. Otherwise, return.
  // Process data and get result arrays:
  // mRangeArray, mVerticalAngleArray, mHorizontalAngleArray, mIntensityArray
  // , mRisingArray, mXCoordArray, mYCoordArray, mZCoordArray
  // Put result array data into output data queue

  // return -1: same frame id
  // return 1: normal case
  //==================================================================================================
  cntPrcsThread++;

  std::vector<unsigned int> values(ptr->DataArr);
  unsigned int timeStamp (ptr->TimeStamp);
  double sysTime (ptr->system_time);

  /// DPR TEST
  static StopWatch timer;
  static float no_dpr;
  no_dpr += values.size()/4;//241
  if (timer.GetTimeElapsed() >= 1000)
  {
    mDPR = no_dpr;
    //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("Driver no_dpr : %d\n", no_dpr));
    no_dpr = 0;
    timer.Start();
  }

  unsigned int maxTimeValue(0);
  if (mParams->mOutputModeParam == 0)/// RAW SIGNAL MODE
  {
    maxTimeValue = 600000;
  }
  else/// NORMAL RANGING MODE
  {
    maxTimeValue = 0xFFFF;
  }

  for ( int i(0); i < values.size(); i += 4 )
  {
    // <horizontalCount>
    // : 0 ~ 500,000

    // <verticalCount>
    // count: 4096
    // value(verticalCount): 0 ~ 4095
    // angle range: -22.5 ~ 22.5 (deg)
    // va = 45/4096 * (verticalCount + 1) - 22.5
    //    = 22.5/2048 * (verticalCount - 2047)

    int horizontalCount = values[i+0];
    int verticalCount   = values[i+1];
    unsigned int t1_r   = values[i+2];
    unsigned int t1_f   = values[i+2] + values[i+3];
    
    // filtering abnormal raw data
    if (mParams->mOutputModeParam == 0)//RAW SIGNAL MODE
    {
      if ( t1_r > maxTimeValue || t1_f > maxTimeValue || t1_r > t1_f || t1_r < 1000 || t1_f < 1000)
      {
        t1_r = 0;
        t1_f = 0;
      }
    }
    else//NORMAL RANGING MODE
    {
      if ( t1_r > maxTimeValue )
      {
        t1_r = 0;
        t1_f = 0;
      }
    }

    if ( mHAngles.size() > 20 )
    {
      // for specifying the situation when counting is dropped to bottom to catch one rotation
      if ( horizontalCount < 10000
        && mRecordPreHoriAngleCnt > horizontalCount)
      {
        if ( mRecordPreHoriAngleCnt < 490000 )
        {
          LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("v2 lidar; abnormal horizontal angle: %d --> %d\n", mRecordPreHoriAngleCnt, horizontalCount));
          mRanges0.clear();
          mHAngles.clear();
          mVAngles.clear();
          mRisings0.clear();
          mIntensities0.clear();
          mRecordPreHoriAngleCnt = horizontalCount;
          continue;
        }

        // // measure time to take 100 times horizontal rotations
        // double current_time_scan = StopWatch().GetCurrentTimeInSeconds();
        // static double time_scan(current_time_scan);
        // static int no_rotations(0);
        // if( ++no_rotations >= 100 )
        // {
        //     no_rotations = 0;
        //     double scan_interval = current_time_scan - time_scan;
        //     time_scan = current_time_scan;
        //     //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("fps = %lf\n", 100.0 / scan_interval));
        // }

        // Here, mRecordPreHoriAngleCnt is considered the total number of counts.
        double usec_to_horizontal_angle = (2.0 * M_PI) / static_cast<double>(mRecordPreHoriAngleCnt);

        // Convert horizontal angles from counts to radians
        // And shift range to keep angle range: -PI ~ +PI
        for ( int j(0); j < mHAngles.size(); j++ )
        {
          float ta = mHAngles[j] * usec_to_horizontal_angle;
          
          //shift angle range if it needed
          if ( ta > M_PI ) ta -= 2*M_PI;
          else if ( ta < -M_PI ) ta += 2*M_PI;

          /// optimization for faster computation
          // Apply horizontal angle offset
          if (mParams->mSensorCoordRZ && !mParams->mSensorCoordRX && !mParams->mSensorCoordRY)
          {
            ta += Degree2Radian<float>()( mParams->mSensorCoordRZ );
          }
          
          //shift angle range if it needed
          if ( ta > M_PI ) ta -= 2*M_PI;
          else if ( ta < -M_PI ) ta += 2*M_PI;

          mHAngles[j] = ta;//final horizontal angle

          // Apply vertical angle offset
          mVAngles[j] += Degree2Radian<double>()(mParams->mVertiAngleOffsetParam);//final vertical angle
        }

        // NOISE FILTERING
        int oi = 1;
        std::vector<float> hits(mHAngles.size(), 1);

        /// optimization for faster computation
        double filter_level = std::cos(Degree2Radian<double>()(mParams->mNoiseFilterLevelParam));
        Eigen::Vector2f v0(std::cos(mHAngles[0]) * mRanges0[0] * std::cos(mVAngles[0]), std::sin(mHAngles[0]) * mRanges0[0] * std::cos(mVAngles[0]));
        Eigen::Vector2f v1(std::cos(mHAngles[1]) * mRanges0[1] * std::cos(mVAngles[1]), std::sin(mHAngles[1]) * mRanges0[1] * std::cos(mVAngles[1]));
        Eigen::Vector2f v2(std::cos(mHAngles[2]) * mRanges0[2] * std::cos(mVAngles[2]), std::sin(mHAngles[2]) * mRanges0[2] * std::cos(mVAngles[2]));
        Eigen::Vector2f ar = v2 - v0;
        Eigen::Vector2f am = v1;
        double noiseDiscriminator(0);

        for (int j(1); j < mHAngles.size() - 1; j++)
        {
          if (j != 1)
          {
            v0 = v1;
            v1 = v2;
            v2 = Eigen::Vector2f(std::cos(mHAngles[j + oi]) * mRanges0[j + oi] * std::cos(mVAngles[j + oi]), std::sin(mHAngles[j + oi]) * mRanges0[j + oi] * std::cos(mVAngles[j + oi]));
            ar = v2 - v0;
            am = v1;
          }
          noiseDiscriminator = (ar.dot(am))/(ar.norm() * am.norm());

          if (noiseDiscriminator > filter_level)// remove left wing
          {
            hits[j] = 0; /// remove uncontrolled noises
            hits[j+1] = 0;
            if (j != mHAngles.size() - 2) /// do not remove [j+2] when j gets greater than size() - 2, j=2 -> j-2 = 0
            {
              hits[j+2] = 0;
            }
          }
          else if (noiseDiscriminator < -filter_level) // remove right wing
          {
            hits[j] = 0; /// remove uncontrolled noises
            hits[j-1] = 0;
            if (j != 1) /// do not remove [j-2] until j gets greater than 1, j=2 -> j-2 = 0
            {
              hits[j-2] = 0;
            }
          }
        }
        for (int j(0); j < hits.size(); j++) if (hits[j] == 0) mRanges0[j] = 0;

        int numValidPoints(0);
        std::shared_ptr< PointDataArray > tail = std::make_shared< PointDataArray >();
        float xCoord(0), yCoord(0), zCoord(0);
        Eigen::Vector3f coordinateVector(0,0,0);
        float m1(0), m2(0), m3(0);
        for (int j(0); j < mHAngles.size(); j++)
        {
          // remove some invalid points
          if ( mRanges0[j] < mParams->mMinRangeParam )        continue;
          if ( mRanges0[j] > mParams->mMaxRangeParam )        continue;
          if ( mHAngles[j] > mParams->mMaxHoriAngleParam )    continue;
          if ( mHAngles[j] < mParams->mMinHoriAngleParam )    continue;
          if ( mVAngles[j] > mParams->mMaxVertiAngleParam )   continue;
          if ( mVAngles[j] < mParams->mMinVertiAngleParam )   continue;

          // XYZ coordinates calculation
          xCoord = mRanges0[j] * std::cos( mVAngles[j] ) * std::cos( mHAngles[j] );
          yCoord = mRanges0[j] * std::cos( mVAngles[j] ) * std::sin( mHAngles[j] );
          zCoord = mRanges0[j] * std::sin( mVAngles[j] );

          if (mParams->mSensorCoordRX || mParams->mSensorCoordRY) /// optimization for faster computation
          {
            m1 = mParams->mExtrinsicTransformMatParam(0,0) * xCoord + mParams->mExtrinsicTransformMatParam(0,1)* yCoord + mParams->mExtrinsicTransformMatParam(0,2)* zCoord + mParams->mSensorCoordX;
            m2 = mParams->mExtrinsicTransformMatParam(1,0) * xCoord + mParams->mExtrinsicTransformMatParam(1,1)* yCoord + mParams->mExtrinsicTransformMatParam(1,2)* zCoord + mParams->mSensorCoordY; 
            m3 = mParams->mExtrinsicTransformMatParam(2,0) * xCoord + mParams->mExtrinsicTransformMatParam(2,1)* yCoord + mParams->mExtrinsicTransformMatParam(2,2)* zCoord + mParams->mSensorCoordZ;
            coordinateVector = Eigen::Vector3f(m1,m2,m3);
          }
          else
          {
            coordinateVector = Eigen::Vector3f(xCoord + mParams->mSensorCoordX, yCoord + mParams->mSensorCoordY, zCoord + mParams->mSensorCoordZ);
          }

          if ( coordinateVector.z() > mParams->mMaxZParam || coordinateVector.z() < mParams->mMinZParam)
          {
            continue;
          }
          if ( coordinateVector.y() < mParams->mMinYParam || coordinateVector.y() > mParams->mMaxYParam )
          {
            continue;
          }
          if ( coordinateVector.x() < mParams->mMinXParam || coordinateVector.x() > mParams->mMaxXParam )
          {
            continue;
          }

          tail->mSystemTime = sysTime;
          tail->mRangeArray.push_back( mRanges0[j] );
          tail->mVerticalAngleArray.push_back( mVAngles[j] );
          tail->mHorizontalAngleArray.push_back( mHAngles[j] );
          tail->mIntensityArray.push_back( mIntensities0[j] );
          tail->mRisingArray.push_back( mRisings0[j] );
          tail->mXCoordArray.push_back(coordinateVector.x());
          tail->mYCoordArray.push_back(coordinateVector.y());
          tail->mZCoordArray.push_back(coordinateVector.z());

          numValidPoints++;
        }

        if ( numValidPoints > 20)
        {
          cntFinalOutput++;
          mOutputDataLocker.lock();
          mOutputDataDeque.push_back( tail );
          mOutputDataLocker.unlock();

          /// OPTIMIZATION FOR LOW COMPUTATIONAL POWER ENV
          mOutputDataLocker.lock();
          if ( mOutputDataDeque.size() >= 6) /// 0.3SEC DELAY IN USER APPLICATION
          {
            mOutputDataDeque.clear();
            //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("DATA CLEAR. GET OUTPUT DATA FASTER\n"));
          }
          else if ( mOutputDataDeque.size() >= 3) /// 0.15SEC DELAY IN USER APPLICATION
          {
            mOutputDataDeque.pop_front();
            LOGPRINT(YujinRobotYrlDriver, YRL_LOG_WARN, ("THERE IS A DELAY IN REAL-TIME OUTPUT DATA PROCESSING. PLEASE TAKE THE OUTPUT DATA AT MINIMUM 20HZ FROM YOUR APPLICATION. \n"));
            //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("DATA POPFRONT. GET OUTPUT DATA FASTER\n"));
          }
          mOutputDataLocker.unlock();

          numValidPoints = 0;
        }
        /// optimization : performance checker
        if (clockPerformance.GetTimeElapsed() >= 1000)
        {
          clockPerformance.Start();
          //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("For last 1 second, cntCommThread : %d cntPrcsThread : %d cntFinalOutput : %d\n",cntCommThread ,cntPrcsThread,cntFinalOutput));
          cntCommThread = 0;
          cntPrcsThread = 0;
          cntFinalOutput = 0;
        }

        /// clear all in order to get new data
        mRanges0.clear();
        mHAngles.clear();
        mVAngles.clear();
        mRisings0.clear();
        mIntensities0.clear();
      }

      mRecordPreHoriAngleCnt = horizontalCount;
    }

    //--< calculate dt1_r, range0, tw, vertical_angle_in_degree >-----------------------------------
    float dt1_r(0); /// TIME OF FLIGHT OR RANGE
    float tw(0); /// WIDTH, INTENSITY OF LASER REFLECTED

    if (mParams->mOutputModeParam == 0)//RAW SIGNAL MODE
    {
      dt1_r = 0.001 * t1_r;
      tw = 0.001 * (t1_f - t1_r);
    }
    else//NORMAL RANGING MODE
    {
      dt1_r = t1_r;
      tw = 0.001 * values[i+3];
    }
    float range0 = dt1_r * 1e-3;
    
    //----------------------------------------------------------------------------------------------
    //--< save valuse >-----------------------------------------------------------------------------
    mVAngles.push_back( (static_cast<float>(verticalCount)/4095 * 180 - 90) * M_PI/180 );
    mHAngles.push_back( static_cast<float>(horizontalCount) );
    mRanges0.push_back( range0 );
    mRisings0.push_back( dt1_r );
    mIntensities0.push_back( tw );
    //----------------------------------------------------------------------------------------------
    
    /// FOR INTERNAL USE IN YUJIN ROBOT
    if (useRawDataGraph)
    {
      mRawGraphLocker.lock();
      switch (mGraphType)
      {
      case typeRisingWidthGraph:
        mGraphRisingWidthData.push_back(dt1_r);
        mGraphRisingWidthData.push_back(tw);
        if ( mGraphRisingWidthData.size() > 20000 )
        {
          LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("mGraphRisingWidthData: some raw data were dropped\n"));
          mGraphRisingWidthData.clear();
        }
        break;
          
      case typeRangeGraph:
        mGraphRangeData.push_back(range0);
        if ( mGraphRangeData.size() > 10000 )
        {
          LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("mGraphRangeData: some raw data were dropped\n"));
          mGraphRangeData.clear();
        }
        break;
      
      case typeHorizontalCountsGraph:
        mGraphHorizontalCountData.push_back(static_cast<float>(horizontalCount));
        if ( mGraphHorizontalCountData.size() > 10000 )
        {
          LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("mGraphHorizontalCountData: some raw data were dropped\n"));
          mGraphHorizontalCountData.clear();
        }
        break;

      case typeVerticalCountsGraph:
        mGraphVerticalCountData.push_back(static_cast<float>(verticalCount));
        if ( mGraphVerticalCountData.size() > 10000 )
        {
          LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("mGraphVerticalCountData: some raw data were dropped\n"));
          mGraphVerticalCountData.clear();
        }
        break;

      default:
        break;
      }
      mRawGraphLocker.unlock();
    }     

    // to cope with situation that cannot enter to the if statement
    // : horizontalCount < 100000 && mRecordPreHoriAngleCnt > horizontalCount
    if ( mHAngles.size() > 10000 )
    {
      //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("clear! horizontalCount: %d | mRecordPreHoriAngleCnt: %d\n", horizontalCount, mRecordPreHoriAngleCnt));
      mVAngles.clear();
      mHAngles.clear();
      mRanges0.clear();
      mRisings0.clear();
      mIntensities0.clear();
    }
  }
}

void YujinRobotYrlDriver::GetStatusData (float &v_pd, float &v_ld, float &t_pd,
                                        float &t_mcu, float &v_target_pd, float &rpm,
                                        float &vertical_speed, float &sample_rate)
{
  mStatusDataLocker.lock();
  v_pd = mLidarStatus->V_pd;
  v_ld = mLidarStatus->V_ld;
  t_pd = mLidarStatus->T_pd;
  t_mcu = mLidarStatus->T_mcu;
  v_target_pd = mLidarStatus->V_target_pd;
  rpm = mLidarStatus->rpm;
  vertical_speed = mLidarStatus->vertical_speed;
  sample_rate = mLidarStatus->sample_rate;
  mStatusDataLocker.unlock();
}

void YujinRobotYrlDriver::GetErrorCode (unsigned int &origin_info)
{
  mStatusDataLocker.lock();
  origin_info = mLidarStatus->origin_info;
  mStatusDataLocker.unlock();
}

void YujinRobotYrlDriver::GetDPR (float &dpr)
{
  dpr = mDPR;
}

int YujinRobotYrlDriver::GetUDPErrorCase ()
{
  return UDPErrorCase;
}

void YujinRobotYrlDriver::SetIPAddrParam (const std::string &ipAddr)
{
  mParams->mIPAddrParam = ipAddr;
  mNI->saveIPAddr(ipAddr);
}

void YujinRobotYrlDriver::SetPortNumParam (const unsigned short int portNum)
{
  mParams->mPortNumParam = portNum;
  mNI->savePortNum(portNum);
}

void YujinRobotYrlDriver::SetMinZParam (const float z_min)
{
  mParams->mMinZParam = z_min;
}

void YujinRobotYrlDriver::SetMaxZParam (const float z_max)
{
  mParams->mMaxZParam = z_max;
}

void YujinRobotYrlDriver::SetMinYParam (const float y_min)
{
  mParams->mMinYParam = y_min;
}

void YujinRobotYrlDriver::SetMaxYParam (const float y_max)
{
  mParams->mMaxYParam = y_max;
}

void YujinRobotYrlDriver::SetMinXParam (const float x_min)
{
  mParams->mMinXParam = x_min;
}

void YujinRobotYrlDriver::SetMaxXParam (const float x_max)
{
  mParams->mMaxXParam = x_max;
}

void YujinRobotYrlDriver::SetMinRangeParam (const float range_min)
{
  mParams->mMinRangeParam = range_min;
}

void YujinRobotYrlDriver::SetMaxRangeParam (const float range_max)
{
  mParams->mMaxRangeParam = range_max;
}

void YujinRobotYrlDriver::SetExtrinsicTransformMatParam (const float x, const float y, const float z,
                                                        const float rx, const float ry, const float rz)
{
  mParams->ArrayToExtrinsics(x, y, z, rx, ry, rz);
}

void YujinRobotYrlDriver::SetHoriAngleOffsetParam (const float hori_angle_offset)
{
  //mParams->mHoriAngleOffsetParam = hori_angle_offset;
  mParams->mSensorCoordRZ = hori_angle_offset;
}

void YujinRobotYrlDriver::SetVertiAngleOffsetParam (const float verti_angel_offset)
{
  mParams->mVertiAngleOffsetParam = verti_angel_offset;
}

void YujinRobotYrlDriver::SetOutputModeParam (const int output_mode)
{
  mParams->mOutputModeParam = output_mode;
}

void YujinRobotYrlDriver::SetVerticalModeParam (const float vertical_mode)
{
  mParams->mVerticalModeParam = vertical_mode;
}

void YujinRobotYrlDriver::SetMaxVertiAngleParam (const float verti_angle_max)
{
  mParams->mMaxVertiAngleParam = verti_angle_max;
}

void YujinRobotYrlDriver::SetMinVertiAngleParam (const float verti_angle_min)
{
  mParams->mMinVertiAngleParam = verti_angle_min;
}

void YujinRobotYrlDriver::SetMaxHoriAngleParam (const float hori_angle_max)
{
  mParams->mMaxHoriAngleParam = hori_angle_max;
}

void YujinRobotYrlDriver::SetMinHoriAngleParam (const float hori_angle_min)
{
  mParams->mMinHoriAngleParam = hori_angle_min;
}

void YujinRobotYrlDriver::SetNoiseFilterLevelParam (const float filter_level)
{
  mParams->mNoiseFilterLevelParam = filter_level;
}

std::string YujinRobotYrlDriver::GetIPAddrParam ()
{
  return mParams->mIPAddrParam;
}

unsigned short int YujinRobotYrlDriver::GetPortNumParam ()
{
  return mParams->mPortNumParam;
}

float YujinRobotYrlDriver::GetMinZParam ()
{
  return mParams->mMinZParam;
}

float YujinRobotYrlDriver::GetMaxZParam ()
{
  return mParams->mMaxZParam;
}

float YujinRobotYrlDriver::GetMinYParam ()
{
  return mParams->mMinYParam;
}

float YujinRobotYrlDriver::GetMaxYParam ()
{
  return mParams->mMaxYParam;
}

float YujinRobotYrlDriver::GetMinXParam ()
{
  return mParams->mMinXParam;
}

float YujinRobotYrlDriver::GetMaxXParam ()
{
  return mParams->mMaxXParam;
}

float YujinRobotYrlDriver::GetMinRangeParam ()
{
  return mParams->mMinRangeParam;
}

float YujinRobotYrlDriver::GetMaxRangeParam ()
{
  return mParams->mMaxRangeParam;
}

void YujinRobotYrlDriver::GetExtrinsicTransformParam (float &x, float &y, float &z,
                                                      float &rx, float &ry, float &rz)
{
  x = mParams->mSensorCoordX;
  y = mParams->mSensorCoordY;
  z = mParams->mSensorCoordZ;
  rx = mParams->mSensorCoordRX;
  ry = mParams->mSensorCoordRY;
  rz = mParams->mSensorCoordRZ;
}

float YujinRobotYrlDriver::GetHoriAngleOffsetParam ()
{
  //return mParams->mHoriAngleOffsetParam;
  return mParams->mSensorCoordRZ;
}

float YujinRobotYrlDriver::GetVertiAngleOffsetParam ()
{
  return mParams->mVertiAngleOffsetParam;
}

int YujinRobotYrlDriver::GetOutputModeParam ()
{
  return mParams->mOutputModeParam;
}

int YujinRobotYrlDriver::GetVerticalModeParam ()
{
  return mParams->mVerticalModeParam;
}

float YujinRobotYrlDriver::GetMaxVertiAngleParam ()
{
  return mParams->mMaxVertiAngleParam;
}

float YujinRobotYrlDriver::GetMinVertiAngleParam ()
{
  return mParams->mMinVertiAngleParam;
}

float YujinRobotYrlDriver::GetMaxHoriAngleParam ()
{
  return mParams->mMaxHoriAngleParam;
}

float YujinRobotYrlDriver::GetMinHoriAngleParam ()
{
  return mParams->mMinHoriAngleParam;
}

float YujinRobotYrlDriver::GetNoiseFilterLevelParam ()
{
  return mParams->mNoiseFilterLevelParam;
}

void YujinRobotYrlDriver::FWGetYRLInitialSettingParameters ()
{
  uint32_t temp = 0;
  int32_t temp2 = 0;
  
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 5, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.adjust_mode = temp;
  mParams->mOutputModeParam = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("adjust mode: %d\n", mLidarParam.adjust_mode));

  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 7, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  temp -= 45;
  mLidarParam.vertical_lower = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("vertical angle lower: %d\n", mLidarParam.vertical_lower));

  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 8, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  temp -= 45;
  mLidarParam.vertical_upper = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("vertical angle upper: %d\n", mLidarParam.vertical_upper));

  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 9, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.vertical_speed = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("vertical speed: %d\n", mLidarParam.vertical_speed));

  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 10, 0);
  memcpy(&temp2, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.vertical_offset_lower = temp2;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("vertical offset lower: %d\n", mLidarParam.vertical_offset_lower));

  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 11, 0);
  memcpy(&temp2, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.vertical_mode = temp2;
  SetVerticalModeParam (temp2);
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("vertical mode: %d\n", mLidarParam.vertical_mode));

  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 15, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.serial_number1 = temp;
  
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 16, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.serial_number2 = temp;
  
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 17, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.serial_number3 = temp;

  // get model number
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 19, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.model_no = temp;

  // get horizontal offset
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 21, 0);
  memcpy(&temp2, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.horizontal_offset = temp2;
  SetHoriAngleOffsetParam (static_cast<float> (temp2) / 10);
  //mParams->mHoriAngleOffsetParam = temp2;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("horizontal offset: %d\n", mLidarParam.horizontal_offset));

  // get vertical offset
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 22, 0);
  memcpy(&temp2, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.vertical_offset = temp2;
  SetVertiAngleOffsetParam(static_cast<float> (temp2) / 10);
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("vertical offset: %d\n", mLidarParam.vertical_offset));

  // get MAC address 1
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 23, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.MAC1 = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("MAC1: %d\n", mLidarParam.MAC1));

  // get MAC address 2
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 24, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.MAC2 = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LOG_INFO, ("MAC2: %d\n", mLidarParam.MAC2));

  // get ethernet board FW version_major
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 25, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.eth_major = temp;

  // get ethernet board FW version_minor
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 26, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.eth_minor = temp;
  
  // get ethernet board FW version_revision
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 27, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.eth_revision = temp;
  
  // get TDC board FW major version 
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 28, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.tdc_major = temp;

  // get TDC board FW minor version 
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 29, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.tdc_minor = temp;

  // get TDC board FW tdc_revision
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 30, 0);
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.tdc_revision = temp;

  // send command to start lidar
  mNI->doCommunicationMode(SEND_CMD_MODE, CMD_CMD, 1, 13);

  // start UDP communication
  mNI->doCommunicationMode(RECV_DATA_MODE, 0, 0, 0);
}

void YujinRobotYrlDriver::FWGetYRLSiPMVoltage ()
{
  // get SiPM voltage
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 1, 0);

  // save SiPM voltage
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.bias_volt = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("1. SiPM voltage: %d\n", mLidarParam.bias_volt));
}

void YujinRobotYrlDriver::FWGetYRLSiPMGain ()
{
  // get SiPM gain
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 2, 0);

  // save SiPM gain
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.gain_volt = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("2. SiPM gain: %d\n", mLidarParam.gain_volt));
}

void YujinRobotYrlDriver::FWGetYRLLDVoltage ()
{
  // get LD voltage
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 3, 0);

  // save LD voltage
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.ld_volt = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("3. LD voltage: %d\n", mLidarParam.ld_volt));
}

void YujinRobotYrlDriver::FWGetYRLLDPulseWidth ()
{
  // get LD pulse width
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 4, 0);

  // save LD pulse width
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.ld_pulse = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("4. LD pulse width: %d\n", mLidarParam.ld_pulse));
}

void YujinRobotYrlDriver::FWGetYRLAdjustMode ()
{
  // get adjust mode
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 5, 0);

  // save adjust mode
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.adjust_mode = temp;
  SetOutputModeParam(temp);
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("5. adjust mode: %d\n", mLidarParam.adjust_mode));
}

void YujinRobotYrlDriver::FWGetYRLSafetyRotationSpeed ()
{
  // get safety rotation speed
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 6, 0);

  // save safety rotation speed
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.safety_rotation_ms = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("6. safety ratation speed: %d\n", mLidarParam.safety_rotation_ms));
}

void YujinRobotYrlDriver::FWGetYRLVerticalAngleLower ()
{
  // get lower vertical angle
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 7, 0);

  // save slower vertical angle
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  temp -= 45;
  mLidarParam.vertical_lower = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("vertical angle lower: %d\n", mLidarParam.vertical_lower));
}

void YujinRobotYrlDriver::FWGetYRLVerticalAngleUpper ()
{
  // get upper vertical angle
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 8, 0);

  // save upper vertical angle
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  temp -= 45;
  mLidarParam.vertical_upper = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("vertical angle upper: %d\n", mLidarParam.vertical_upper));
}

void YujinRobotYrlDriver::FWGetYRLVerticalSpeed ()
{
  // get vertical speed
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 9, 0);

  // save vertical speed
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.vertical_speed = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("vertical speed: %d\n", mLidarParam.vertical_speed));
}

void YujinRobotYrlDriver::FWGetYRLVerticalOffsetLower ()
{
  // get lower vertical offset
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 10, 0);

  // save lower vertical offset
  int32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.vertical_offset_lower = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("10. vertical offset lower: %d\n", mLidarParam.vertical_offset_lower));
}

void YujinRobotYrlDriver::FWGetYRLVerticalMode ()
{
  // get upper vertical offset
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 11, 0);

  // save upper vertical offset
  int32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.vertical_mode = temp;
  SetVerticalModeParam(temp);
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("vertical mode: %d\n", mLidarParam.vertical_mode));
}

void YujinRobotYrlDriver::FWGetYRLTableSize ()
{
  // get table size
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 12, 0);

  // save table size
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.table_size = temp;
  mNI->saveTableSize(mLidarParam.table_size);
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("12. table size: %d\n", mLidarParam.table_size));
}

void YujinRobotYrlDriver::FWGetYRLWidthTable (std::vector<int32_t> &_width_table)
{
  if (mLidarParam.table_size <= 42)
  {
    // get width table
    mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 13, 0);

    // save width table
    saveWidthTableParameter1(mNI->mBulkDataBuffer, mLidarParam.table_size);

    _width_table.clear();
    for (int i = 0; i < mLidarParam.table_size; i++)
    {
      _width_table.push_back(mLidarParam.width_table1[i]);
    }
  }
  else
  {
    // get width table
    mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 13, 0);

    // save width table
    saveWidthTableParameter1(mNI->mBulkDataBuffer, 42);

    mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 31, 0);

    saveWidthTableParameter2(mNI->mBulkDataBuffer, 42);

    _width_table.clear();
    for (int i = 0; i < 42; i++)
    {
      _width_table.push_back(mLidarParam.width_table1[i]);
    }

    for (int i = 0; i < mLidarParam.table_size - 42; i++)
    {
      _width_table.push_back(mLidarParam.width_table2[i]);
    }
  }

  // print width table
  printWidthTableParameter();
}

void YujinRobotYrlDriver::FWGetYRLCompTable (std::vector<int32_t> &_comp_table)
{
  if (mLidarParam.table_size <= 42)
  {
    // get comp table
    mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 14, 0);

    // save comp table
    saveCompTableParameter1(mNI->mBulkDataBuffer, mLidarParam.table_size);

    _comp_table.clear();
    for (int i = 0; i < mLidarParam.table_size; i++)
    {
      _comp_table.push_back(mLidarParam.comp_table1[i]);
    }
  }
  else
  {
    // get comp table
    mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 14, 0);

    // save comp table
    saveCompTableParameter1(mNI->mBulkDataBuffer, 42);

    mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 32, 0);

    saveCompTableParameter2(mNI->mBulkDataBuffer, 42);

    _comp_table.clear();
    for (int i = 0; i < 42; i++)
    {
      _comp_table.push_back(mLidarParam.comp_table1[i]);
    }
    for (int i = 0; i < mLidarParam.table_size - 42; i++)
    {
      _comp_table.push_back(mLidarParam.comp_table2[i]);
    }
  }

  // print comp table
  printCompTableParameter();
}

void YujinRobotYrlDriver::FWGetYRLSerialNumber1 ()
{
  // get serial number1
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 15, 0);

  // save serial number1
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.serial_number1 = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("15. serial number1: %d\n", mLidarParam.serial_number1));
}

void YujinRobotYrlDriver::FWGetYRLSerialNumber2 ()
{
  // get serial number2
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 16, 0);

  // save serial number2
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.serial_number2 = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("16. serial number2: %d\n", mLidarParam.serial_number2));
}

void YujinRobotYrlDriver::FWGetYRLSerialNumber3 ()
{
  // get serial number3
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 17, 0);

  // save serial number3
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.serial_number3 = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("17. serial number3: %d\n", mLidarParam.serial_number3));
}

void YujinRobotYrlDriver::FWGetYRLIPAddress (int& ip_a, int& ip_b, int& ip_c, int& ip_d)
{
  // get IP address
  mNI->doCommunicationMode(SEND_CMD_MODE, CMD_GET_PARAM, 18, 0);

  // save IP address
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.ip_address = temp;
  ip_a = (0xff & (temp >> 0));
  ip_b = (0xff & (temp >> 8));
  ip_c = (0xff & (temp >> 16));
  ip_d = (0xff & (temp >> 24));

  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("Ip address: %d.%d.%d.%d\n", ip_a, ip_b, ip_c, ip_d));

  std::string ip = std::to_string (ip_a) + "." + std::to_string (ip_b) + "." + std::to_string (ip_c) + "." + std::to_string (ip_d);
  SetIPAddrParam (ip);
}

void YujinRobotYrlDriver::FWGetYRLModelNumber ()
{
  // get model number
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 19, 0);

  // save model number
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.model_no = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("19. model no: %d\n", mLidarParam.model_no));
}

void YujinRobotYrlDriver::FWGetYRLHorizontalSpeed ()
{
  // get horizontal speed
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 20, 0);

  // save horizontal speed
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.horizontal_speed = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("20. horizontal speed: %d\n", mLidarParam.horizontal_speed));
}

void YujinRobotYrlDriver::FWGetYRLHorizontalOffset ()
{
  // get horizontal offset
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 21, 0);

  // save horizontal offset
  int32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.horizontal_offset = temp;
  SetHoriAngleOffsetParam(static_cast<float> (temp) / 10);
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("21. horizontal offset: %d\n", mLidarParam.horizontal_offset));
}

void YujinRobotYrlDriver::FWGetYRLVerticalOffset ()
{
  // get vertical offset
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 22, 0);

  // save vertical offset
  int32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(int32_t));
  mLidarParam.vertical_offset = temp;
  SetVertiAngleOffsetParam (static_cast<float> (temp) / 10);
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("22. vertical offset: %d\n", mLidarParam.vertical_offset));
}

void YujinRobotYrlDriver::FWGetYRLMAC1 ()
{
  // get MAC address 1
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 23, 0);

  // save MAC address 1
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.MAC1 = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("23. MAC1: %d\n", mLidarParam.MAC1));
}

void YujinRobotYrlDriver::FWGetYRLMAC2 ()
{
  // get MAC address 2
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 24, 0);

  // save ethernet board FW version_major
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.MAC2 = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("24. MAC2: %d\n", mLidarParam.MAC2));
}

void YujinRobotYrlDriver::FWGetYRLEthFWMajor ()
{
  // get ethernet board FW version_major
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 25, 0);

  // save ethernet board FW version_major
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.eth_major = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("25. eth_major: %d\n", mLidarParam.eth_major));
}

void YujinRobotYrlDriver::FWGetYRLEthFWMinor ()
{
  // get ethernet board FW version_minor
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 26, 0);

  // save ethernet board FW version_minor
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.eth_minor = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("26. eth_minor: %d\n", mLidarParam.eth_minor));
}

void YujinRobotYrlDriver::FWGetYRLEthFWRevision ()
{
  // get ethernet board FW version_revision
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 27, 0);

  // save ethernet board FW version_revision
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.eth_revision = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("27. eth_revision: %d\n", mLidarParam.eth_revision));
}

void YujinRobotYrlDriver::FWGetYRLTDCFWMajor ()
{
  // get TDC board FW major version 
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 28, 0);

  // save TDC board FW major version
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.tdc_major = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("28. tdc_major: %d\n", mLidarParam.tdc_major));
}

void YujinRobotYrlDriver::FWGetYRLTDCFWMinor ()
{
  // get TDC board FW minor version 
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 29, 0);

  // save TDC board FW minor version
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.tdc_minor = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("29. tdc_minor: %d\n", mLidarParam.tdc_minor));
}

void YujinRobotYrlDriver::FWGetYRLTDCFWRevision ()
{
  // get TDC board FW tdc_revision
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 30, 0);

  // save TDC board FW tdc_revision
  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.tdc_revision = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("30. tdc_revision: %d\n", mLidarParam.tdc_revision));
}

void YujinRobotYrlDriver::FWGetYRLFactoryMode ()
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 33, 0);

  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.factory_mode = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("33. factory_mode: %d\n", mLidarParam.factory_mode));
}

void YujinRobotYrlDriver::FWGetYRLMainModelNumber ()
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_GET_PARAM, 34, 0);

  uint32_t temp = 0;
  memcpy(&temp, &mNI->mBulkDataBuffer[0], sizeof(uint32_t));
  mLidarParam.main_model_no = temp;
  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("34. main_model_no: %d\n", mLidarParam.main_model_no));
}

void YujinRobotYrlDriver::FWSetYRLSiPMVoltage (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 1, value);
}

void YujinRobotYrlDriver::FWSetYRLSiPMGain (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 2, value);
}

void YujinRobotYrlDriver::FWSetYRLLDVoltage (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 3, value);
}

void YujinRobotYrlDriver::FWSetYRLLDPulseWidth (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 4, value);
}

void YujinRobotYrlDriver::FWSetYRLAdjustMode (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 5, value);
}

void YujinRobotYrlDriver::FWSetYRLSafetyRotationSpeed (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 6, value);
}

void YujinRobotYrlDriver::FWSetYRLVerticalAngleLower (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 7, value);
}

void YujinRobotYrlDriver::FWSetYRLVerticalAngleUpper (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 8, value);
}

void YujinRobotYrlDriver::FWSetYRLVerticalSpeed (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 9, value);
}

void YujinRobotYrlDriver::FWSetYRLVerticalOffsetLower (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 10, value);
}

void YujinRobotYrlDriver::FWSetYRLVerticalMode (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 11, value);
}

void YujinRobotYrlDriver::FWSetYRLTableSize (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 12, value);
}

void YujinRobotYrlDriver::FWSetYRLWidthTable (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 13, value);
}

void YujinRobotYrlDriver::FWSetYRLCompTable (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 14, value);
}

void YujinRobotYrlDriver::FWSetYRLSerialNumber1 (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 15, value);
}

void YujinRobotYrlDriver::FWSetYRLSerialNumber2 (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 16, value);
}

void YujinRobotYrlDriver::FWSetYRLSerialNumber3 (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 17, value);
}

void YujinRobotYrlDriver::FWSetYRLIPAddress (int value1, int value2, int value3, int value4)
{
  uint32_t ip_add(0);
  uint32_t ip1 = (static_cast<uint32_t>(value1)) & 0x000000ff;
  uint32_t ip2 = (static_cast<uint32_t>(value2) << 8) & 0x0000ff00;
  uint32_t ip3 = (static_cast<uint32_t>(value3) << 16) & 0x00ff0000;
  uint32_t ip4 = (static_cast<uint32_t>(value4) << 24) & 0xff000000;
  ip_add = ip1 | ip2 | ip3 | ip4;

  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 18, ip_add);
}

void YujinRobotYrlDriver::FWSetYRLModelNumber (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 19, value);
}

void YujinRobotYrlDriver::FWSetYRLHorizontalSpeed (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 20, value);
}

void YujinRobotYrlDriver::FWSetYRLHorizontalOffset (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 21, value);
}

void YujinRobotYrlDriver::FWSetYRLVerticalOffset (int value)	
{	
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 22, value);	
}

void YujinRobotYrlDriver::FWSetYRLFactoryMode (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 33, value);	
}

void YujinRobotYrlDriver::FWSetYRLMainModelNumber (int value)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_SET_PARAM, 34, value);	
}

void YujinRobotYrlDriver::FWCMD (uint8_t code, int32_t data)
{
  mNI->doCommunicationMode (SEND_CMD_MODE, CMD_CMD, code, data);
  
  switch (code)
  {
  case 1:
    if (data == 11) //reboot ethernet
    {
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(10000));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(10000));
#endif
    }
    else if (data == 9) //save param main
    {
      //additional delay for saving table
      //The larger the size of table, the longer it may take.
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
#endif
    }
    else if (data == 12) //save param ethernet
    {
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
#endif
    }
    else if (data == 6) //reboot main
    {
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#endif
    }
    else if (data = 14) //
    {
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
#endif
    }
    else if (data = 16) //for calibration mode
    {
      //std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    else
    {
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
#endif
    }

    break;
  case 2:
    /* code */
    break;
  case 3:
    /* code */
    break;
  case 4:
    /* code */
    break;
  default:
    break;
  }
}

void YujinRobotYrlDriver::StartStreaming ()
{
  mNI->doCommunicationMode(RECV_DATA_MODE, 0, 0, 0);
}

void YujinRobotYrlDriver::printWholeParameter ()
{
  printf("1. SiPM voltage: %d\n", mLidarParam.bias_volt);
  printf("2. SiPM gain: %d\n", mLidarParam.gain_volt);
  printf("3. LD voltage: %d\n", mLidarParam.ld_volt);
  printf("4. LD pulse width: %d\n", mLidarParam.ld_pulse);
  printf("5. adjust mode: %d\n", mLidarParam.adjust_mode);
  printf("6. safety ratation speed: %d\n", mLidarParam.safety_rotation_ms);
  printf("7. vertical angle lower: %d\n", mLidarParam.vertical_lower);
  printf("8. vertical angle upper: %d\n", mLidarParam.vertical_upper);
  printf("9. vertical speed: %d\n", mLidarParam.vertical_speed);
  printf("10. vertical offset lower: %d\n", mLidarParam.vertical_offset_lower);
  printf("11. vertical mode: %d\n", mLidarParam.vertical_mode);
  printf("12. table size: %d\n", mLidarParam.table_size);
  printWidthTableParameter ();
  printCompTableParameter ();

  char sn[12] = {0};
  sn[0] = 0xff & (mLidarParam.serial_number1 >> 0);
  sn[1] = 0xff & (mLidarParam.serial_number1 >> 8);
  sn[2] = 0xff & (mLidarParam.serial_number1 >> 16);
  sn[3] = 0xff & (mLidarParam.serial_number1 >> 24);
  sn[4] = 0xff & (mLidarParam.serial_number2 >> 0);
  sn[5] = 0xff & (mLidarParam.serial_number2 >> 8);
  sn[6] = 0xff & (mLidarParam.serial_number2 >> 16);
  sn[7] = 0xff & (mLidarParam.serial_number2 >> 24);
  sn[8] = 0xff & (mLidarParam.serial_number3 >> 0);
  sn[9] = 0xff & (mLidarParam.serial_number3 >> 8);
  sn[10] = 0xff & (mLidarParam.serial_number3 >> 16);
  sn[11] = 0xff & (mLidarParam.serial_number3 >> 24);
  printf("15~17. serial number: ");
  for (int i = 0; i < 12; i++)
  {
    printf("%x ", sn[i]);
  }
  printf("\n");

  int ip_a = (0xff & (mLidarParam.ip_address >> 0));
  int ip_b = (0xff & (mLidarParam.ip_address >> 8));
  int ip_c = (0xff & (mLidarParam.ip_address >> 16));
  int ip_d = (0xff & (mLidarParam.ip_address >> 24));
  printf("18. ip address: %d.%d.%d.%d\n", ip_a, ip_b, ip_c, ip_d);
  printf("19. model no: %d\n", mLidarParam.model_no);
  printf("20. horizontal speed: %d\n", mLidarParam.horizontal_speed);
  printf("21. horizontal offset: %d\n", mLidarParam.horizontal_offset);
  printf("22. vertical offset: %d\n", mLidarParam.vertical_offset);

  char mac[6] = {0};
  mac[0] = 0xff & (mLidarParam.MAC1 >> 0);
  mac[1] = 0xff & (mLidarParam.MAC1 >> 8);
  mac[2] = 0xff & (mLidarParam.MAC1 >> 16);
  mac[3] = 0xff & (mLidarParam.MAC1 >> 24);
  mac[4] = 0xff & (mLidarParam.MAC2 >> 0);
  mac[5] = 0xff & (mLidarParam.MAC2 >> 8);
  printf("23~24. MAC: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%x ", mac[i]);
  }
  printf("\n");

  printf("25. eth_major: %d\n", mLidarParam.eth_major);
  printf("26. eth_minor: %d\n", mLidarParam.eth_minor);
  printf("27. eth_revision: %d\n", mLidarParam.eth_revision);
  printf("28. tdc_major: %d\n", mLidarParam.tdc_major);
  printf("29. tdc_minor: %d\n", mLidarParam.tdc_minor);
  printf("30. tdc_revision: %d\n", mLidarParam.tdc_revision);
}
    
void YujinRobotYrlDriver::saveWidthTableParameter1 (unsigned char * recv_buffer, const int table_size)
{
  mLidarWidthTable = (int32_t*)malloc(table_size * sizeof(int32_t));
  memset(mLidarWidthTable, 0, sizeof(mLidarWidthTable));
  memcpy(mLidarWidthTable, recv_buffer, table_size * sizeof(int32_t));
  mLidarParam.width_table1 = mLidarWidthTable;
}

void YujinRobotYrlDriver::saveWidthTableParameter2 (unsigned char * recv_buffer, const int table_size)
{
  mLidarWidthTable = (int32_t*)malloc(table_size * sizeof(int32_t));
  memset(mLidarWidthTable, 0, sizeof(mLidarWidthTable));
  memcpy(mLidarWidthTable, recv_buffer, table_size * sizeof(int32_t));
  mLidarParam.width_table2 = mLidarWidthTable;
}

void YujinRobotYrlDriver::printWidthTableParameter ()
{
  std::string log = "13. width table: \n";

  if (mLidarParam.table_size <= 42)
  {
    if (mLidarParam.width_table1 != nullptr)
    {
      for (int i = 0; i < mLidarParam.table_size; i++)
      {
        log += "[" + std::to_string(mLidarParam.width_table1[i]) + "], ";
      }
      log += "\n";
    }
    else
    {
      log += "width table 1 is not exist\n";
    }
  }
  else
  {
    if (mLidarParam.width_table1 == nullptr)
    {
      log += "width table 1 is not exist\n";
    }
    else
    {
      for (int i = 0; i < 42; i++)
      {
          log += "[" + std::to_string(mLidarParam.width_table1[i]) + "], ";
      }
      log += "\n";
    }

    if (mLidarParam.width_table2 == nullptr)
    {
      log += "width table 2 is not exist\n";
    }
    else
    {
      for (int i = 0; i < 42; i++)
      {
        log += "[" + std::to_string(mLidarParam.width_table2[i]) + "], ";
      }
      log += "\n";
    }
  }

  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("%s", log.c_str()));
}

void YujinRobotYrlDriver::saveCompTableParameter1 (unsigned char * recv_buffer, const int table_size)
{
  mLidarCompTable = (int32_t*)malloc(table_size * sizeof(int32_t));
  memset(mLidarCompTable, 0, sizeof(mLidarCompTable));
  memcpy(mLidarCompTable, recv_buffer, table_size * sizeof(int32_t));
  mLidarParam.comp_table1 = mLidarCompTable;
}

void YujinRobotYrlDriver::saveCompTableParameter2 (unsigned char * recv_buffer, const int table_size)
{
  mLidarCompTable = (int32_t*)malloc(table_size * sizeof(int32_t));
  memset(mLidarCompTable, 0, sizeof(mLidarCompTable));
  memcpy(mLidarCompTable, recv_buffer, table_size * sizeof(int32_t));
  mLidarParam.comp_table2 = mLidarCompTable;
}

void YujinRobotYrlDriver::printCompTableParameter ()
{
  std::string log = "14. comp table: \n";

  if (mLidarParam.table_size <= 42)
  {
    if (mLidarParam.comp_table1 != nullptr)
    {
      for (int i = 0; i < mLidarParam.table_size; i++)
      {
        log += "[" + std::to_string(mLidarParam.comp_table1[i]) + "], ";
      }
      log += "\n";
    }
    else
    {
      log += "comp table 1 is not exist\n";
    }
  }
  else
  {
    if (mLidarParam.comp_table1 == nullptr)
    {
      log += "comp table 1 is not exist\n";
    }
    else
    {
      for (int i = 0; i < 42; i++)
      {
        log += "[" + std::to_string(mLidarParam.comp_table1[i]) + "], ";
      }
      log += "\n";
    }
    
    if (mLidarParam.comp_table2 == nullptr)
    {
      log += "comp table 2 is not exist\n";
    }
    else
    {
      for (int i = 0; i < 42; i++)
      {
        log += "[" + std::to_string(mLidarParam.comp_table2[i]) + "], ";
      }
      log += "\n";
    }
  }

  LOGPRINT(YujinRobotYrlDriver, YRL_LIDAR_INFO, ("%s", log.c_str()));
}

int YujinRobotYrlDriver::GetWholeOutput (double &_SystemTime,
                                        std::vector<float> &_RangeArray,
                                        std::vector<float> &_HorizontalAngleArray,
                                        std::vector<float> &_VerticalAngleArray,
                                        std::vector<float> &_RisingArray,
                                        std::vector<float> &_IntensityArray,
                                        std::vector<float> &_XCoordArray,
                                        std::vector<float> &_YCoordArray,
                                        std::vector<float> &_ZCoordArray)
{
  std::shared_ptr< PointDataArray > head;
  mOutputDataLocker.lock();
  if (mOutputDataDeque.empty())
  {
    mOutputDataLocker.unlock();
    return -1;
  }
  else
  {
    //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("mOutputDataDeque.size() : %d\n", mOutputDataDeque.size()));
    head = mOutputDataDeque.front();
    mOutputDataDeque.pop_front();
    mOutputDataLocker.unlock();
  }

  _SystemTime             = head->mSystemTime;
  _RangeArray             = head->mRangeArray;
  _HorizontalAngleArray   = head->mHorizontalAngleArray;
  _VerticalAngleArray     = head->mVerticalAngleArray;
  _RisingArray            = head->mRisingArray;
  _IntensityArray         = head->mIntensityArray;
  _XCoordArray            = head->mXCoordArray;
  _YCoordArray            = head->mYCoordArray;
  _ZCoordArray            = head->mZCoordArray;

  return 1;
}

int YujinRobotYrlDriver::GetCartesianOutputsWithIntensity (double &_SystemTime,
                                                          std::vector <float>& _IntensityArray,
                                                          std::vector <float>& _XCoordArray,
                                                          std::vector <float>& _YCoordArray,
                                                          std::vector <float>& _ZCoordArray)
{
  /// *** DO NOT USE THIS FUNCTION WITH getSphericalOutputsWithIntensity
  std::shared_ptr< PointDataArray > head;
  mOutputDataLocker.lock();
  if (mOutputDataDeque.empty())
  {
    mOutputDataLocker.unlock();
    return -1;
  }
  else
  {
    //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("mOutputDataDeque.size() : %d\n", mOutputDataDeque.size()));        
    head = mOutputDataDeque.front();
    mOutputDataDeque.pop_front();
    mOutputDataLocker.unlock();
  }

  _SystemTime     = head->mSystemTime;
  _IntensityArray = head->mIntensityArray;
  _XCoordArray    = head->mXCoordArray;
  _YCoordArray    = head->mYCoordArray;
  _ZCoordArray    = head->mZCoordArray;

  return 1;
}

int YujinRobotYrlDriver::GetSphericalOutputsWithIntensity (double &_SystemTime,
                                                          std::vector <float>& _IntensityArray,
                                                          std::vector <float>& _RangeArray, 
                                                          std::vector <float>& _HorizontalAngleArray, 
                                                          std::vector <float>& _VerticalAngleArray)
{
  /// *** DO NOT USE THIS FUNCTION WITH getCartesianOutputsWithIntensity
  std::shared_ptr< PointDataArray > head;

  mOutputDataLocker.lock();
  if (mOutputDataDeque.empty())
  {
    mOutputDataLocker.unlock();
    return -1;
  }
  else
  {
    //LOGPRINT(YujinRobotYrlDriver, YRL_LOG_DEBUG, ("mOutputDataDeque.size() : %d\n", mOutputDataDeque.size()));
    head = mOutputDataDeque.front();
    mOutputDataDeque.pop_front();
    mOutputDataLocker.unlock();
  }

  _SystemTime             = head->mSystemTime;
  _IntensityArray         = head->mIntensityArray;
  _RangeArray             = head->mRangeArray;
  _HorizontalAngleArray   = head->mHorizontalAngleArray;
  _VerticalAngleArray     = head->mVerticalAngleArray;

  return 1;
}

bool YujinRobotYrlDriver::getuseRawDataGraph_UI ()
{
  return useRawDataGraph;
}

int YujinRobotYrlDriver::getGraphDataRisingWidth ( std::vector< float > & data )
{
  if (mGraphRisingWidthData.empty())
  {
    return 0;
  }

  data.clear();
  mRawGraphLocker.lock();
  data = mGraphRisingWidthData;
  mGraphRisingWidthData.clear();
  mRawGraphLocker.unlock();

  return data.size();
}

int YujinRobotYrlDriver::getGraphDataRange ( std::vector< float > & data )
{
  if (mGraphRangeData.empty())
  {
    return 0;
  }

  data.clear();
  mRawGraphLocker.lock();
  data = mGraphRangeData;
  mGraphRangeData.clear();
  mRawGraphLocker.unlock();
  
  return data.size();
}

int YujinRobotYrlDriver::getGraphDataHorizontalCounts ( std::vector< float > & data )
{
  if (mGraphHorizontalCountData.empty())
  {
    return 0;
  }

  data.clear();
  mRawGraphLocker.lock();
  data = mGraphHorizontalCountData;
  mGraphHorizontalCountData.clear();
  mRawGraphLocker.unlock();

  return data.size();
}

int YujinRobotYrlDriver::getGraphDataVerticalCounts (std::vector< float > & data)
{
  if (mGraphVerticalCountData.empty())
  {
    return 0;
  }
  
  data.clear();
  mRawGraphLocker.lock();
  data = mGraphVerticalCountData;
  mGraphVerticalCountData.clear();
  mRawGraphLocker.unlock();

  return data.size();
}