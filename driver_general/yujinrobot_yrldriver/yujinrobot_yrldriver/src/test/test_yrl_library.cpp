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

#include <iostream>
#include "../../include/yujinrobot_yrldriver/yujinrobot_yrldriver.hpp"

#ifdef _WIN32
bool ctrlcpressed = false;
BOOL WINAPI CtrlHandler (DWORD fdwCtrlType)
{
  switch (fdwCtrlType)
  {
  // Handle the CTRL-C signal.
  case CTRL_C_EVENT:
    //printf("Ctrl-C event\n\n");
    Beep(750, 300);
    ctrlcpressed = true;
    return TRUE;

  // Pass other signals to the next handler.
  case CTRL_BREAK_EVENT:
    Beep(900, 200);
    ctrlcpressed = true;
    //printf("Ctrl-Break event\n\n");
    return TRUE;

  default:
    return FALSE;
  }
}

int threadFucntion()
{
  if (SetConsoleCtrlHandler(CtrlHandler, TRUE))
  {
    printf("\nPress Ctrl + C to exit the program.\n\n");
    while (1)
    {}
  }
  else
  {
    printf("\nERROR: Could not set control handler");
    return 1;
  }
  return 0;
}
#endif

int main( int argc, char ** argv)
{
#ifdef _WIN32
  std::thread thread(threadFucntion);
#endif

  std::cout << "========================================" << std::endl; 
  std::cout << "           Test YRL driver          " << std::endl; 
  std::cout << "========================================\n" << std::endl;

  if (argc != 2)
  {
    LOGPRINT(main, YRL_LOG_USER, ("WRONG ARGUMENT! USAGE: [./test_yrl_library] [IP_ADDRESS]\n"));
    return -1;
  }
  
  std::string ip = argv[1];

#ifdef _WIN32
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
  {
    printf("WSAStartup() error!");
  }
#endif

  //== 1. CREATE DRIVER INSTANCE ===========================================
  YujinRobotYrlDriver* instance = new YujinRobotYrlDriver();
  //========================================================================
  
  //== 2. SET IP ===========================================================
  // THIS MUST BE SET BEFOR CALLING Start().
  // THIS WILL BE USED TO CONNECT LIDAR
  instance->SetIPAddrParam(ip);
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
#ifdef _WIN32
  WSACleanup();
#endif
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
  float SensorX;
  float SensorY;
  float SensorZ;
  float SensorRX;
  float SensorRY;
  float SensorRZ;
  instance->GetExtrinsicTransformParam (SensorX, SensorY, SensorZ, SensorRX, SensorRY, SensorRZ);
  std::cout << "SensorX : " << SensorX << std::endl;
  std::cout << "SensorY : " << SensorY << std::endl;
  std::cout << "SensorZ : " << SensorZ << std::endl;
  std::cout << "SensorRX : " << SensorRX << std::endl;
  std::cout << "SensorRY : " << SensorRY << std::endl;
  std::cout << "SensorRZ : " << SensorRZ << std::endl;

  instance->SetExtrinsicTransformMatParam ( 0, 0, 1.0f, 0, 0, 0 );
  //========================================================================
    
  //== 5. OTHER GET PARAMETER FUNCTIONS ====================================
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

  std::string IpAddress = instance->GetIPAddrParam();
  std::cout << "inputIpAddress : " << IpAddress << std::endl;

  int PortNumber = instance->GetPortNumParam ();
  std::cout << "PortNumber : " << PortNumber << std::endl;

  float MinZ = instance->GetMinZParam();
  std::cout << "MinZ : " << MinZ << std::endl;

  float MaxZ = instance->GetMaxZParam();
  std::cout << "MaxZ : " << MaxZ << std::endl;

  float MinY = instance->GetMinYParam();
  std::cout << "MinY : " << MinY << std::endl;

  float MaxY = instance->GetMaxYParam();
  std::cout << "MaxY : " << MaxY << std::endl;

  float MinX = instance->GetMinXParam();
  std::cout << "MinX : " << MinX << std::endl;

  float MaxX = instance->GetMaxXParam();
  std::cout << "MaxX : " << MaxX << std::endl;

  float MinRange = instance->GetMinRangeParam();
  std::cout << "MinRange : " << MinRange << std::endl;

  float MaxRange = instance->GetMaxRangeParam();
  std::cout << "MaxRange : " << MaxRange << std::endl;

  float MaxVertiAngle = instance->GetMaxVertiAngleParam();
  std::cout << "MaxVertiAngle : " << MaxVertiAngle << std::endl;

  float MinVertiAngle = instance->GetMinVertiAngleParam();
  std::cout << "MinVertiAngle : " << MinVertiAngle << std::endl;

  float MaxHoriAngle = instance->GetMaxHoriAngleParam();
  std::cout << "MaxHoriAngle : " << MaxHoriAngle << std::endl;

  float MinHoriAngle = instance->GetMinHoriAngleParam();
  std::cout << "MinHoriAngle : " << MinHoriAngle << std::endl;

  float NoiseFilterLevel = instance->GetNoiseFilterLevelParam();
  std::cout << "NoiseFilterLevel : " << NoiseFilterLevel << std::endl;
  //========================================================================
  
  //== 6. OTHER SET PARAMETER FUNCTIONS ====================================
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

  instance->SetMinZParam ( 0 );
  instance->SetMaxZParam ( 5.0f );
  instance->SetMinYParam (-35.0f);
  instance->SetMaxYParam ( 35.0f );
  instance->SetMinXParam ( -35.0f );
  instance->SetMaxXParam ( 35.0f );
  instance->SetMinRangeParam ( 0 );
  instance->SetMaxRangeParam ( 35.0f );
  instance->SetMaxVertiAngleParam ( Degree2Radian<double>()( +40.0 ) );
  instance->SetMinVertiAngleParam ( Degree2Radian<double>()( -40.0 ) );
  instance->SetMaxHoriAngleParam ( Degree2Radian<double>()( +180.0 ) );
  instance->SetMinHoriAngleParam ( Degree2Radian<double>()( -180.0 ) );
  instance->SetNoiseFilterLevelParam (30.0);
  //========================================================================

  //== 7. START GETTING SENSOR DATA ========================================
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
  //
  // WE WILL GET DATA DURING 20SECS.
  instance->StartGettingDataStream();
  
  StopWatch sw;
  double SystemTime;
  std::vector <float> IntensityArray;
  std::vector <float> XCoordArray;
  std::vector <float> YCoordArray;
  std::vector <float> ZCoordArray;
  float dpr;

  sw.Start();
  while (sw.GetTimeElapsedInSec() < 20)
  {
#ifdef _WIN32
    if (ctrlcpressed)
    {
      ctrlcpressed = false;
      break;
    }
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    instance->GetDPR(dpr);
    // std::cout << "DPR: " << dpr << std::endl;

    int ret = instance->GetCartesianOutputsWithIntensity (SystemTime, IntensityArray, XCoordArray, YCoordArray, ZCoordArray);
    if (ret == -1)
    {
      // std::cout << "empty queue\n";
      continue;
    }
    
    // for (int i(0); i < intensity.size(); i++)
    // {
    //     std::cout << "intensity : " << intensity.at(i) << ", (" << coord_x.at(i) << ", " << coord_y.at(i) << ", " << coord_z.at(i) << ")" << std::endl;
    // }
  }
  //========================================================================
  
  delete instance;
#ifdef _WIN32
  WSACleanup();
#endif
  return 0;
}