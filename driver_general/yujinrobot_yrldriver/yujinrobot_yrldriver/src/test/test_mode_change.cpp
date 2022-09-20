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

int main( int argc, char ** argv)
{
  std::cout << "========================================" << std::endl; 
  std::cout << "           Test YRL driver          " << std::endl; 
  std::cout << "========================================\n" << std::endl;

  if (argc != 3)
  {
    LOGPRINT(main, YRL_LOG_USER, ("WRONG ARGUMENT! USAGE: [./test_mode_change] [IP_ADDRESS] [SCANNIG_MODE]\n"));
    return -1;
  }

  std::string ip = argv[1];
  int mode = std::stoi(argv[2]);

#ifdef _WIN32
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
  {
    printf("WSAStartup() error!");
  }
#endif

  LOGPRINT(main, YRL_LOG_USER, ("CHANGE SCANNING MODE TO %s\n", argv[2]));

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

  //== 4. SET NEW MODE OF LIDAR ============================================
  instance->UserChangeMode (mode);
  //========================================================================

  //== 5. START GETTING SENSOR DATA ========================================
  LOGPRINT(main, YRL_LOG_USER, ("START GETTING SCANNED DATA\n"));
  instance->StartGettingDataStream();
  
  double SystemTime;
  std::vector <float> IntensityArray;
  std::vector <float> XCoordArray;
  std::vector <float> YCoordArray;
  std::vector <float> ZCoordArray;
  float dpr;

  LOGPRINT(main, YRL_LOG_USER, ("DO SOMETHING....\n"));
  StopWatch sw;
  sw.Start();
  while (sw.GetTimeElapsedInSec() < 10)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    instance->GetDPR(dpr);
    int ret = instance->GetCartesianOutputsWithIntensity (SystemTime, IntensityArray, XCoordArray, YCoordArray, ZCoordArray);
    if (ret == -1)
    {
      // std::cout << "empty queue\n";
      continue;
    }
  }
  //========================================================================

  //== 6. SET NEW MODE OF LIDAR DURING GETTING SENSOR DATA =================
  instance->StopGettingDataStreamAndSet();
  instance->UserChangeMode (mode);
  //========================================================================

  //== 7. START GETTING SENSOR DATA AGAIN ==================================
  LOGPRINT(main, YRL_LOG_USER, ("START GETTING SCANNED DATA AGAIN\n"));
  instance->StartGettingDataStream();
  
  LOGPRINT(main, YRL_LOG_USER, ("DO SOMETHING....\n"));
  sw.Start();
  while (sw.GetTimeElapsedInSec() < 10)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    instance->GetDPR(dpr);
    int ret = instance->GetCartesianOutputsWithIntensity (SystemTime, IntensityArray, XCoordArray, YCoordArray, ZCoordArray);
    if (ret == -1)
    {
      // std::cout << "empty queue\n";
      continue;
    }
  }
  //========================================================================

  delete instance;
#ifdef _WIN32
  WSACleanup();
#endif
  return 0;
}