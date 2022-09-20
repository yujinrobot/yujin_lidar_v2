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
    LOGPRINT(main, YRL_LOG_USER, ("WRONG ARGUMENT! USAGE: [./test_IP_change] [CURRENT_IP_ADDRESS] [NEW_IP_ADDRESS]\n"));
    return -1;
  }

  std::string ip = argv[1];
  std::string new_ip = argv[2];

#ifdef _WIN32
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
  {
    printf("WSAStartup() error!");
  }
#endif

  LOGPRINT(main, YRL_LOG_USER, ("CHANGE IP ADDRESS TO %s\n", argv[2]));

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

  //== 4. SET NEW IP OF LIDAR ==============================================
  instance->UserChangeIP(new_ip);
  //========================================================================

  //== 5. DISCONNECT LIDAR =================================================
  instance->DisconnectTOYRL3V2();
  //========================================================================

  //== 6. SET IP PARAMETER =================================================
  // SET IP PARAMETER WITH NEW IP ADDRESS
  instance->SetIPAddrParam(new_ip);
  //========================================================================

  //== 7. CONNECT TO LIDAR AGAIN ===========================================
  ret = instance->ConnectTOYRL3V2();
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
  //========================================================================

  delete instance;
#ifdef _WIN32
  WSACleanup();
#endif
  return 0;
}