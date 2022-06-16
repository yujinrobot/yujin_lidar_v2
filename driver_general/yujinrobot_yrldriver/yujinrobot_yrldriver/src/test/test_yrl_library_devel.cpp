/*********************************************************************
*  Copyright (c) 2020, YujinRobot Corp.
*
*  Hyeon Jeong Kim, hjkim2@yujinrobot.com
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
#include <dlfcn.h>
#include "../../include/yujinrobot_yrldriver/yujinrobot_yrldriver.hpp"

int main( int argc, char ** argv)
{
    std::cout << "========================================" << std::endl; 
    std::cout << "           Test YRL driver          " << std::endl; 
    std::cout << "========================================\n" << std::endl;
 
    if (argc != 2)
    {
        LOGPRINT(main, YRL_LOG_USER, ("WRONG ARGUMENT! USAGE: [./test_yrl_library] [IP_ADDRESS]\n"));
        return -1;
    }
    
    std::string ip = argv[1];

    //== 1. CREATE DRIVER INSTANCE ===========================================
    YujinRobotYrlDriver* instance = new YujinRobotYrlDriver();
    //========================================================================
    
    //== 2. SET IP ===========================================================
    // THIS MUST BE SET BEFOR CALLING Start().
    // THIS WILL BE USED TO CONNECT LIDAR
    instance->SetIPAddrParam(ip);
    //========================================================================

    //== 3. START DRIVER =====================================================
    // THIS FUNCTION SHOULD BE ONLY ONCE CALLED.
    int ret = instance->Start();
    if (ret < 0)
    {
        std::string IpAddress = instance->GetIPAddrParam();
        int PortNumber = instance->GetPortNumParam ();
        LOGPRINT(main, YRL_LOG_USER, ("CANNOT START COMMUNICATION WITH LIDAR.\n"));
        LOGPRINT(main, YRL_LOG_USER, ("CONNECT TO [IP:%s PORT:%d] FAILED. CHECK YOUR NETWORK CONNECTION.\n", IpAddress.c_str(), PortNumber));
        delete instance;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    instance->FWCMD(1, 14);
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

    instance->FWCMD(1, 13);
    instance->StartStreaming();

    StopWatch sw;
    double SystemTime;
    std::vector <float> IntensityArray;
    std::vector <float> XCoordArray;
    std::vector <float> YCoordArray;
    std::vector <float> ZCoordArray;

    std::vector<float> RangeArray;
    std::vector<float> HorizontalAngleArray;
    std::vector<float> VerticalAngleArray;
    std::vector<float> RisingArray;
    
    float dpr;
    int cnt_invalid_x = 0;

    while (1)
    {

        //instance->GetDPR(dpr);
        //std::cout << "DPR: " << dpr << std::endl;

        instance->GetCartesianOutputsWithIntensity (SystemTime, IntensityArray, XCoordArray, YCoordArray, ZCoordArray);
        // for (int i(0); i < intensity.size(); i++)
        // {
        //     std::cout << "intensity : " << intensity.at(i) << ", (" << coord_x.at(i) << ", " << coord_y.at(i) << ", " << coord_z.at(i) << ")" << std::endl;
        // }

        for (int i(0); i < XCoordArray.size(); i++)
        {
            if (XCoordArray[i] > 35)
            {
                cnt_invalid_x++;
                std::cout << "x = " << XCoordArray[i] << ", invalid cnt: "<< cnt_invalid_x << std::endl;
            }
        }

        // instance->GetWholeOutput( SystemTime,
        //                 RangeArray, 
        //                 HorizontalAngleArray,
        //                 VerticalAngleArray, 
        //                 RisingArray, 
        //                 IntensityArray,
        //                 XCoordArray, 
        //                 YCoordArray, 
        //                 ZCoordArray);

        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    // StopWatch sw;

    // instance->FWSetYRLModelNumber(1);
    // instance->FWSetYRLMainModelNumber(1);

    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    // instance->FWCMD(1, 12);

    // instance->FWGetYRLMainModelNumber();
    // instance->FWGetYRLModelNumber();
    
    // std::cout << "main model number: " << instance->mLidarParam.main_model_no << "\n";
    // std::cout << "eth model number: " << instance->mLidarParam.model_no << "\n";

    // instance->FWSetYRLFactoryMode(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLFactoryMode();
    // int factory_mode = instance->mLidarParam.factory_mode;
    // if (factory_mode == 0)
    // {
    //     std::cout << "Factory mode is 0\n";
    // }
    // else
    // {
    //     std::cout << "Factory mode is 1\n";
    //     std::cout << "Turning off factory mode is failed\n";
    // }

    // instance->FWCMD(1, 13);
    // instance->StartStreaming();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 5)
    // {

    // }


        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // instance->FWCMD(1, 13);
    // instance->StartStreaming();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 5)
    // {

    // }
    
    // instance->mNI->mUDPSocket.SocketClose();
    
    
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 5)
    // {

    // }

    // instance->mNI->mUDPSocket.SocketCreate(SOCK_DGRAM, IPPROTO_UDP);
    // // structure for destination
    // sockaddr_in dest_addr;

    // bool ret2 = instance->mNI->mUDPSocket.buildAddrStructure("192.168.1.250", 1234, dest_addr);
    // // at the first we have to make structure to specify destination
    // if( !ret2 )
    // {
    //     LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("Empty serverAddress\n"));
    //     // std::cout << "Empty serverAddress" << std::endl;
    //     return -2;
    // }



    // int adjust_mode;
    // instance->FWGetYRLAdjustMode();
    // adjust_mode = instance->GetOutputModeParam();
    // if (adjust_mode == 1)
    // {
    //     std::cout << "setAdjustMode() : Adjust mode is 1\n";
    //     std::cout << "setAdjustMode() : Change adjust mode to 0\n";
    //     instance->FWSetYRLAdjustMode(0);
    //     instance->FWCMD(1, 9);
    //     instance->FWCMD(1, 14);

    //     instance->FWGetYRLAdjustMode();
    //     adjust_mode = instance->GetOutputModeParam();
    // }
    // else
    // {
    //     std::cout << "setAdjustMode() : Adjust mode is 0\n";
    // }

    // if (adjust_mode == 1)
    // {
    //     //FAIL!!
    //     std::cout << "setAdjustMode() : Changing adjust mode is failed\n";

    // }
    // else
    // {
    //     std::cout << "setAdjustMode() : Adjust mode is 0 now\n";

    // }

    // std::cout << "setLDOn() : Set LD on\n";

    // //calibration mode
    // instance->FWCMD(1, 16);

    // instance->FWCMD(1, 4); // ld turn on

    // std::cout << "setLDOn() : Set LD on222222\n";




    // instance->printWholeParameter();

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // StopWatch sw;
    // instance->FWCMD(1, 13);
    // instance->StartStreaming();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 10)
    // {

    // }


    // int a=0,b=0,c=0,d=0;
    // instance->FWGetYRLIPAddress(a, b, c, d);
    // instance->FWSetYRLIPAddress(192, 168, 1, 250);
    // instance->FWCMD(1, 12);

    // instance->SetIPAddrParam("192.168.1.250");
    // instance->mNI->mTCPSocket.SocketClose();
    // instance->mNI->mTCPSocket.SocketCreate(SOCK_STREAM, IPPROTO_TCP);
    
    // std::this_thread::sleep_for(std::chrono::milliseconds(8000));

    // if (!instance->ReconnectTCP())
    // {
    //     printf("Failed connect to TCP Server!\n");
    // }
    // instance->FWGetYRLIPAddress(a, b, c, d);




    // std::string ip ="192.168.1.54";
    // std::stringstream s(ip);
    // int a,b,c,d; //to store the 4 ints
    // char ch; //to temporarily store the '.'
    // s >> a >> ch >> b >> ch >> c >> ch >> d;
    // std::cout << a << "  " << b << "  " << c << "  "<< d;

    //instance->FWGetYRLInitialSettingParameters();

    // while (1)
    // {

    // }
    // instance->FWGetYRLHorizontalOffset();
    // instance->FWSetYRLHorizontalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalOffset();

    // instance->FWGetYRLVerticalOffset();
    // instance->FWSetYRLVerticalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLVerticalOffset();

    // instance->FWCMD(1,11);
    // instance->mNI->mTCPSocket.SocketClose();
    // instance->mNI->mTCPSocket.SocketCreate(SOCK_STREAM, IPPROTO_TCP);
    // if (!instance->ReconnectTCP())
    // {
    //     printf("Failed connect to TCP Server!\n");
    // }

    // instance->FWGetYRLAdjustMode();
    // instance->FWSetYRLAdjustMode(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLAdjustMode();
    // instance->FWCMD(1, 13);
    // instance->StartUDP();
    // std::cout << "===================================================================="<<std::endl;
    // StopWatch sw;
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 20)
    // {
        
    // }
    
    // instance->FWCMD(1, 6);
    // std::cout << "===================================================================="<<std::endl;
    // instance->FWGetYRLAdjustMode();
    // instance->FWSetYRLAdjustMode(1);
    // instance->FWCMD(1, 9);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    // instance->FWGetYRLAdjustMode();
    // std::cout << "===================================================================="<<std::endl;
    // instance->FWGetYRLAdjustMode();
    // instance->FWSetYRLAdjustMode(1);
    // instance->FWCMD(1, 9);
    // //std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    // instance->FWGetYRLAdjustMode();
    // instance->FWCMD(1, 13);
    // instance->StartUDP();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 20)
    // {
        
    // }

    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26001);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();
    // std::cout << "===================================================================="<<std::endl;
    // instance->FWCMD(1, 13);
    // instance->StartUDP();
    // StopWatch sw;
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 20)
    // {
        
    // }
    // instance->FWCMD(1, 6);
    // std::this_thread::sleep_for(std::chrono::milliseconds(20000));
    // std::cout << "===================================================================="<<std::endl;
    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26000);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();
    // std::cout << "===================================================================="<<std::endl;
    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26000);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();
    // instance->FWCMD(1, 13);
    // instance->StartUDP();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 20)
    // {
        
    // }



    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26000);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();
    // std::cout << "===================================================================="<<std::endl;
    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26001);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();









    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26001);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();

    // instance->FWGetYRLSiPMGain();
    // instance->FWSetYRLSiPMGain(3501);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMGain();

    // instance->FWGetYRLLDVoltage();
    // instance->FWSetYRLLDVoltage(30001);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLLDVoltage();

    // instance->FWGetYRLLDPulseWidth();
    // instance->FWSetYRLLDPulseWidth(1);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLLDPulseWidth();

    // instance->FWGetYRLAdjustMode();
    // instance->FWSetYRLAdjustMode(1);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLAdjustMode();

    // instance->FWGetYRLSafetyRotationSpeed();
    // instance->FWSetYRLSafetyRotationSpeed(1);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSafetyRotationSpeed();

    // instance->FWGetYRLVerticalAngleLower();
    // instance->FWSetYRLVerticalAngleLower(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalAngleLower();

    // instance->FWGetYRLVerticalAngleUpper();
    // instance->FWSetYRLVerticalAngleUpper(84);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalAngleUpper();

    // instance->FWGetYRLVerticalSpeed();
    // instance->FWSetYRLVerticalSpeed(81);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalSpeed();

    // instance->FWGetYRLVerticalOffsetLower();
    // instance->FWSetYRLVerticalOffsetLower(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalOffsetLower();

    // instance->FWGetYRLVerticalOffsetUpper();
    // instance->FWSetYRLVerticalOffsetUpper(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalOffsetUpper();

    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable ();
    // instance->FWGetYRLCompTable ();
    // instance->FWSetYRLTableSize(40);
    // for (int i = 100000; i < 100040; i++)
    // {
    //     instance->FWSetYRLWidthTable (i);
    // }
    // for (int i = 100000; i < 100040; i++)
    // {
    //     instance->FWSetYRLCompTable (i);
    // }
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable ();
    // instance->FWGetYRLCompTable ();

    // instance->FWGetYRLSerialNumber1();
    // instance->FWSetYRLSerialNumber1(1);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber1();

    // instance->FWGetYRLSerialNumber2();
    // instance->FWSetYRLSerialNumber2(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber2();
    
    // instance->FWGetYRLSerialNumber3();
    // instance->FWSetYRLSerialNumber3(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber3();

    // instance->FWGetYRLModelNumber();
    // instance->FWSetYRLModelNumber(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLModelNumber();

    // instance->FWGetYRLHorizontalSpeed();
    // instance->FWSetYRLHorizontalSpeed(49501);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalSpeed();

    // instance->FWGetYRLHorizontalOffset();
    // instance->FWSetYRLHorizontalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalOffset();

    // instance->FWGetYRLVerticalOffset();
    // instance->FWSetYRLVerticalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLVerticalOffset();



    // instance->FWSetYRLSiPMVoltage(26000);
    // instance->FWSetYRLSiPMGain(3500);
    // instance->FWSetYRLLDVoltage(30000);
    // instance->FWSetYRLLDPulseWidth(2);
    // instance->FWSetYRLAdjustMode(1);
    // instance->FWSetYRLSafetyRotationSpeed(0);
    // instance->FWSetYRLVerticalAngleLower(5);
    // instance->FWSetYRLVerticalAngleUpper(85);
    // instance->FWSetYRLVerticalSpeed(80);
    // instance->FWSetYRLVerticalOffsetLower(0);
    // instance->FWSetYRLVerticalOffsetUpper(0);
    // instance->FWSetYRLTableSize(42);
    // for (int i = 100000; i < 100042; i++)
    // {
    //     instance->FWSetYRLWidthTable (i);
    // }
    // for (int i = 100000; i < 100042; i++)
    // {
    //     instance->FWSetYRLCompTable (i);
    // }
    // instance->FWCMD(1, 9);
    // instance->FWSetYRLSerialNumber1(1);
    // instance->FWSetYRLSerialNumber2(1);
    // instance->FWSetYRLSerialNumber3(1);
    // //instance->FWSetYRLIPAddress(int value1, int value2, int value3, int value4);
    // instance->FWSetYRLModelNumber(1);
    // instance->FWSetYRLHorizontalSpeed(49500);
    // instance->FWSetYRLHorizontalOffset(0);
    // instance->FWSetYRLVerticalOffset(0);
    // instance->FWCMD(1, 12);
    // std::cout << "===================================================================="<<std::endl;

    // instance->FWGetYRLSiPMVoltage();
    // instance->FWGetYRLSiPMGain();
    // instance->FWGetYRLLDVoltage();
    // instance->FWGetYRLLDPulseWidth();
    // instance->FWGetYRLAdjustMode();
    // instance->FWGetYRLSafetyRotationSpeed();
    // instance->FWGetYRLVerticalAngleLower();
    // instance->FWGetYRLVerticalAngleUpper();
    // instance->FWGetYRLVerticalSpeed();
    // instance->FWGetYRLVerticalOffsetLower();
    // instance->FWGetYRLVerticalOffsetUpper();
    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable ();
    // instance->FWGetYRLCompTable ();
    // instance->FWGetYRLSerialNumber1();
    // instance->FWGetYRLSerialNumber2();
    // instance->FWGetYRLSerialNumber3();
    // int a=0,b=0,c=0,d=0;
    // instance->FWGetYRLIPAddress(a, b, c, d);
    // instance->FWGetYRLModelNumber();
    // instance->FWGetYRLHorizontalSpeed();
    // instance->FWGetYRLHorizontalOffset();
    // instance->FWGetYRLVerticalOffset();
    // instance->FWGetYRLMAC1();
    // instance->FWGetYRLMAC2();
    // unsigned char mac[6] = {0};
    // mac[0] = 0xff & (instance->mLidarParam.MAC1 >> 0);
    // mac[1] = 0xff & (instance->mLidarParam.MAC1 >> 8);
    // mac[2] = 0xff & (instance->mLidarParam.MAC1 >> 16);
    // mac[3] = 0xff & (instance->mLidarParam.MAC1 >> 24);
    // mac[4] = 0xff & (instance->mLidarParam.MAC2 >> 0);
    // mac[5] = 0xff & (instance->mLidarParam.MAC2 >> 8);
    // //sprintf(mac2, "%x", mac);
    // //printf("23~24. MAC: %s\n", mac2);
    // printf("23~24. MAC: ");
    // for (int i = 0; i < 6 ; i++)
    // {
    //     printf("%x ", mac[i]);
    // }
    // printf("\n");
    // instance->FWGetYRLEthFWMajor();
    // instance->FWGetYRLEthFWMinor();
    // instance->FWGetYRLEthFWRevision();
    // instance->FWGetYRLTDCFWMajor();
    // instance->FWGetYRLTDCFWMinor();
    // instance->FWGetYRLTDCFWRevision();
    // instance->FWCMD(1, 13);
    // std::cout << "===================================================================="<<std::endl;
    // instance->StartUDP();

    // int timer(0);
    // int period(1);
    // int total_loop(10000000 * period);
    // //int total_loop(11000000 * period);
    
    // std::vector <float> intensity;
    // std::vector <float> coord_x;
    // std::vector <float> coord_y;
    // std::vector <float> coord_z;




    


    //std::vector<float> intensity1, range1, horizontal_a1, vertical_a1;


    

/*
    while (1)
    {
     
        timer += period;
        //instance->GetSphericalOutputsWithIntensity(intensity1, range1, horizontal_a1, vertical_a1);
        //int size_buffer1 = range1.size();
        
        //std::cout << "size of range array: " << size_buffer1 << '\n';

        //std::this_thread::sleep_for(std::chrono::milliseconds(period));
        if (timer == total_loop)
        {
            break;
        }
    
    }
  */  

    delete instance;
    return 0;
}