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
        LOGPRINT(main, YRL_LOG_USER, ("WRONG ARGUMENT! USAGE: [./test_fw_set_get_param] [IP_ADDRESS]\n"));
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

    ///=== SET/GET FW PARAMETERS (1 ~ 14) =======================================================
    

    // // 1. SiPM Voltage
    // //=============================================
    // instance->FWGetYRLSiPMVoltage();
    
    // instance->FWSetYRLSiPMVoltage(26001);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLSiPMVoltage();
    // //=============================================

    // // 2. SiPM Gain
    // //=============================================
    // instance->FWGetYRLSiPMGain();

    // instance->FWSetYRLSiPMGain(5000);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLSiPMGain();
    // //=============================================

    // // 3. LD Voltage
    // //============================================= 
    // instance->FWGetYRLLDVoltage();

    // instance->FWSetYRLLDVoltage(30001);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLLDVoltage();
    // //=============================================
    
    // // 4. LD Pulse Width
    // //============================================= 
    // instance->FWGetYRLLDPulseWidth();
    
    // instance->FWSetYRLLDPulseWidth(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLLDPulseWidth();
    // //=============================================

    // // 5. Adjust Mode
    // //============================================= 
    // instance->FWGetYRLAdjustMode();

    // instance->FWSetYRLAdjustMode(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLAdjustMode();
    // //=============================================

    // // 6. Safety Rotation Speed
    // //=============================================
    // instance->FWGetYRLSafetyRotationSpeed();

    // instance->FWSetYRLSafetyRotationSpeed(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLSafetyRotationSpeed();
    // //=============================================

    // // 7. Vertical Angle Lower
    // //=============================================
    // instance->FWGetYRLVerticalAngleLower();

    // instance->FWSetYRLVerticalAngleLower(0);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLVerticalAngleLower();
    // //=============================================

    // // 8. Vertical Angle Lower
    // //=============================================
    // instance->FWGetYRLVerticalAngleUpper();
    
    // instance->FWSetYRLVerticalAngleUpper(84);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLVerticalAngleUpper();
    // //=============================================

    // // 9. Vertical Speed
    // //=============================================
    // instance->FWGetYRLVerticalSpeed();
    
    // instance->FWSetYRLVerticalSpeed(81);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLVerticalSpeed();
    // //=============================================

    // // 10. Vertical Offset Lower
    // //=============================================
    // instance->FWGetYRLVerticalOffsetLower();
    
    // instance->FWSetYRLVerticalOffsetLower(-10);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLVerticalOffsetLower();
    // //=============================================

    // // 11. Vertical Mode
    // //=============================================
    // instance->FWGetYRLVerticalMode();
    
    // instance->FWSetYRLVerticalMode(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLVerticalMode();
    // //=============================================

    // // 12. Table Size, 13. width table, 14. comp table
    // //=============================================
    // std::vector <int32_t> width_table;
    // std::vector <int32_t> comp_table;

    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable (width_table);
    // instance->FWGetYRLCompTable (comp_table);

    // instance->FWSetYRLTableSize(45); // can be 1 ~84
    // for (int i = 0; i < 84; i++)
    // {
    //     instance->FWSetYRLWidthTable (i);
    // }
    // for (int i = 0; i < 84; i++)
    // {
    //     instance->FWSetYRLCompTable (i);
    // }
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable (width_table);
    // instance->FWGetYRLCompTable (comp_table);
    // //=============================================


    ///==========================================================================================





    ///=== SET/GET FW PARAMETERS (15 ~ 22) ======================================================
        

    // // 15 serial number1
    // //=============================================
    // instance->FWGetYRLSerialNumber1();

    // instance->FWSetYRLSerialNumber1(2);
    // instance->FWCMD(1, 12);

    // instance->FWGetYRLSerialNumber1();
    // //=============================================

    // // 16 serial number2
    // //=============================================
    // instance->FWGetYRLSerialNumber2();

    // instance->FWSetYRLSerialNumber2(2);
    // instance->FWCMD(1, 12);

    // instance->FWGetYRLSerialNumber2();
    // //=============================================

    // // 17 serial number3
    // //=============================================
    // instance->FWGetYRLSerialNumber3();

    // instance->FWSetYRLSerialNumber3(2);
    // instance->FWCMD(1, 12);

    // instance->FWGetYRLSerialNumber3();
    // //=============================================

    // // 19 model no
    // //=============================================
    // instance->FWGetYRLModelNumber();

    // instance->FWSetYRLModelNumber(1);
    // instance->FWCMD(1, 12);
    
    // instance->FWGetYRLModelNumber();
    // //=============================================

    // // 20 horizontal speed
    // //=============================================
    // instance->FWGetYRLHorizontalSpeed();
    
    // instance->FWSetYRLHorizontalSpeed(49501);
    // instance->FWCMD(1, 12);
    
    // instance->FWGetYRLHorizontalSpeed();
    // //=============================================

    // // 21 horizontal offset
    // //=============================================
    // instance->FWGetYRLHorizontalOffset();

    // instance->FWSetYRLHorizontalOffset(-10);
    // instance->FWCMD(1, 12);
    
    // instance->FWGetYRLHorizontalOffset();
    // //=============================================

    // // 22 vertical offset
    // //=============================================
    // instance->FWGetYRLVerticalOffset();
    
    // instance->FWSetYRLVerticalOffset(-10);
    // instance->FWCMD(1, 12);
    
    // instance->FWGetYRLVerticalOffset();
    // //=============================================

    ///==========================================================================================





    ///=== GET FW PARAMETERS (23 ~ 30) ==========================================================


    // // 23. MAC1, 24. MAC2
    // //=============================================
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
    // //=============================================

    // // 25 eth_major
    // instance->FWGetYRLEthFWMajor();
    
    // // 26 eth_minor
    // instance->FWGetYRLEthFWMinor();
    
    // // 27 eth_revision
    // instance->FWGetYRLEthFWRevision();
    
    // // 28 tdc_major
    // instance->FWGetYRLTDCFWMajor();
    
    // // 29 tdc_minor
    // instance->FWGetYRLTDCFWMinor();
    
    // // 30 tdc_revision
    // instance->FWGetYRLTDCFWRevision();


    ///==========================================================================================

    delete instance;
    return 0;
}