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

    //== 1. CREATE DRIVER INSTANCE ===========================================
    YujinRobotYrlDriver* instance = new YujinRobotYrlDriver();
    //========================================================================
    
    //== 2. SET IP ===========================================================
    // THIS MUST BE SET BEFOR CALLING Start().
    instance->SetIPAddrParam("192.168.1.13");
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
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    instance->FWCMD(1, 14);
    //========================================================================

    StopWatch sw;

    instance->FWSetYRLIPAddress(192, 168, 1, 250);
    instance->FWCMD(1, 12);

    

    // instance->FWGetYRLVerticalMode();
    // //instance->FWSetYRLVerticalMode(2);
    // instance->FWCMD(4, 2);
    // //instance->FWCMD(1, 9);
    // //instance->FWCMD(1, 14);
    // instance->FWGetYRLVerticalMode();
    // instance->FWCMD(4, 3);
    // instance->FWGetYRLVerticalMode();
    

    // instance->FWSetYRLTableSize(45);
    // for (int i = 0; i < 84; i++)
    // {
    //     instance->FWSetYRLWidthTable (1);
    // }
    // for (int i = 0; i < 84; i++)
    // {
    //     instance->FWSetYRLCompTable (2);
    // }
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // std::vector <int32_t> width_table;
    // std::vector <int32_t> comp_table;
    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable (width_table);
    // instance->FWGetYRLCompTable (comp_table);

    // printf("\n");
    // for (int i = 0; i < width_table.size(); i++)
    // {
    //     printf("[%d], ", width_table[i]);
    // }
    // printf("\n");
    
    // for (int i = 0; i < comp_table.size(); i++)
    // {
    //     printf("[%d], ", comp_table[i]);
    // }
    // printf("\n");






    // instance->FWGetYRLSafetyRotationSpeed();
    // instance->FWSetYRLSafetyRotationSpeed(0);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    // instance->FWGetYRLSafetyRotationSpeed();


  

    // instance->StopTCP();
    // std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    // instance->SetIPAddrParam("192.168.1.13");

    // ret = instance->StartTCP();
    // if (ret == -1)
    // {
    //     printf("TCP is not connected. try again!\n");


    //     ///TCP Connect error handle
    //     return 1;
    // }
    // instance->FWCMD(1, 14);


    // instance->FWGetYRLVerticalSpeed();
    // instance->FWSetYRLVerticalSpeed(50);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    // instance->FWGetYRLVerticalSpeed();


/*


    void FWGetYRLTableSize();
    void FWGetYRLWidthTable (std::vector<int32_t> &_width_table);
    void FWGetYRLCompTable (std::vector<int32_t> &_comp_table);
    void FWGetYRLSerialNumber1();
    void FWGetYRLSerialNumber2();
    void FWGetYRLSerialNumber3();
    void FWGetYRLIPAddress(int& ip_a, int& ip_b, int& ip_c, int& ip_d);
    void FWGetYRLModelNumber();
    void FWGetYRLHorizontalSpeed();
    void FWGetYRLHorizontalOffset();
    void FWGetYRLVerticalOffset();
    void FWGetYRLMAC1();
    void FWGetYRLMAC2();
    void FWGetYRLEthFWMajor();
    void FWGetYRLEthFWMinor();
    void FWGetYRLEthFWRevision();
    void FWGetYRLTDCFWMajor();
    void FWGetYRLTDCFWMinor();
    void FWGetYRLTDCFWRevision();
*/

    /// 1. set/get parameter (1 ~ 14)
  
    // //= 1. SiPM Voltage ===========================
    // instance->FWGetYRLSiPMVoltage();
    
    // instance->FWSetYRLSiPMVoltage(26001);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLSiPMVoltage();
    // //=============================================

    // //= 2. SiPM Gain ==============================
    // instance->FWGetYRLSiPMGain();

    // instance->FWSetYRLSiPMGain(5000);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLSiPMGain();
    // //=============================================

    // //= 3. LD Voltage ============================= 
    // instance->FWGetYRLLDVoltage();

    // instance->FWSetYRLLDVoltage(30001);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLLDVoltage();
    // //=============================================
    
    // //= 4. LD Pulse Width ========================= 
    // instance->FWGetYRLLDPulseWidth();
    
    // instance->FWSetYRLLDPulseWidth(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLLDPulseWidth();
    // //=============================================

    // //= 5. Adjust Mode ============================ 
    // instance->FWGetYRLAdjustMode();

    // instance->FWSetYRLAdjustMode(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLAdjustMode();
    // //=============================================

    // //= 6. Safety Rotation Speed ==================
    // instance->FWGetYRLSafetyRotationSpeed();

    // instance->FWSetYRLSafetyRotationSpeed(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLSafetyRotationSpeed();
    // //=============================================

    // //= 7. Vertical Angle Lower ===================
    // instance->FWGetYRLVerticalAngleLower();

    // instance->FWSetYRLVerticalAngleLower(5);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLVerticalAngleLower();
    // //=============================================

    // //= 8. Vertical Angle Upper ===================
    // instance->FWGetYRLVerticalAngleUpper();
    
    // instance->FWSetYRLVerticalAngleUpper(85);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLVerticalAngleUpper();
    // //=============================================

    // //= 9. Vertical Speed =========================
    // instance->FWGetYRLVerticalSpeed();
    
    // instance->FWSetYRLVerticalSpeed(81);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);
    
    // instance->FWGetYRLVerticalSpeed();
    // //=============================================

    // //= 10. Vertical Offset Lower =================
    // instance->FWGetYRLVerticalOffsetLower();
    
    // instance->FWSetYRLVerticalOffsetLower(-10);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLVerticalOffsetLower();
    // //=============================================

    // //= 11. Vertical Mode =========================
    // instance->FWGetYRLVerticalMode();
    
    // instance->FWSetYRLVerticalMode(1);
    // instance->FWCMD(1, 9);
    // instance->FWCMD(1, 14);

    // instance->FWGetYRLVerticalMode();
    // //=============================================

    // //= 12. Table Size, 13. width table, 14. comp table  
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


    // start UDP and run UDP mode for 20sec
    //instance->FWCMD(1, 13);
    //instance->StartUDP();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 10)
    // {

    // }

    // // reboot
    // instance->FWCMD(1, 6);//main reboot

    // Again get, set and get 
    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26000);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();

    // // 2
    // instance->FWGetYRLSiPMGain();
    // instance->FWSetYRLSiPMGain(5000);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMGain();

    // // 3
    // instance->FWGetYRLLDVoltage();
    // instance->FWSetYRLLDVoltage(30000);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLLDVoltage();

    // // 4
    // instance->FWGetYRLLDPulseWidth();
    // instance->FWSetYRLLDPulseWidth(2);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLLDPulseWidth();

    // // 5
    // instance->FWGetYRLAdjustMode();
    // instance->FWSetYRLAdjustMode(1);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLAdjustMode();

    // // 6
    // instance->FWGetYRLSafetyRotationSpeed();
    // instance->FWSetYRLSafetyRotationSpeed(67);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSafetyRotationSpeed();

    // // 7
    // instance->FWGetYRLVerticalAngleLower();
    // instance->FWSetYRLVerticalAngleLower(10);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalAngleLower();

    // // 8
    // instance->FWGetYRLVerticalAngleUpper();
    // instance->FWSetYRLVerticalAngleUpper(80);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalAngleUpper();

    // // 9
    // instance->FWGetYRLVerticalSpeed();
    // instance->FWSetYRLVerticalSpeed(70);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalSpeed();

    // // 10
    // instance->FWGetYRLVerticalOffsetLower();
    // instance->FWSetYRLVerticalOffsetLower(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalOffsetLower();

    // // 11
    // instance->FWGetYRLVerticalMode();
    // instance->FWSetYRLVerticalMode(1);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalMode();

    // // 12, 13, 14
    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable (width_table);
    // instance->FWGetYRLCompTable (comp_table);
    // instance->FWSetYRLTableSize(1);
    // for (int i = 0; i < 1; i++)
    // {
    //     instance->FWSetYRLWidthTable (i);
    // }
    // for (int i = 0; i < 1; i++)
    // {
    //     instance->FWSetYRLCompTable (i);
    // }
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable (width_table);
    // instance->FWGetYRLCompTable (comp_table);

    // // start UDP and run UDP mode for 20sec
    // instance->FWCMD(1, 13);
    // instance->StartUDP();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 10)
    // {

    // }


    // /// 2. test set parameter (15 ~ 22) after main reboot (3/1/6)
    
    // // 15
    // instance->FWGetYRLSerialNumber1();
    // instance->FWSetYRLSerialNumber1(2);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber1();

    // // 16
    // instance->FWGetYRLSerialNumber2();
    // instance->FWSetYRLSerialNumber2(2);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber2();
    
    // // 17
    // instance->FWGetYRLSerialNumber3();
    // instance->FWSetYRLSerialNumber3(2);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber3();

    // // 19
    // instance->FWGetYRLModelNumber();
    // instance->FWSetYRLModelNumber(1);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLModelNumber();

    // // 20
    // instance->FWGetYRLHorizontalSpeed();
    // instance->FWSetYRLHorizontalSpeed(49501);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalSpeed();

    // // 21
    // instance->FWGetYRLHorizontalOffset();
    // instance->FWSetYRLHorizontalOffset(-10);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalOffset();

    // // 22
    // instance->FWGetYRLVerticalOffset();
    // instance->FWSetYRLVerticalOffset(-10);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLVerticalOffset();

    // // 23, 24
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

    // // 25
    // instance->FWGetYRLEthFWMajor();
    
    // // 26
    // instance->FWGetYRLEthFWMinor();
    
    // // 27
    // instance->FWGetYRLEthFWRevision();
    
    // // 28
    // instance->FWGetYRLTDCFWMajor();
    
    // // 29
    // instance->FWGetYRLTDCFWMinor();
    
    // // 30
    // instance->FWGetYRLTDCFWRevision();



    // // start UDP and run UDP mode for 20sec
    // instance->FWCMD(1, 13);
    // instance->StartUDP();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 10)
    // {

    // }

    // // reboot
    // instance->FWCMD(1, 6);//main reboot

    // // 15
    // instance->FWGetYRLSerialNumber1();
    // instance->FWSetYRLSerialNumber1(1);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber1();

    // // 16
    // instance->FWGetYRLSerialNumber2();
    // instance->FWSetYRLSerialNumber2(1);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber2();
    
    // // 17
    // instance->FWGetYRLSerialNumber3();
    // instance->FWSetYRLSerialNumber3(1);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber3();

    // // 19
    // instance->FWGetYRLModelNumber();
    // instance->FWSetYRLModelNumber(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLModelNumber();

    // // 20
    // instance->FWGetYRLHorizontalSpeed();
    // instance->FWSetYRLHorizontalSpeed(49500);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalSpeed();

    // // 21
    // instance->FWGetYRLHorizontalOffset();
    // instance->FWSetYRLHorizontalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalOffset();

    // // 22
    // instance->FWGetYRLVerticalOffset();
    // instance->FWSetYRLVerticalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLVerticalOffset();

    // // 23, 24
    // instance->FWGetYRLMAC1();
    // instance->FWGetYRLMAC2();
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

    // // 25
    // instance->FWGetYRLEthFWMajor();
    
    // // 26
    // instance->FWGetYRLEthFWMinor();
    
    // // 27
    // instance->FWGetYRLEthFWRevision();
    
    // // 28
    // instance->FWGetYRLTDCFWMajor();
    
    // // 29
    // instance->FWGetYRLTDCFWMinor();
    
    // // 30
    // instance->FWGetYRLTDCFWRevision();

    // // start UDP and run UDP mode for 20sec
    // instance->FWCMD(1, 13);
    // instance->StartUDP();
    // sw.Start();
    // while (sw.GetTimeElapsedInSec() < 10)
    // {

    // }



    // /// 3. test get and set whole parameter
    // // 1
    // instance->FWGetYRLSiPMVoltage();
    // instance->FWSetYRLSiPMVoltage(26001);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMVoltage();
    
    // // 2
    // instance->FWGetYRLSiPMGain();
    // instance->FWSetYRLSiPMGain(3501);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSiPMGain();
    
    // // 3
    // instance->FWGetYRLLDVoltage();
    // instance->FWSetYRLLDVoltage(30001);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLLDVoltage();
    
    // // 4
    // instance->FWGetYRLLDPulseWidth();
    // instance->FWSetYRLLDPulseWidth(1);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLLDPulseWidth();
    
    // 5
    // instance->FWGetYRLAdjustMode();
    // instance->FWSetYRLAdjustMode(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLAdjustMode();
    
    // // 6
    // instance->FWGetYRLSafetyRotationSpeed();
    // instance->FWSetYRLSafetyRotationSpeed(1);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLSafetyRotationSpeed();
    
    // // 7
    // instance->FWGetYRLVerticalAngleLower();
    // instance->FWSetYRLVerticalAngleLower(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalAngleLower();

    // // 8
    // instance->FWGetYRLVerticalAngleUpper();
    // instance->FWSetYRLVerticalAngleUpper(84);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalAngleUpper();

    // // 9
    // instance->FWGetYRLVerticalSpeed();
    // instance->FWSetYRLVerticalSpeed(81);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalSpeed();

    // // 10
    // instance->FWGetYRLVerticalOffsetLower();
    // instance->FWSetYRLVerticalOffsetLower(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalOffsetLower();

    // // 11
    // instance->FWGetYRLVerticalOffsetUpper();
    // instance->FWSetYRLVerticalOffsetUpper(0);
    // instance->FWCMD(1, 9);
    // instance->FWGetYRLVerticalOffsetUpper();

    // // 12, 13, 14
    // std::vector <int32_t> width_table;
    // std::vector <int32_t> comp_table;
    // instance->FWGetYRLTableSize();
    // instance->FWGetYRLWidthTable (width_table);
    // instance->FWGetYRLCompTable (comp_table);
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
    // instance->FWGetYRLWidthTable (width_table);
    // instance->FWGetYRLCompTable (comp_table);

    // // 15
    // instance->FWGetYRLSerialNumber1();
    // instance->FWSetYRLSerialNumber1(1);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber1();

    // // 16
    // instance->FWGetYRLSerialNumber2();
    // instance->FWSetYRLSerialNumber2(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber2();
    
    // // 17
    // instance->FWGetYRLSerialNumber3();
    // instance->FWSetYRLSerialNumber3(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLSerialNumber3();

    // // 19
    // instance->FWGetYRLModelNumber();
    // instance->FWSetYRLModelNumber(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLModelNumber();

    // // 20
    // instance->FWGetYRLHorizontalSpeed();
    // instance->FWSetYRLHorizontalSpeed(49501);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalSpeed();

    // // 21
    // instance->FWGetYRLHorizontalOffset();
    // instance->FWSetYRLHorizontalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLHorizontalOffset();

    // // 22
    // instance->FWGetYRLVerticalOffset();
    // instance->FWSetYRLVerticalOffset(0);
    // instance->FWCMD(1, 12);
    // instance->FWGetYRLVerticalOffset();

    // // 23, 24
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

    // // 25
    // instance->FWGetYRLEthFWMajor();
    
    // // 26
    // instance->FWGetYRLEthFWMinor();
    
    // // 27
    // instance->FWGetYRLEthFWRevision();
    
    // // 28
    // instance->FWGetYRLTDCFWMajor();
    
    // // 29
    // instance->FWGetYRLTDCFWMinor();
    
    // // 30
    // instance->FWGetYRLTDCFWRevision();






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

    delete instance;
    return 0;
}