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

char generate_crc(uint8_t *pdata, char length)
{
    char crc = 0x0;
    for (int i = 0; i < length; i++)
    {
        //    printf("crc : %x\r\n", pdata[i]);
        crc ^= pdata[i];
    }
    return crc;
}

int main( int argc, char ** argv)
{
    std::cout << "========================================" << std::endl; 
    std::cout << "           Test YRL driver          " << std::endl; 
    std::cout << "========================================\n" << std::endl;
 
    YujinRobotYrlDriver* mDriver = new YujinRobotYrlDriver();
    
    int result;
    int adjust_mode;
    StopWatch sw;
    static YRLUDPSocket socket("UDP socket for Cali");
    static std::vector< unsigned char > send_buffer(24, 0);
    static std::vector< unsigned char > recv_buffer(4096*4, 0);
    static uint8_t packetID = 1;
    static int count_timeout = 0;
    
    // /// 1. function_Start()
    // /// calibration start
    
    // /// 2. function_InitialPosition()
    // /// moving jig

    // /// 3. function_InitialAngle
    // /// moving jig

    // /// ============================================================================================================
    // /// ex> 4 (LiDAR 1) -> 5 (LiDAR 1) -> 4 (LiDAR 2) -> 5 (LiDAR 2) -> 4 (LiDAR 3) ..... repeated by num of LiDARS
    

    // /// 4. function_SequentialPowerOn()
    // /// power on LiDAR #

    // /// 5. function_SequentialSetIP()
    // /// 5-1. change IP from 250 to 11
    // mDriver->StopTCP();//close TCP socket
    // mDriver->SetIPAddrParamPortNumParam("192.168.1.250", 1234);
    // result = mDriver->StartTCP();//create TCP socket and connect to server
    // if (result == -1)
    // {
    //     printf("TCP is not connected. try again!\n");
    //     return 1;
    // }
    // mDriver->FWCMD(1, 14);

    // mDriver->FWSetYRLIPAddress(192, 168, 1, 11);
    // mDriver->FWCMD(1, 12);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    // /// 5-2. IP connect to LiDAR
    // mDriver->SetIPAddrParam("192.168.1.11");
    // mDriver->StopTCP();
    // result = mDriver->StartTCP();
    // if (result == -1)
    // {
    //     printf("TCP is not connected. try again!\n");
    //     return 1;
    // }
    // mDriver->FWCMD(1, 14);
    // /// END 4->5 ============================================================================================================




    // /// ============================================================================================================
    // /// ex> 6 (LiDAR 1) -> 6 (LiDAR 2) -> 6 (LiDAR 3) -> ..... repeated by num of LiDARS

    // /// 6.function_InitialParamSetting()
    // /// 6-1. IP connect to LiDAR
    // mDriver->SetIPAddrParam("192.168.1.11");
    // mDriver->StopTCP();
    // result = mDriver->StartTCP();
    // if (result == -1)
    // {
    //     printf("TCP is not connected. try again!\n");
    //     return 1;
    // }
    // mDriver->FWCMD(1, 14);

    // /// 6-2. change adjust mode to 0 and check
    // mDriver->FWGetYRLAdjustMode();
    // adjust_mode = mDriver->GetOutputModeParam();
    // if (adjust_mode == 1)
    // {
    //     mDriver->FWSetYRLAdjustMode(0);
    //     mDriver->FWCMD(1, 9);
    //     mDriver->FWCMD(1, 14);
        
    //     mDriver->FWGetYRLAdjustMode();
    //     adjust_mode = mDriver->GetOutputModeParam();
    //     if (adjust_mode == 1)
    //     {
    //         //FAIL!!
    //         return 1;
    //     }
    //     else { }
    // }

    // //calibration mode 
    // mDriver->FWCMD(1,16);
    
    // /// 6-3. Set LD ON
    // mDriver->FWCMD(1, 4);

    // /// 6-4. Set vertical angle through UI buttons
    // /// ex> 
    // mDriver->FWCMD(2, 3); //center
    // mDriver->FWCMD(2, 1); //up
    // mDriver->FWCMD(2, 1); //up
    // mDriver->FWCMD(2, 1); //up
    // mDriver->FWCMD(2, 2); //down 
    // mDriver->FWCMD(2, 2); //down 
    // mDriver->FWCMD(2, 2); //down 


    // /// 6-5. Set LD Off
    // mDriver->FWCMD(1, 8);
    // /// END 6 ============================================================================================================



    // /// 7. function_StartPosition()
    // /// moving jig

    // /// ============================================================================================================
    // /// 8 -> ... -> 13 : this repeated by num of LiDARS

    /// 8. function_GetReadyCalibration()
    /// 8-1. IP connect to LiDAR
    mDriver->SetIPAddrParam("192.168.1.11");
    mDriver->StopTCP();
    result = mDriver->StartTCP();
    if (result == -1)
    {
        printf("TCP is not connected. try again!\n");
        return 1;
    }
    mDriver->FWCMD(1, 14);

    //calibration mode 
    mDriver->FWCMD(1,16);

    /// 8-2. Set LD ON
    mDriver->FWCMD(1, 4);

    /// 8-3. UDP start
    //mDriver->FWCMD(1, 13);
    mDriver->FWCMD(1, 17);


    /// ======================================================================
    /// ex> 9 (LiDAR 1) -> 10 (LiDAR 1) -> 9 (LiDAR 1) -> 10 (LiDAR 1) -> ... 
    /// repeated by (num of panels * max dist / resolution)
    
    /// 9. function_Calibration()
    /// moving jig

    /// 10. function_GetLidarData()
    /// get Data through UDP

    send_buffer.assign(24, 0);
    recv_buffer.assign(4096 * 4, 0);

    send_buffer[0] = 0x02;
    send_buffer[1] = 0x02;
    send_buffer[2] = 0x02;
    send_buffer[3] = 0x02;
    send_buffer[11] = 0x02;
    send_buffer[12] = 0x12;
    send_buffer[13] = 0x23;
    send_buffer[14] = 8 & 0x000000ff;
    send_buffer[15] = packetID;//1~255
    send_buffer[22] = generate_crc(&send_buffer[14], 8);
    send_buffer[23] = 0x03;

    //if packet id exceeded 255, then initialize it to zero.
    if(packetID >= 0xff)
    {
        packetID = 1;
    }
    else
    {
        packetID++;
    }

    result = socket.WriteTo( send_buffer.data(), send_buffer.size(), "192.168.1.11", 1234 );
    if (result < 0)
    {
        return 1;
    }
    else
    {
        int no_read = socket.ReadWithTimeout( recv_buffer.data(), recv_buffer.size(), 1000 );
        unsigned char * ptr( recv_buffer.data() );
        printf("no_read: %d\n", no_read);
        if( no_read <= 0 )
        {
            return 1;
        }
        else { }
    }
    /// END 9->10 ======================================================================


    /// 11. function_GetReadyVerification()
    /// 11-1. reboot
    mDriver->FWCMD(1, 6);//main reboot
    mDriver->FWCMD(1,14);
    /// 11-2. set width table and comp table
    /// ex>
    mDriver->FWSetYRLTableSize(40);
    for (int i = 1; i <= 40; i++)
    {
        mDriver->FWSetYRLWidthTable (i);
    }
    for (int i = 1; i <= 40; i++)
    {
        mDriver->FWSetYRLCompTable (i);
    }
    mDriver->FWCMD(1, 9);
    mDriver->FWCMD(1,14);

    /// 11-3. get width table and comp table for check
    std::vector <int32_t> width_table;
    std::vector <int32_t> comp_table;
    mDriver->FWGetYRLTableSize();
    mDriver->FWGetYRLWidthTable (width_table);
    mDriver->FWGetYRLCompTable (comp_table);

    /// 11-4. change adjust mode to 1 and check
    mDriver->FWGetYRLAdjustMode();
    adjust_mode = mDriver->GetOutputModeParam();
    if (adjust_mode == 0)
    {
        mDriver->FWSetYRLAdjustMode(1);
        mDriver->FWCMD(1, 9);
        mDriver->FWCMD(1, 14);
        
        mDriver->FWGetYRLAdjustMode();
        adjust_mode = mDriver->GetOutputModeParam();
        if (adjust_mode == 0)
        {
            //FAIL
            return 1;
        }
        else { }
    }

    //calibration mode 
    mDriver->FWCMD(1,16);

    /// 11-5. set LD on
    mDriver->FWCMD(1, 4);

    /// 11-6. UDP start
    //mDriver->FWCMD(1, 13);
    mDriver->FWCMD(1, 17);

    /// ======================================================================
    /// ex> 12 (LiDAR 1) -> 10 (LiDAR 1) -> 12 (LiDAR 1) -> 10 (LiDAR 1) -> ... 
    /// repeated by (num of panels * max dist / resolution)

    /// 12. function_Verification()
    /// moving jig

    /// 10. function_GetLidarData()
    /// get Data through UDP
    send_buffer.assign(24, 0);
    recv_buffer.assign(4096 * 4, 0);

    send_buffer[0] = 0x02;
    send_buffer[1] = 0x02;
    send_buffer[2] = 0x02;
    send_buffer[3] = 0x02;
    send_buffer[11] = 0x02;
    send_buffer[12] = 0x12;
    send_buffer[13] = 0x23;
    send_buffer[14] = 8 & 0x000000ff;
    send_buffer[15] = packetID;//1~255
    send_buffer[22] = generate_crc(&send_buffer[14], 8);
    send_buffer[23] = 0x03;

    //if packet id exceeded 255, then initialize it to zero.
    if(packetID >= 0xff)
    {
        packetID = 1;
    }
    else
    {
        packetID++;
    }

    result = socket.WriteTo( send_buffer.data(), send_buffer.size(), "192.168.1.11", 1234 );
    if (result < 0)
    {
        return 1;
    }
    else
    {
        int no_read = socket.ReadWithTimeout( recv_buffer.data(), recv_buffer.size(), 1000 );
        unsigned char * ptr( recv_buffer.data() );
        printf("no_read: %d\n", no_read);
        if( no_read <= 0 )
        {
            return 1;
        }
        else { }
    }
    /// END 10->12 ======================================================================
    while(1){}


    /// 13-1. reboot
    mDriver->FWCMD(1, 6); //main reboot
    
    /// 13-2. Set LD Off
    mDriver->FWCMD(1, 8);

    /// Go back to 8
    /// END 8->...->13============================================================================================================

    delete mDriver;
    return 0;
}