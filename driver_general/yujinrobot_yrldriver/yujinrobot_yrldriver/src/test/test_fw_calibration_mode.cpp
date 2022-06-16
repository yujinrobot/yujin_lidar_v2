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

typedef struct verticalmodetest
{
    bool pass_mode_test;
    float min_hori_speed;
    float max_hori_speed;
    float min_verti_speed;
    float max_verti_speed;
    float min_sample_rate;
} VerticalModeTest;

typedef struct additionaltest
{
    VerticalModeTest test[4];
} AdditionalTest;

AdditionalTest lidar_additional_test_result[6];
YujinRobotYrlDriver* mDriver;
StopWatch sw;
int mode_err_cnt[4] = {0, };
int mode_test_cnt[4] = {0, };

std::vector<float> hori_speed_arr;
std::vector<float> verti_speed_arr;
std::vector<float> sample_rate_arr;

void function_AdditionalTest()
{
    static int step = 0;
    //static QTime temp_timer;
    static int mode = 0;
    static bool mode_setting_last = false;
    static bool do_test_again = false;
    static int cnt_do_test_again = 0;

    switch (step)
    {
    case 0:
        printf("TEST STARTED.\n");
        step = 1;
        break;

    case 1:
    {
        mDriver->StopTCP();// disconnect last Lidar
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        int result = 0;
        int cnt = 0;
        do
        {
            result = mDriver->StartTCP();
            if (result == -1)
            {
                printf("TCP is not connected. try again!\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            cnt++;
        } while(result == -1 && cnt < 3);

        if (result == -1)
        {
            printf("Start TCP Failed!\n");
            step = 1;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        mDriver->FWCMD(1, 14);

        step = 2;
        break;
    }
    case 2:
    {
        if (!do_test_again)
        {
            mode++;
        }

        if (mode > 4)
        {
            mode = 1;
            mode_setting_last = true;
        }

        /// set scanning mode
        mDriver->FWSetYRLVerticalMode(mode);

        mDriver->FWCMD(1, 9);
        mDriver->FWCMD(1, 6);
        mDriver->FWCMD(1, 14);

        mDriver->FWGetYRLVerticalMode();
        mDriver->FWGetYRLVerticalSpeed();
        mDriver->FWGetYRLVerticalAngleLower();
        mDriver->FWGetYRLVerticalAngleUpper();

        int vertical_mode = mDriver->GetVerticalModeParam();
        int vertical_speed = mDriver->mLidarParam.vertical_speed;
        int vertical_angle_lower = mDriver->mLidarParam.vertical_lower;
        int vertical_angle_upper = mDriver->mLidarParam.vertical_upper;

        if (vertical_mode == mode)
        {
            //qDebug() << "function_AdditionalTest() : Succeeded to set vertical mode to " << mode;
        }
        else
        {
            //qDebug() << "function_AdditionalTest() : Failed to set vertical mode to " << mode;
        }
        //qDebug() << "function_AdditionalTest() : Vertical mode: " << vertical_mode << ", vertical speed: " << vertical_speed << ", vertical angle lower: " << vertical_angle_lower << ", vertical angle upper: " << vertical_angle_upper;

        if (mode_setting_last)
        {
            mode_setting_last = false;
            step = 6;
        }
        else
        {
            step = 3;
        }
        break;
    }
    case 3:
        /// UDP start
        mDriver->FWCMD(1, 13);
        mDriver->StartStreaming();

        std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //wait for socket buffer clearing

        hori_speed_arr.clear();
        verti_speed_arr.clear();
        sample_rate_arr.clear();

        sw.Start();
        step = 4;
        break;

    case 4:
    {
        // rpm
        // v_speed/10
        // sample_rate
        // dpr/1000
        if (sw.GetTimeElapsedInSec() > 30)
        { 
            float min_hori_speed = 9999;
            float max_hori_speed = -9999;

            float min_verti_speed = 9999;
            float max_verti_speed = -9999;

            float min_sample_rate = 999999;

            for (int i = 0; i < hori_speed_arr.size(); i++)
            {
                if (min_hori_speed > hori_speed_arr[i])
                {
                    min_hori_speed = hori_speed_arr[i];
                }

                if (max_hori_speed < hori_speed_arr[i])
                {
                    max_hori_speed = hori_speed_arr[i];
                }
            }
            for (int i = 0; i < verti_speed_arr.size(); i++)
            {
                if (min_verti_speed > verti_speed_arr[i])
                {
                    min_verti_speed = verti_speed_arr[i];
                }

                if (max_verti_speed < verti_speed_arr[i])
                {
                    max_verti_speed = verti_speed_arr[i];
                }
            }
            for (int i = 0; i < sample_rate_arr.size(); i++)
            {
                if (min_sample_rate > sample_rate_arr[i])
                {
                    min_sample_rate = sample_rate_arr[i];
                }
            }

            lidar_additional_test_result[0].test[mode - 1].min_hori_speed = (1 / max_hori_speed) * 1000;
            lidar_additional_test_result[0].test[mode - 1].max_hori_speed = (1 / min_hori_speed) * 1000;
            lidar_additional_test_result[0].test[mode - 1].min_verti_speed = min_verti_speed * 0.1;
            lidar_additional_test_result[0].test[mode - 1].max_verti_speed = max_verti_speed * 0.1;
            lidar_additional_test_result[0].test[mode - 1].min_sample_rate = min_sample_rate * 1000;

            hori_speed_arr.clear();
            verti_speed_arr.clear();
            sample_rate_arr.clear();

            step = 5;
            sw.Start();
            break;
        }

        float v_pd;
        float v_ld;
        float t_pd;
        float t_mcu;
        float v_target_pd;
        float rpm;
        float vertical_speed;
        float sample_rate;

        double SystemTime;
        std::vector <float> IntensityArray;
        std::vector <float> XCoordArray;
        std::vector <float> YCoordArray;
        std::vector <float> ZCoordArray;

        mDriver->GetStatusData(v_pd, v_ld, t_pd, t_mcu, v_target_pd, rpm, vertical_speed, sample_rate);

        mDriver->GetCartesianOutputsWithIntensity (SystemTime, IntensityArray, XCoordArray, YCoordArray, ZCoordArray);//meaningless




        hori_speed_arr.push_back(rpm);
        verti_speed_arr.push_back(vertical_speed);
        sample_rate_arr.push_back(sample_rate);
        break;
    }
    case 5:

        // qDebug() << "function_AdditionalTest() : Result of LiDAR " << lidar_number + 1 ;
        // qDebug() << "function_AdditionalTest() : Mode " << mode << " result";
        // qDebug() << "function_AdditionalTest() : 1-a. Min of horizontal speed: " << lidar_additional_test_result[lidar_number].test[mode - 1].min_hori_speed  << "Hz";
        // qDebug() << "function_AdditionalTest() : 1-b. Max of horizontal speed of LiDAR: " << lidar_additional_test_result[lidar_number].test[mode - 1].max_hori_speed << "Hz";
        // qDebug() << "function_AdditionalTest() : 2-a. Min of vertical speed of LiDAR: " << lidar_additional_test_result[lidar_number].test[mode - 1].min_verti_speed << "Hz";
        // qDebug() << "function_AdditionalTest() : 2-b. Max of vertical speed of LiDAR: " << lidar_additional_test_result[lidar_number].test[mode - 1].max_verti_speed << "Hz";
        // qDebug() << "function_AdditionalTest() : 3. Min of sample rate of LiDAR: " << lidar_additional_test_result[lidar_number].test[mode - 1].min_sample_rate;

        if (mode == 4)
        {
            if (lidar_additional_test_result[0].test[mode - 1].min_hori_speed >= 19
                    && lidar_additional_test_result[0].test[mode - 1].min_hori_speed <= 21
                    && lidar_additional_test_result[0].test[mode - 1].max_hori_speed >= 19
                    && lidar_additional_test_result[0].test[mode - 1].max_hori_speed <= 21
                    && lidar_additional_test_result[0].test[mode - 1].min_verti_speed >= 9.5
                    && lidar_additional_test_result[0].test[mode - 1].min_verti_speed <= 11.9
                    && lidar_additional_test_result[0].test[mode - 1].max_verti_speed >= 9.5
                    && lidar_additional_test_result[0].test[mode - 1].max_verti_speed <= 11.9
                    && lidar_additional_test_result[0].test[mode - 1].min_sample_rate >= 35000)
            {
                lidar_additional_test_result[0].test[mode - 1].pass_mode_test = true;
                
                //qDebug() << "function_AdditionalTest() : Additional test succeeded LiDAR " << lidar_number + 1 << ", mode " << mode;
            }
            else
            {
                lidar_additional_test_result[0].test[mode - 1].pass_mode_test = false;
                printf("function_AdditionalTest() : MODE %d TEST FAILED\n", mode);
                printf("1-a. Min of horizontal speed: %f\n", lidar_additional_test_result[0].test[mode - 1].min_hori_speed);
                printf("1-b. Max of horizontal speed: %f\n", lidar_additional_test_result[0].test[mode - 1].max_hori_speed);
                printf("2-a. Min of vertical speed: %f\n", lidar_additional_test_result[0].test[mode - 1].min_verti_speed);
                printf("2-b. Max of vertical speed: %f\n", lidar_additional_test_result[0].test[mode - 1].max_verti_speed);
                printf("3. Min of sample rate: %f\n", lidar_additional_test_result[0].test[mode - 1].min_sample_rate);

                //qDebug() << "function_AdditionalTest() : Additional test failed LiDAR " << lidar_number + 1 << ", mode " << mode;
            }
        }
        else
        {
            if (lidar_additional_test_result[0].test[mode - 1].min_hori_speed >= 19
                    && lidar_additional_test_result[0].test[mode - 1].min_hori_speed <= 21
                    && lidar_additional_test_result[0].test[mode - 1].max_hori_speed >= 19
                    && lidar_additional_test_result[0].test[mode - 1].max_hori_speed <= 21
                    && lidar_additional_test_result[0].test[mode - 1].min_verti_speed >= 14.3
                    && lidar_additional_test_result[0].test[mode - 1].min_verti_speed <= 16.7
                    && lidar_additional_test_result[0].test[mode - 1].max_verti_speed >= 14.3
                    && lidar_additional_test_result[0].test[mode - 1].max_verti_speed <= 16.7
                    && lidar_additional_test_result[0].test[mode - 1].min_sample_rate >= 35000)
            {
                lidar_additional_test_result[0].test[mode - 1].pass_mode_test = true;
                //qDebug() << "function_AdditionalTest() : Additional test succeeded LiDAR " << lidar_number + 1 << ", mode " << mode;
            }
            else
            {
                lidar_additional_test_result[0].test[mode - 1].pass_mode_test = false;
                printf("function_AdditionalTest() : MODE %d TEST FAILED\n", mode);
                printf("1-a. Min of horizontal speed: %f\n", lidar_additional_test_result[0].test[mode - 1].min_hori_speed);
                printf("1-b. Max of horizontal speed: %f\n", lidar_additional_test_result[0].test[mode - 1].max_hori_speed);
                printf("2-a. Min of vertical speed: %f\n", lidar_additional_test_result[0].test[mode - 1].min_verti_speed);
                printf("2-b. Max of vertical speed: %f\n", lidar_additional_test_result[0].test[mode - 1].max_verti_speed);
                printf("3. Min of sample rate: %f\n", lidar_additional_test_result[0].test[mode - 1].min_sample_rate);
                //qDebug() << "function_AdditionalTest() : Additional test failed LiDAR " << lidar_number + 1 << ", mode " << mode;
            }
        }

        mode_test_cnt[mode -1]++;
        if (!lidar_additional_test_result[0].test[mode - 1].pass_mode_test)
        {
            mode_err_cnt[mode - 1]++;

            if (cnt_do_test_again < 1)
            {
                printf("function_AdditionalTest() : Do MODE %d test again\n", mode);
                do_test_again = true;
                cnt_do_test_again++;
            }
            else
            {
                do_test_again = false;
                cnt_do_test_again = 0;
            }
        }
        else
        {
            do_test_again = false;
            cnt_do_test_again = 0;
        }

        /// Lidar reboot
        mDriver->FWCMD(1, 6);
        mDriver->FWCMD(1, 14);

        step = 2;
        break;

    case 6:
        printf("Mode Error Cnt: %d/%d %d/%d %d/%d %d/%d\n", mode_err_cnt[0], mode_test_cnt[0], mode_err_cnt[1], mode_test_cnt[1], mode_err_cnt[2], mode_test_cnt[2], mode_err_cnt[3], mode_test_cnt[3]);
        printf("TEST FINISHED.\n");
        step = 0;
        mode = 0;
        break;

    case 7:
        step = 0;
        break;

    default:
        break;
    }
}

int main( int argc, char ** argv)
{
    std::cout << "========================================" << std::endl; 
    std::cout << "           Test YRL driver          " << std::endl; 
    std::cout << "========================================\n" << std::endl;
 
    if (argc != 2)
    {
        LOGPRINT(main, YRL_LOG_USER, ("WRONG ARGUMENT! USAGE: [./test_mode_change] [IP_ADDRESS]\n"));
        return -1;
    }

    std::string ip = argv[1];

    //== 1. CREATE DRIVER INSTANCE ===========================================
    mDriver = new YujinRobotYrlDriver();
    //========================================================================
    
    //== 2. SET IP ===========================================================
    // THIS MUST BE SET BEFOR CALLING Start().
    // THIS WILL BE USED TO CONNECT LIDAR
    mDriver->SetIPAddrParam(ip);
    //========================================================================

    //== 3. START DRIVER =====================================================
    // THIS FUNCTION SHOULD BE ONLY ONCE CALLED.
    int ret = mDriver->Start();
    if (ret < 0)
    {
        std::string IpAddress = mDriver->GetIPAddrParam();
        int PortNumber = mDriver->GetPortNumParam ();
        LOGPRINT(main, YRL_LOG_USER, ("CANNOT START COMMUNICATION WITH LIDAR.\n"));
        LOGPRINT(main, YRL_LOG_USER, ("CONNECT TO [IP:%s PORT:%d] FAILED. CHECK YOUR NETWORK CONNECTION.\n", IpAddress.c_str(), PortNumber));
        delete mDriver;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    mDriver->FWCMD(1, 14);
    //========================================================================

    while (1)
    {
        function_AdditionalTest();
    }

    delete mDriver;
    return 0;
}