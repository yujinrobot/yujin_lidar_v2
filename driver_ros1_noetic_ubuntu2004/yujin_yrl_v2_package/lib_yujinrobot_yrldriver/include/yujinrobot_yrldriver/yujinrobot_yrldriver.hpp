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
#ifndef YUJIN_ROBOT_YRL_DRIVER_HPP
#define YUJIN_ROBOT_YRL_DRIVER_HPP

#ifdef _WIN32
#ifdef YRL3V2DRIVERDLL_EXPORTS
#define YRL3V2DRIVERDLL_API __declspec(dllexport)
#else
#define YRL3V2DRIVERDLL_API __declspec(dllimport)
#endif
#endif

#include "extracode.hpp"
#include "NetworkInterface.hpp"

#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#ifdef _WIN32
#pragma warning(disable : 4251)
#endif

#ifdef _WIN32
class YRL3V2DRIVERDLL_API YujinRobotYrlDriver
#else
class YujinRobotYrlDriver
#endif
{
public:
  LidarParam mLidarParam;
  std::shared_ptr <NetworkInterface> mNI;
  Parameters* mParams;

  bool useRawDataGraph;
  enum RawDataGraphType mGraphType;

protected:
  std::thread mFirstThread;
  std::thread mSecondThread;
  std::atomic <bool> mbRunThread;

  std::mutex mInputDataLocker;
  std::condition_variable mCVInputData;
  std::deque< std::shared_ptr < ValueGroup > > mInputDataDeque;

  std::mutex mOutputDataLocker;
  std::deque< std::shared_ptr < PointDataArray > > mOutputDataDeque;

  LidarStatusData* mLidarStatus;
  std::mutex mStatusDataLocker;

  int mRecordPreHoriAngleCnt;
  std::vector<float> mRanges0;
  std::vector<float> mHAngles;
  std::vector<float> mVAngles;
  std::vector<float> mRisings0;  
  std::vector<float> mIntensities0;
  float mDPR;

  int32_t* mLidarWidthTable;
  int32_t* mLidarCompTable;

  int mNumContinuousReadingFailures;
  int UDPErrorCase;

  std::vector<float> mGraphRisingWidthData;
  std::vector<float> mGraphRangeData;
  std::vector<float> mGraphHorizontalCountData;
  std::vector<float> mGraphVerticalCountData;
  std::mutex mRawGraphLocker;

  StopWatch clockPerformance;
  int cntCommThread;
  int cntPrcsThread;
  int cntFinalOutput;

public:
  YujinRobotYrlDriver ();
  ~YujinRobotYrlDriver ();

  int Start ();
  void StartThreads ();
  int StartTCP ();
  bool IsTCPConnected ();
  void StopTCP ();
  bool ReconnectTCP ();
  void RecoveryNetwork ();

protected:   
  void deleteNetworkInterface ();
  void createNetworkInterface ();
  void removeThreads ();
  int seeDataStructure ( unsigned char * ptr, unsigned int numData, unsigned int& time_stamp, std::vector< unsigned int > & values );
  void getScanningData ();
  void dataProcessing (std::shared_ptr < ValueGroup > &ptr);
  void threadedFunction1 ();
  void threadedFunction2 ();

public:
  /***************************** 1. Main User Interface for LiDAR *****************************/
  int ConnectTOYRL3V2();
  void DisconnectTOYRL3V2();
  void UserChangeMode(int mode);
  void UserChangeIP(std::string new_ip);
  void StartGettingDataStream();
  void StopGettingDataStreamAndSet();

  /****************************** 2. User Output Data Interface ******************************/
  void GetDPR (float &dpr);
  
  /// *** USE EITHER ONE OF BELOW 3 FUNCTIONS!
  int GetWholeOutput (double &_SystemTime,
                      std::vector<float> &_RangeArray, 
                      std::vector<float> &_HorizontalAngleArray,
                      std::vector<float> &_VerticalAngleArray, 
                      std::vector<float> &_RisingArray, 
                      std::vector<float> &_IntensityArray,
                      std::vector<float> &_XCoordArray, 
                      std::vector<float> &_YCoordArray, 
                      std::vector<float> &_ZCoordArray);
  int GetCartesianOutputsWithIntensity (double &_SystemTime,
                                        std::vector <float>& _IntensityArray,
                                        std::vector <float>& _XCoordArray, 
                                        std::vector <float>& _YCoordArray, 
                                        std::vector <float>& _ZCoordArray);
  int GetSphericalOutputsWithIntensity (double &_SystemTime,
                                        std::vector <float>& _IntensityArray,
                                        std::vector <float>& _RangeArray,
                                        std::vector <float>& _HorizontalAngleArray,
                                        std::vector <float>& _VerticalAngleArray);

  /******************************* 3. User Parameter Interface *******************************/
  void SetIPAddrParam (const std::string &ipAddr);
  void SetPortNumParam (const unsigned short int portNum);
  void SetMinZParam (const float z_min);
  void SetMaxZParam (const float z_max);
  void SetMinYParam (const float y_min);
  void SetMaxYParam (const float y_max);
  void SetMinXParam (const float x_min);
  void SetMaxXParam (const float x_max);
  void SetMinRangeParam (const float range_min);
  void SetMaxRangeParam (const float range_max);
  void SetExtrinsicTransformMatParam (const float x, const float y, const float z,
                                      const float rx, const float ry, const float rz);
  void SetMaxVertiAngleParam (const float verti_angle_max);
  void SetMinVertiAngleParam (const float verti_angle_min);
  void SetMaxHoriAngleParam (const float hori_angle_max);
  void SetMinHoriAngleParam (const float hori_angle_min);
  void SetNoiseFilterLevelParam (const float filter_level);
  std::string GetIPAddrParam ();
  unsigned short int GetPortNumParam ();
  float GetMinZParam ();
  float GetMaxZParam ();
  float GetMinYParam ();
  float GetMaxYParam ();
  float GetMinXParam ();
  float GetMaxXParam ();
  float GetMinRangeParam ();
  float GetMaxRangeParam ();
  void GetExtrinsicTransformParam (float &x, float &y, float &z,
                                  float &rx, float &ry, float &rz);
  float GetMaxVertiAngleParam ();
  float GetMinVertiAngleParam ();
  float GetMaxHoriAngleParam ();
  float GetMinHoriAngleParam ();
  float GetNoiseFilterLevelParam ();
  
  /************************************** DO NOT USE FUNCTIONS BELOW **************************************/
  void SetHoriAngleOffsetParam (const float hori_angle_offset);
  void SetVertiAngleOffsetParam (const float verti_angel_offset);
  void SetOutputModeParam (const int output_mode); //related to fw param
  void SetVerticalModeParam (const float vertical_mode);
  float GetHoriAngleOffsetParam ();
  float GetVertiAngleOffsetParam ();
  int GetOutputModeParam ();
  int GetVerticalModeParam ();

  void StartStreaming ();

  void FWCMD (uint8_t code, int32_t data);
  void FWGetYRLInitialSettingParameters ();

  void FWGetYRLSiPMVoltage ();
  void FWGetYRLSiPMGain ();
  void FWGetYRLLDVoltage ();
  void FWGetYRLLDPulseWidth ();
  void FWGetYRLAdjustMode ();
  void FWGetYRLSafetyRotationSpeed ();
  void FWGetYRLVerticalAngleLower ();
  void FWGetYRLVerticalAngleUpper ();
  void FWGetYRLVerticalSpeed ();
  void FWGetYRLVerticalOffsetLower ();
  void FWGetYRLVerticalMode ();
  void FWGetYRLTableSize ();
  void FWGetYRLWidthTable (std::vector<int32_t> &_width_table);
  void FWGetYRLCompTable (std::vector<int32_t> &_comp_table);
  void FWGetYRLSerialNumber1 ();
  void FWGetYRLSerialNumber2 ();
  void FWGetYRLSerialNumber3 ();
  void FWGetYRLIPAddress (int& ip_a, int& ip_b, int& ip_c, int& ip_d);
  void FWGetYRLModelNumber ();
  void FWGetYRLHorizontalSpeed ();
  void FWGetYRLHorizontalOffset ();
  void FWGetYRLVerticalOffset ();
  void FWGetYRLMAC1 ();
  void FWGetYRLMAC2 ();
  void FWGetYRLEthFWMajor ();
  void FWGetYRLEthFWMinor ();
  void FWGetYRLEthFWRevision ();
  void FWGetYRLTDCFWMajor ();
  void FWGetYRLTDCFWMinor ();
  void FWGetYRLTDCFWRevision ();
  void FWGetYRLFactoryMode ();
  void FWGetYRLMainModelNumber ();

  void FWSetYRLSiPMVoltage (int value);
  void FWSetYRLSiPMGain (int value);
  void FWSetYRLLDVoltage (int value);
  void FWSetYRLLDPulseWidth (int value);
  void FWSetYRLAdjustMode (int value);
  void FWSetYRLSafetyRotationSpeed (int value);
  void FWSetYRLVerticalAngleLower (int value);
  void FWSetYRLVerticalAngleUpper (int value);
  void FWSetYRLVerticalSpeed (int value);
  void FWSetYRLVerticalOffsetLower (int value);
  void FWSetYRLVerticalMode (int value);
  void FWSetYRLTableSize (int value);
  void FWSetYRLWidthTable (int value);
  void FWSetYRLCompTable (int value);
  void FWSetYRLSerialNumber1 (int value);
  void FWSetYRLSerialNumber2 (int value);
  void FWSetYRLSerialNumber3 (int value);
  void FWSetYRLIPAddress (int value1, int value2, int value3, int value4);
  void FWSetYRLModelNumber (int value);
  void FWSetYRLHorizontalSpeed (int value);
  void FWSetYRLHorizontalOffset (int value);
  void FWSetYRLVerticalOffset (int value);
  void FWSetYRLFactoryMode (int value);
  void FWSetYRLMainModelNumber (int value);
  
  void printWholeParameter ();
  void saveWidthTableParameter1 (unsigned char * recv_buffer, const int table_size);
  void saveWidthTableParameter2 (unsigned char * recv_buffer, const int table_size);
  void printWidthTableParameter ();
  void saveCompTableParameter1 (unsigned char * recv_buffer, const int table_size);
  void saveCompTableParameter2 (unsigned char * recv_buffer, const int table_size);
  void printCompTableParameter ();

  void GetStatusData (float &v_pd, float &v_ld, float &t_pd,
                      float &t_mcu, float &v_target_pd, float &rpm,
                      float &vertical_speed, float &sample_rate);
  void GetErrorCode (unsigned int &origin_info);
  int GetUDPErrorCase ();
  
  bool getuseRawDataGraph_UI ();
  int getGraphDataRisingWidth ( std::vector< float > & data );
  int getGraphDataRange ( std::vector< float > & data );
  int getGraphDataHorizontalCounts ( std::vector< float > & data );
  int getGraphDataVerticalCounts (std::vector< float > & data );
};
#endif // YUJIN_ROBOT_YRL_DRIVER_HPP