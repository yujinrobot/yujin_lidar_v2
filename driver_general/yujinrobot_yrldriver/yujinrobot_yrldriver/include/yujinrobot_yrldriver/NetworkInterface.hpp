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
#ifndef NETWORK_INTERFACE_HPP
#define NETWORK_INTERFACE_HPP

#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>

#include "YRLUDPSocket.hpp"
#include "YRLTCPSocket.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>	
#include <mmsystem.h>	
#pragma comment(lib, "winmm.lib")	
#endif

#define NUM_BYTES_FOR_HEADER_ID         4
#define NUM_BYTES_FOR_FW_VERSION        2
#define NUM_BYTES_FOR_HW_VERSION        1
#define NUM_BYTES_FOR_TIMESTAMP         4
#define NUM_BYTES_FOR_DES               1
#define NUM_BYTES_FOR_SUB_HEAD          2
#define NUM_BYTES_FOR_FRAME_ID          4
#define NUM_BYTES_FOR_SUB_LENGTH        1
#define NUM_BYTES_FOR_SUB_PID           1
#define NUM_BYTES_FOR_SUB_CMD           1

#define IDX_FW_VERSION   (             0  + NUM_BYTES_FOR_HEADER_ID) //4
#define IDX_HW_VERSION   (IDX_FW_VERSION  + NUM_BYTES_FOR_FW_VERSION)//6
#define IDX_TIME_STAMP   (IDX_HW_VERSION  + NUM_BYTES_FOR_HW_VERSION)//7
#define IDX_DES          (IDX_TIME_STAMP  + NUM_BYTES_FOR_TIMESTAMP) //11
#define IDX_SUB_HEAD     (IDX_DES         + NUM_BYTES_FOR_DES)       //12
#define IDX_FRAME_ID     (IDX_DES         + NUM_BYTES_FOR_DES)       //12
#define IDX_SUB_LENGTH   (IDX_SUB_HEAD    + NUM_BYTES_FOR_SUB_HEAD)  //14
#define IDX_SUB_PID      (IDX_SUB_LENGTH  + NUM_BYTES_FOR_SUB_LENGTH)//15
#define IDX_SUB_CMD      (IDX_SUB_PID     + NUM_BYTES_FOR_SUB_PID)   //16
#define IDX_CODE         (IDX_SUB_CMD     + NUM_BYTES_FOR_SUB_CMD)   //17

//for packet descriptor
#define PACK_DES_TOFDATA_UDP            2
#define PACK_DES_COMMANDS_TCP           6

//for mCommWorkingMode
#define RECV_DATA_MODE                  0
#define SEND_CMD_MODE                   1

//commands types
//used for sub_packet_type in protocol
#define CMD_SET_PARAM                   0x1//commands for setting one specific lidar parameter
#define CMD_GET_PARAM                   0x2//commands for getting one specific lidar parameter
#define CMD_CMD                         0x3//other commands

//for communicationErrorHandler()
#define TCP_M_SEND_ERROR        1
#define TCP_M_LIDAR_SEND_ERROR  2
#define TCP_M_DIFF_LEN_ERROR    3
#define TCP_M_UNCORRECT_ERROR   4
#define TCP_M_RECV_ERROR        5
#define UDP_M_SEND_ERROR        6   
#define UDP_M_RECV_ERROR        7

class NetworkInterface
{
public:
  YRLUDPSocket mUDPSocket;
  YRLTCPSocket mTCPSocket;

  // buffer for packet
  unsigned char mBulkDataBuffer[2048];
  std::vector< unsigned char > mUDPRecvBuffer;

  //control whether getScanningData() runs or not
  std::atomic <bool> mbStopGettingScanData;

  bool TCPConnected;

  std::string mIPAddr;
  int mPortNum;
   
private:
  std::vector< unsigned char > mTCPSendBuffer;
  std::vector< unsigned char > mUDPSendBuffer;

  //0: receving data from Lidar
  //1: sending command to Lidar
  int mCommWorkingMode;

  //for pid of sending TCP M
  uint8_t mSendTCPPID;
  
  //for save table_size
  uint32_t mTableSize;

public:
  NetworkInterface ();
  ~NetworkInterface ();
  
  int ConnectTCP ();
  void CloseTCP ();
  
  void savePortNum (const int portNum);
  void saveIPAddr (const std::string& ipAddr);
  void saveTableSize (const uint32_t table_size);
  
  void doCommunicationMode (int working_mode, uint8_t command_type, uint8_t code, int32_t data);
  int requestAndGetResponse (uint8_t command_type, uint8_t code, int32_t data);
  bool sendTCPCommand (uint8_t command_type, uint8_t code, int32_t data);
  int recvTCPResponse2 (uint8_t command_type, uint8_t code);
  bool infromUDPStart ();

  char generate_crc (uint8_t* pdata, char length);
  void setTCPCommand (uint8_t packet_id, uint8_t command_type, uint8_t code, int32_t data);
  void setUDPCommand ();
  void printSendTCPMessage ();
  void printReceivedUDPMessage ();

  //void printReceivedTCPMessage (unsigned char * buffer);
};
#endif //NETWORK_INTERFACE_HPP