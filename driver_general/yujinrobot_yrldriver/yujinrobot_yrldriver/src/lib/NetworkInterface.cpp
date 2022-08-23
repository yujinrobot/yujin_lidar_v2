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
//#include "pch.h"
#include "../../include/yujinrobot_yrldriver/NetworkInterface.hpp"

#ifdef _WIN32
#pragma warning(disable : 4251)
#pragma warning(disable: 4819)
#endif

NetworkInterface::NetworkInterface ()
: mUDPSocket("udp_listener")
, mTCPSocket("tcp_client")
, mIPAddr("192.168.1.250")
, mPortNum(1234)
, mCommWorkingMode(SEND_CMD_MODE)
, mbStopGettingScanData(true)
, mSendTCPPID(1)
, mTableSize(0)
, TCPConnected(false)
{
  mUDPRecvBuffer.assign(4096 * 4, 0);
  mUDPSendBuffer.assign(24, 0);
  mTCPSendBuffer.assign(24, 0);
  memset(mBulkDataBuffer, 0, sizeof(mBulkDataBuffer));

  LOGPRINT(NetworkInterface, YRL_LOG_TRACE, ("constructor called\n"));
}

NetworkInterface::~NetworkInterface ()
{
  LOGPRINT(NetworkInterface, YRL_LOG_TRACE, ("distructor called\n"));
}

int NetworkInterface::ConnectTCP ()
{
  //==================================================================================================
  // ConnectTCP()

  // return -1: Fail to connect to TCP
  // return 1: Success to connect to TCP
  // return 0: Already connected
  //==================================================================================================

  if (!TCPConnected)
  {
    if (mTCPSocket.GetSocketDescriptor() == -1)
    {
      mTCPSocket.SocketCreate(SOCK_STREAM, IPPROTO_TCP);
    }

    if (!mTCPSocket.Init(mIPAddr, mPortNum))
    {
      LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("Init() error.\n"));
      TCPConnected = false;
      return -1;
    }
    else
    {
      TCPConnected = true;
      return 1;
    }
  }
  else 
  {
    return 0;
  }
}

void NetworkInterface::CloseTCP ()
{
  if (mTCPSocket.GetSocketDescriptor() >= 0)
  {
    mTCPSocket.SocketClose();
    TCPConnected = false;
  }
}

void NetworkInterface::savePortNum (const int portNum)
{
  mPortNum = portNum;
}

void NetworkInterface::saveIPAddr (const std::string& ipAddr)
{
  mIPAddr = ipAddr;
}

void NetworkInterface::saveTableSize (const uint32_t table_size)
{
  mTableSize = table_size;
}

void NetworkInterface::doCommunicationMode (const int working_mode, const uint8_t command_type, const uint8_t code, const int32_t data)
{
  //==================================================================================================
  // doCommunicationMode()
  // There are two types of working mode. Do things according to the mode.

  // <working_mode>
  // 1. RECV_DATA_MODE: receive scanning data from LiDAR using UDP communication
  // 2. SEND_CMD_MODE: send command to LiDAR using TCP communication

  // <RECV_DATA_MODE>
  // Start UDP commnunication to get scanning data from LiDAR
  // And then mbStopGettingScanData is changed from true to false so that getScanningData() can do something.

  // <SEND_CMD_MODE>
  // Stop getScanningData() by changing mbStopGettingScanData from false to true
  // Send a command message and receive message response for the command.
  //==================================================================================================
  
  int result = 0;

  switch (working_mode)
  {
  case RECV_DATA_MODE: //UDP
    LOGPRINT(NetworkInterface, YRL_LOG_TRACE, ("Start UDP communication\n"));
    if (!infromUDPStart())
    {
    }
    else
    {
      LOGPRINT(NetworkInterface, YRL_LOG_TRACE, ("Do getScanningData()\n"));
      if (mbStopGettingScanData)
      {
        mbStopGettingScanData = false;
      }
    }
    break;

  case SEND_CMD_MODE: //TCP
  {
    if (!mbStopGettingScanData)
    {
      mbStopGettingScanData = true;
    }

    int cnt = 0;
    do
    {
      result = requestAndGetResponse(command_type, code, data);
      if (result != 0)
      {
        LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("Try requestAndGetResponse() again.\n"));
#ifdef _WIN32	
        timeBeginPeriod(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        timeEndPeriod(1);
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
#endif
      }
      cnt++;
    } while (result != 0 && cnt < 2);
    break;
  }

  default:
    break;
  }
}

int NetworkInterface::requestAndGetResponse (const uint8_t command_type, const uint8_t code, const int32_t data)
{
  //==================================================================================================
  // requestAndGetResponse()
  // Send command to the LiDAR, and if it succeeded to send, then wait to receive response from LiDAR.
  // If sending M is failed, try sending M again up to 3 times.
  // When received M from LiDAR, verify and save data of the message.

  // return TCP_M_SEND_ERROR:         Failed to send command message
  // return TCP_M_RECV_ERROR:         Failed to receive response message
  // return TCP_M_LIDAR_SEND_ERROR:   LiDAR can't send response message
  // return TCP_M_UNCORRECT_ERROR:    Received wrong message
  // return TCP_M_DIFF_LEN_ERROR:     Received different number of message
  // return 0:                        Succeeded to send and receive message correctly
  //==================================================================================================

  bool successSend;
  int cnt = 0;
  int ret = 0;

  do {
    successSend = sendTCPCommand(command_type, code, data);
    if (!successSend)
    {
      LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("Failed to send TCP M to LiDAR. Send again.\n"));
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
    }
    cnt++;
  } while(!successSend && cnt < 3);

  if (!successSend)
  {
    LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("Failed to send command to LiDAR.\n"));
    return TCP_M_SEND_ERROR;
  }
  else
  {
    ret = recvTCPResponse2(command_type, code);
    if ( ret < 0)
    {
      if (ret == -1)
      {
        LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("FAILED TO RECEIVE M FROM LiDAR.\n"));
        return TCP_M_RECV_ERROR;
      }
      else if (ret == -2)
      {
        LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("LIDAR CANNOT SEND M NOW. REQUEST AGAIN.\n"));
        return TCP_M_LIDAR_SEND_ERROR;
      }
      else if (ret == -3)
      {
        LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("RECEIVED WORNG M.\n"));
        return TCP_M_UNCORRECT_ERROR;
      }
      else
      {
        LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("NUMBER OF RECEIVED M IS DIFFERENT.\n"));
        return TCP_M_DIFF_LEN_ERROR;
      } 
    }
  }

  LOGPRINT(NetworkInterface, YRL_LOG_TRACE, ("Succeeded to send command and receive correct msg from LiDAR\n"));
  return 0;
}

bool NetworkInterface::sendTCPCommand (const uint8_t command_type, const uint8_t code, const int32_t data)
{
  //==================================================================================================
  // sendTCPCommand()
  // Send command to the LiDAR through TCP communication.

  // Write() returns size of M sent.
  // Whether sending M is successful or not depends on the size of M sent.

  // return false: Failed to send command message (when Wirte() returned -1 or 0 or something else)
  // return true : Succeeded to send response message
  //==================================================================================================

  setTCPCommand(mSendTCPPID, command_type, code, data);
  printSendTCPMessage();

  if (mSendTCPPID >= 0xff)
  {
    mSendTCPPID = 1;
  }
  else
  {
    mSendTCPPID++;
  }

  if ((mTCPSocket.Write(mTCPSendBuffer.data(), mTCPSendBuffer.size())) != static_cast<int>(mTCPSendBuffer.size()))
  {
    LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("Failed to send msg through TCP communication.\n"));
#ifdef _WIN32	
    timeBeginPeriod(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    timeEndPeriod(1);
#else	
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
    return false;
  }

  return true;
}

int NetworkInterface::recvTCPResponse2 (const uint8_t command_type, const uint8_t code)
{
  //=========================================================================================
  // recvTCPResponse2()
  // Receive response message from LiDAR through TCP Communication
  // In case of getting prameters, save data to mBulkDataBuffer

  // return -4: The number of received complete message is different than expected.
  // return -3: Received data but wrong data
  // return -2: LiDAR can't send response message. Have to send CMD again. 
  // return -1: Failed to receive data from LiDAR
  // return  1: Succeeded to receive complete data
  
  // Rules
  // 1. '0x02' must come four times in a row.
  // 2. '0x12''0x23' must be along together.
  // 3. After header(02020202) came out, subheader(1223) must follow.
  // 4. After subheader(1223) came out, index of PID, CMD, CODE, DATA, ETX are fixed. We assume those data are in right index. 
  // 5. After last 'etx' came out, keep rest of the data.
  // 7. There must be three things(header && subheader && etx) to be considered complete message.
  // 6. In case of 'recv_cnt' less than 24byte, we cosider it we didn't receive data.
  //=========================================================================================

  unsigned char buffer[3000] = {0};
  int recv_len = 0;
  int recv_cnt = 0;
  int recv_m_cnt = 0;

  unsigned char left[24] = {0};
  int left_num = 0;

  uint8_t recv_pid = 0;
  uint8_t recv_cmd = 0;
  uint8_t recv_code = 0;
  uint32_t recv_data = 0;

  int header_cnt = 0;
  bool header = false;
  bool subheader = false;
  bool etx = false;
  int idx_subheader1 = 0;

  memset(mBulkDataBuffer, 0, sizeof(mBulkDataBuffer));

  while (1)
  {
    memset(buffer, 0, sizeof(buffer));
    if (left_num > 0)
    {
      memcpy(&buffer[0], &left[0], left_num);
      memset(left, 0, sizeof(left));
      recv_cnt = mTCPSocket.ReadWithTimeout( &buffer[left_num], sizeof(buffer) - left_num, 1000 );
    }
    else
    {            
      recv_cnt = mTCPSocket.ReadWithTimeout( &buffer[0], sizeof(buffer), 1000 );
    }
    
    LOGPRINT(NetworkInterface, YRL_LOG_DEBUG, ("recv_cnt: %d\n", recv_cnt));
    if (recv_cnt < 0) //in case of -1, -2, -3, -4
    {
      LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("Failed. Didn't get data\n"));
      return -1;//or continue;
    }
    else
    {
      recv_len += recv_cnt;
      recv_cnt += left_num;
    }
    LOGPRINT(NetworkInterface, YRL_LOG_DEBUG, ("recv_len: %d\n", recv_len));
    
    if (left_num == 0)
    {
      recv_m_cnt = 0;
    }
    
    for (int i = 0; i < recv_cnt; i++)
    {
      if (buffer[i] == 0x02)
      {
        header_cnt++;
        if (header_cnt == 4 && !header)
        {
          header = true;
          subheader = false;
          continue;
        }
      }
      else
      {
        header_cnt = 0;
      }

      if (header && !subheader)
      {
        if (buffer[i] == 0x12 && buffer[i+1] == 0x23)
        {
          subheader = true;
          idx_subheader1 = i;
          continue;
        }
      }

      if (header && subheader)
      {
        if (i == idx_subheader1 + 3)//pid
        {}
        else if (i == idx_subheader1 + 4)//cmd 
        {
          memcpy(&recv_cmd, &buffer[i], sizeof(uint8_t));
          if (recv_cmd == 4)
          {   
            header = false;
            subheader = false;
            LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("LiDAR CANNOT SEND RESPONSE. PLEASE SEND CMD AGAIN.\n"));
            return -2;
          }
          if (recv_cmd != command_type) 
          {
            header = false;
            subheader = false;
            LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("INCORRECT COMMAND TYPE: [%d][%d]\n", buffer[i], mTCPSendBuffer[16]));
            return -3;
          }
        }
        else if (i == idx_subheader1 + 5)//code 
        {
          memcpy(&recv_code, &buffer[i], sizeof(uint8_t));
          if (recv_code == 0)
          {
            header = false;
            subheader = false;
            LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("LiDAR CANNOT SEND PARAMETER NOW. PLEASE SEND CMD AGAIN.\n"));
            return -2;
          }
          if (recv_code != code)
          {
            if (command_type == CMD_GET_PARAM && code == 0) //get whole parameters
            {} 
            else
            {
              header = false;
              subheader = false;
              LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("INVALID CODE IN RECEIVED M.\n"));
              return -3;
            }
          }
        }
        else if (i == idx_subheader1 + 6)//data 
        {
          memcpy(&recv_data, &buffer[i], sizeof(uint32_t));
        }
        else if (i == idx_subheader1 + 11)//etx 
        {
          if (buffer[i] != 0x03)
          {
            header = false;
            subheader = false;
            etx = false;
            LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("WRONG ETX\n"));
            return -3;
          }
          else
          {
            etx = true;

            if ((0 < (recv_len - 1) - i) && ((recv_len - 1) - i < 24))
            {
              left_num = recv_len - 1 - i;

              LOGPRINT(NetworkInterface, YRL_LOG_DEBUG, ("i: %d, recv_len - 1: %d, %d LEFT!\n", i, recv_len - 1, left_num));
              memcpy(&left[0], &buffer[i+1], recv_len - 1 - i);
            }
            else
            {
              left_num = 0;
            }
          }
        }
      }

      if (header && subheader && etx)
      {
        std::string log = "Recv TCP M: ";
        for (int j = idx_subheader1; j <= i; j++)
        {
          if (j == idx_subheader1 + 6)
          {
            log += "[" + std::to_string(recv_data) + "]";
            j += 3;
          }
          else
          {
            log += "[" + std::to_string(buffer[j]) + "]";
          }
        }
        LOGPRINT(NetworkInterface, YRL_LOG_MESSAGE, ("%s\n", log.c_str()));
        
        memcpy(&mBulkDataBuffer[4 * recv_m_cnt], &recv_data, sizeof(uint32_t));
        recv_m_cnt++;
        LOGPRINT(NetworkInterface, YRL_LOG_DEBUG, ("%d complete M\n", recv_m_cnt));

        header = false;
        subheader = false;
        etx = false;
      }
    }

    //if finished, break!
    if (command_type == CMD_SET_PARAM || command_type == CMD_CMD)
    {
      if (recv_m_cnt != 1) 
      {
        LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("DIFFERENT NUMBER OF M. RECEIVED M COUNT: %d, EXPECTED M COUNT: 1. SEND COMMAND AGAIN.\n", recv_m_cnt));
        return -4;
      }
      
      break;//break while;
    }
    else
    {
      if (code == 0) //get whole parameter
      {}
      else if (code == 13 || code == 14 || code == 31 || code == 32)
      {
        if (recv_m_cnt < 42) 
        {
          if (recv_len < 24 * 42)
          {}
          else
          {
            LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("DIFFERENT NUMBER OF M. RECEIVED M COUNT: %d, EXPECTED M COUNT: 42. SEND COMMAND AGAIN.\n", recv_m_cnt));
            return -4;
          }
        }
        else
        {
          break;//break while;
        }
      }
      else
      {
        if (recv_m_cnt != 1) 
        {
          LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("DIFFERENT NUMBER OF M. RECEIVED M COUNT: %d, EXPECTED M COUNT: 1. SEND COMMAND AGAIN.\n", recv_m_cnt));
          return -4;
        }
        
        break;//break while;
      }
    }
  }

  return 1;
}

bool NetworkInterface::infromUDPStart ()
{
  //==================================================================================================
  // infromUDPStart()
  // Send a message to LiDAR through UDP connection. This informs LiDAR that TCP communication is ended and UDP communication will start.

  // return false: Failed to send command message through UDP communication
  // return true : Succeeded to send command message through UDP communication
  //==================================================================================================

  setUDPCommand();

  int ret = -2;
  int cnt = 0;

  do {
    //return -1, -2, num of data
    ret = mUDPSocket.WriteTo( mUDPSendBuffer.data(), mUDPSendBuffer.size(), mIPAddr, mPortNum );
    LOGPRINT(NetworkInterface, YRL_LOG_TRACE, ("UDP M number of bytes sent: %d\n", ret));

    if (ret == -1 )
    {
      LOGPRINT(NetworkInterface, YRL_LOG_INFO, ("Cannot send UDP M to LiDAR\n"));
#ifdef _WIN32	
      timeBeginPeriod(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      timeEndPeriod(1);
#else	
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
    }
    cnt++;
  } while (ret < 0 && cnt < 3);

  if (ret < 0)
  {
    LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("Failed to send message through UDP communication\n"));
    return false;
  }
  else
  {
    return true;
  }
}

char NetworkInterface::generate_crc (uint8_t *pdata, char length)
{
  char crc = 0x0;
  for (int i = 0; i < length; i++)
  {
    crc ^= pdata[i];
  }
  return crc;
}

void NetworkInterface::setTCPCommand (const uint8_t packet_id, const uint8_t command_type, const uint8_t code, const int32_t data)
{
  //=========================================================================================
  // setTCPCommand()
  // Setting 3 types of commands to send to Lidar

  // <COMMAND TYPES>
  // CMD_SET_PARAM     0x1//commands for setting one specific lidar parameter
  // CMD_GET_PARAM     0x2//commands for getting one specific lidar parameter
  // CMD_CMD           0x3//other commands

  //The size of command message is 24
  //=========================================================================================

  mTCPSendBuffer.assign(24, 0);

  //[0]~[3]: header ID: 4byte
  mTCPSendBuffer[0] = 0x02;
  mTCPSendBuffer[1] = 0x02;
  mTCPSendBuffer[2] = 0x02;
  mTCPSendBuffer[3] = 0x02;

  //[11]: packet descriptor: 1byte
  mTCPSendBuffer[IDX_DES] = PACK_DES_COMMANDS_TCP;

  //[12]~[13]: sub header: 2byte
  mTCPSendBuffer[IDX_SUB_HEAD] = 0x12;
  mTCPSendBuffer[IDX_SUB_HEAD + 1] = 0x23;

  //[14]: sub legnth. legth of core data: 1byte
  mTCPSendBuffer[IDX_SUB_LENGTH] = 8 & 0x000000ff;

  //[15]: sub packet id: 1byte
  mTCPSendBuffer[IDX_SUB_PID] = packet_id;//0~255

  switch (command_type)
  {
  case CMD_SET_PARAM:
    if (code < 1 || ( 22 < code && code < 33 ) || 34 < code)
    {
      LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("Wrong Code! Does not exist.\n"));
      return;
    }

    //[16]: sub command(packet type): 1byte
    mTCPSendBuffer[IDX_SUB_CMD] = CMD_SET_PARAM;//1

    //[17]: code: 1byte
    mTCPSendBuffer[IDX_CODE] = code;//1~22

    //[18]~[21]: inner data: 4byte
    //data: values to set
    mTCPSendBuffer[IDX_CODE + 1] = data & 0x000000ff;
    mTCPSendBuffer[IDX_CODE + 2] = (data & 0x0000ff00) >> 8;
    mTCPSendBuffer[IDX_CODE + 3] = (data & 0x00ff0000) >> 16;
    mTCPSendBuffer[IDX_CODE + 4] = (data & 0xff000000) >> 24;

    break;
case CMD_GET_PARAM:
    if (code < 0 || code > 34)
    {
      LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("Wrong Code! Does not exist.\n"));
      return;
    }

    //[16]: sub command(packet type): 1byte
    mTCPSendBuffer[IDX_SUB_CMD] = CMD_GET_PARAM;//2

    //[17]: code: 1byte
    mTCPSendBuffer[IDX_CODE] = code;//1~29

    //data value is zero. Data value is not needed in case of getting parameters.

    break;
case CMD_CMD:
    if (code < 1 || code > 4)
    {
      LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("Wrong Code! Does not exist.\n"));
      return;
    }
    //[16]: sub command(packet type): 1byte
    mTCPSendBuffer[IDX_SUB_CMD] = CMD_CMD;//3

    //[17]: code: 1byte
    mTCPSendBuffer[IDX_CODE] = code;//1~4

    //[18]~[21]: inner data: 4byte
    mTCPSendBuffer[IDX_CODE + 1] = data & 0x000000ff;
    mTCPSendBuffer[IDX_CODE + 2] = (data & 0x0000ff00) >> 8;
    mTCPSendBuffer[IDX_CODE + 3] = (data & 0x00ff0000) >> 16;
    mTCPSendBuffer[IDX_CODE + 4] = (data & 0xff000000) >> 24;

    break;
default:
    LOGPRINT(NetworkInterface, YRL_LOG_ERROR, ("Wrong command type! Does not exist.\n"));
    break;
  }

  //[22]: CRC: 1byte
  mTCPSendBuffer[IDX_CODE + 5] = generate_crc(&mTCPSendBuffer[IDX_SUB_LENGTH], 8);

  //[23]: ETX: 1byte
  mTCPSendBuffer[IDX_CODE + 6] = 0x03;
}

void NetworkInterface::setUDPCommand ()
{
  //=========================================================================================
  // setUDPCommand()
  // Setting UDP message to send to Lidar
  // The size of UDP sending message is 24
  //=========================================================================================
  static uint8_t packetID = 1;
  
  mUDPSendBuffer.assign(24, 0);

  //initialize mUDPSendBuffer
  //[0]~[3]: header ID: 4byte
  mUDPSendBuffer[0] = 0x02;
  mUDPSendBuffer[1] = 0x02;
  mUDPSendBuffer[2] = 0x02;
  mUDPSendBuffer[3] = 0x02;

  //[11]: packet descriptor: 1byte
  mUDPSendBuffer[IDX_DES] = PACK_DES_TOFDATA_UDP;

  //[12]~[13]: sub header: 2byte
  mUDPSendBuffer[IDX_SUB_HEAD] = 0x12;
  mUDPSendBuffer[IDX_SUB_HEAD + 1] = 0x23;

  //[14]: sub legnth. legth of core data: 1byte
  mUDPSendBuffer[IDX_SUB_LENGTH] = 8 & 0x000000ff;

  //[15]: sub packet id: 1byte
  mUDPSendBuffer[IDX_SUB_PID] = packetID;//1~255

  //[22]: CRC: 1byte
  mUDPSendBuffer[IDX_CODE + 5] = generate_crc(&mUDPSendBuffer[IDX_SUB_LENGTH], 8);

  //[23]: ETX: 1byte
  mUDPSendBuffer[IDX_CODE + 6] = 0x03;

  //if packet id exceeded 255, then initialize it to zero.
  if(packetID >= 0xff)
  {
    packetID = 1;
  }
  else
  {
    packetID++;
  }
}

void NetworkInterface::printSendTCPMessage ()
{
  unsigned char * ptr = &mTCPSendBuffer[0];
  uint32_t data = 0;
  std::string log = "Send TCP M: ";

  //header0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;

  //header1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //header2
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //header3
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //fw version0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //fw version1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //hw version
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp2
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp4
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //descriptor
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //header0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //header1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //length
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //pid
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //command
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //code
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //data
  memcpy(&data, ptr, sizeof(uint32_t));
  log += "[" + std::to_string(data) + "]";
  ptr += 4;
  
  //LRC
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //ETX
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;

  LOGPRINT(NetworkInterface, YRL_LOG_MESSAGE, ("%s\n", log.c_str()));
}

void NetworkInterface::printReceivedUDPMessage ()
{
  unsigned char * ptr = &mUDPRecvBuffer[0];
  uint32_t data = 0;
  std::string log = "RECV UDP M: ";

  //header0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //header1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //header2
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //header3
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //fw version0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //fw version1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //hw version
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp2
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //time_stamp4
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //descriptor
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //frame_id0
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //frame_id1
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //frame_id2
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //frame_id3
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //status id
  log += "[" + std::to_string(*ptr) + "]";
  ptr++;
  
  //status data
  memcpy(&data, ptr, sizeof(uint32_t));
  log += "[" + std::to_string(data) + "]";
  ptr += 4;
  
  LOGPRINT(NetworkInterface, YRL_LOG_MESSAGE, ("%s\n", log.c_str()));
}

/*
void NetworkInterface::printReceivedTCPMessage (unsigned char * buffer)
{
  unsigned char * ptr = &buffer[0];
  uint32_t data = 0;

  printf("Recv TCP M: ");
  printf("[%x]", *ptr);//header0
  ptr++;
  printf("[%x]", *ptr);//header1
  ptr++;
  printf("[%x]", *ptr);//header2
  ptr++;
  printf("[%x]", *ptr);//header3
  ptr++;
  printf("[%d]", *ptr);//fw version0
  ptr++;
  printf("[%d]", *ptr);//fw version1
  ptr++;
  printf("[%d]", *ptr);//hw version
  ptr++;
  printf("[%d]", *ptr);//time_stamp0
  ptr++;
  printf("[%d]", *ptr);//time_stamp1
  ptr++;
  printf("[%d]", *ptr);//time_stamp2
  ptr++;
  printf("[%d]", *ptr);//time_stamp4
  ptr++;
  printf("[%d]", *ptr);//descriptor
  ptr++;
  printf("[%x]", *ptr);//header0
  ptr++;
  printf("[%x]", *ptr);//header1
  ptr++;
  printf("[%d]", *ptr);//length
  ptr++;
  printf("[%d]", *ptr);//pid
  ptr++;
  printf("[%d]", *ptr);//command
  ptr++;
  printf("[%d]", *ptr);//code
  ptr++;
  memcpy(&data, ptr, sizeof(uint32_t));
  printf("[  %d  ]", data);//data
  ptr += 4;
  printf("[%x]", *ptr);//LRC
  ptr++;
  printf("[%x]", *ptr);//ETX
  ptr++;
  printf("\n");
}
*/