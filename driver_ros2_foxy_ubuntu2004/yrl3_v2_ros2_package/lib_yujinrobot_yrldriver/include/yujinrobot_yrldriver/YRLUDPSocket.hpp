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
#ifndef YRL_UDP_SOCKET_HPP
#define YRL_UDP_SOCKET_HPP
#include "YRLSocket.hpp"

class YRLUDPSocket : public YRLSocket
{
protected:
  sockaddr_in dest_addr;

public:
  YRLUDPSocket( const std::string & socketName)
  : YRLSocket(socketName,SOCK_DGRAM, IPPROTO_UDP)
  {}

  ~YRLUDPSocket()
  {
    LOGPRINT(YRLUDPSocket, YRL_LOG_TRACE, ("distructor called\n"));
  }

  int WriteTo( const void *buffer, int bufferLen, const std::string &serverAddress, unsigned short int serverPort )
  {
    LOGPRINT(YRLUDPSocket, YRL_LOG_TRACE, ("UDP send M size: %d\n", bufferLen));
    // structure for destination
    //sockaddr_in dest_addr;

    // at the first we have to make structure to specify destination
    if( !buildAddrStructure(serverAddress, serverPort, dest_addr) )
    {
      LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("Empty serverAddress\n"));
      return -2;
    }

    //send data to destination
    //success : return num of bytes
    //fail: return -1
    return sendto(socket_descriptor, (char *) buffer, bufferLen, 0, (sockaddr *) &dest_addr, sizeof(dest_addr));
  }

  int ReadFromTimeout(void* buffer, int bufferLen, unsigned int maxTimeInMsec)
  {
    fd_set set;
    struct timeval tv;
    tv.tv_sec = maxTimeInMsec / 1000;
    tv.tv_usec = (maxTimeInMsec % 1000) * 1000;

    FD_ZERO(&set);
    FD_SET(socket_descriptor, &set);
#ifdef _WIN32
    int ret_select = select(0, &set, NULL, NULL, &tv);
#else
    int ret_select = select(socket_descriptor + 1, &set, NULL, NULL, &tv);
#endif

    // select error
#ifdef _WIN32
    if (ret_select == SOCKET_ERROR)
    {
      int error = WSAGetLastError();
      LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("recv() error: %d\n", error));
      return -2;
    }
#else
    if (ret_select == -1)
    {
      LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("select error. errno:%d, %s\n", errno, strerror(errno)));
      return -2;
    }
#endif
    else if (ret_select == 0)
    {//time out
      LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("time out\n"));
      return -3;
    }
    else
    {// ret_select > 0
      if (FD_ISSET(socket_descriptor, &set))
      {
        // system call to get data
        // success: return num of received data (>= 0) (0: EOF)
        // fail: return -1
#ifdef _WIN32
        int size = sizeof(dest_addr);
#else
        socklen_t size = sizeof(dest_addr);
#endif
        int ret = ::recvfrom(socket_descriptor, (char*)buffer, bufferLen, 0, (sockaddr*)&dest_addr, &size);

#ifdef _WIN32
        if (ret == SOCKET_ERROR)
        {
          int error = WSAGetLastError();
          LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("recvfrom() error: %d\n", error));
        }
#else
        if (ret < 0)
        {
          LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("Failed. errno:%d, %s\n", errno, strerror(errno)));
        }
#endif
        return ret;
      }

      LOGPRINT(YRLUDPSocket, YRL_LOG_ERROR, ("error\n"));
      return -4;
    }
  }
};
#endif //YRL_UDP_SOCKET_HPP