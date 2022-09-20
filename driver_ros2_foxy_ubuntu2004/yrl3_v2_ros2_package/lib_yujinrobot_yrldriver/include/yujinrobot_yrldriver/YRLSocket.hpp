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
#ifndef YRL_SOCKET_HPP
#define YRL_SOCKET_HPP

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma warning(disable: 4819)
#pragma warning(disable : 4996)
#else
#include <unistd.h>//for close()
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#endif

#include <cstring>
#include <string>
#include <iostream>
#include <string.h>
#include <errno.h>
#include "YRLLOG.hpp"

class YRLSocket 
{
protected:
  /// name of the socket
  /// it will be used to identify module during debugging
  std::string socket_name;

  /// socket descriptor
#ifdef _WIN32	
  SOCKET socket_descriptor;
#else
  int socket_descriptor;
#endif
  /// blocking mode or not
  bool blocking_mode;

public:
  YRLSocket ( const std::string & socketName, int socketType, int protocolName)
  : socket_name( socketName )
  , socket_descriptor(-1)
  , blocking_mode(true)
  {
    SocketCreate(socketType, protocolName);
  }

#ifdef _WIN32
  YRLSocket (const std::string& socketName, SOCKET socketDescriptor)
  : socket_name(socketName)
  , socket_descriptor(socketDescriptor)
  , blocking_mode(true)
  {}
#else
  YRLSocket (const std::string& socketName, int socketDescriptor)
  : socket_name(socketName)
  , socket_descriptor(socketDescriptor)
  , blocking_mode(true)
  {}
#endif

  ~YRLSocket ()
  {
    LOGPRINT(YRLSocket, YRL_LOG_TRACE, ("distructor called\n"));
    SocketClose();
  }

  void SocketCreate (int socketType, int protocolName)
  {
    socket_descriptor = socket(PF_INET, socketType, protocolName);

    if ( !IsAvailable() )
    {
      LOGPRINT(YRLSocket, YRL_LOG_ERROR, ("[%s] socketType: %d, protocolName: %d, creating socket failed.\n", socket_name.c_str(), socketType, protocolName));
      return;
    }
  }

  //virtual void SocketClose()
  void SocketClose ()
  {
    if (IsAvailable())
    {
#ifdef _WIN32
      ::closesocket(socket_descriptor);
      socket_descriptor = -1;
#else
      ::close(socket_descriptor);
      socket_descriptor = -1;
#endif
    }
  }

#ifdef _WIN32
  SOCKET GetSocketDescriptor ()
  {
    return socket_descriptor;
  }
#else
  int GetSocketDescriptor ()
  {
    return socket_descriptor;
  }
#endif

  bool IsAvailable ()
  {
    return (socket_descriptor >= 0);
  }

  int Read (void *buffer, int bufferLen)
  {
    int ret = ::recv(socket_descriptor, (char *) buffer, bufferLen, 0);//recv: window, linux 동일
    return ret;
  }

  int ReadWithTimeout (void* buffer, int bufferLen, unsigned int maxTimeInMsec)
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
      LOGPRINT(YRLSocket, YRL_LOG_ERROR, ("recv() error: %d\n", error));
      return -2;
    }
#else
    if (ret_select == -1)
    {
      LOGPRINT(YRLSocket, YRL_LOG_ERROR, ("select error. errno:%d, %s\n", errno, strerror(errno)));
      return -2;
    }
#endif
    else if (ret_select == 0)
    {//time out
      LOGPRINT(YRLSocket, YRL_LOG_ERROR, ("time out\n"));
      return -3;
    }
    else
    {// ret_select > 0
      if (FD_ISSET(socket_descriptor, &set))
      {
        // system call to get data
        // success: return num of received data (>= 0) (0: EOF)
        // fail: return -1
        int ret = ::recv(socket_descriptor, (char*)buffer, bufferLen, 0);//recv: window, linux 동일
#ifdef _WIN32
        if (ret == SOCKET_ERROR)
        {
          int error = WSAGetLastError();
          LOGPRINT(YRLSocket, YRL_LOG_ERROR, ("recv() error: %d\n", error));
        }
#else
        if (ret < 0)
        {
          LOGPRINT(YRLSocket, YRL_LOG_ERROR, ("recv() error. errno:%d, %s\n", errno, strerror(errno)));
        }
#endif
        return ret;
      }

      LOGPRINT(YRLSocket, YRL_LOG_ERROR, ("error\n"));
      return -4;
    }
  }

  void recvSocketBufferClear ()
  {
    char temp_char;
#ifdef _WIN32	
    u_long temp_long = 0;
    u_long i;
#else	
    long temp_long = 0;
    long i;
#endif
    

#ifdef _WIN32
    if (ioctlsocket(socket_descriptor, FIONREAD, &temp_long) != SOCKET_ERROR)
    {
      for (i = 0; i < temp_long; i++)
      {
        ::recv(socket_descriptor, &temp_char, sizeof(char), 0);
      }
    }
#else
    if (ioctl(socket_descriptor, FIONREAD, &temp_long) != -1)
    {
      for (i = 0; i < temp_long; i++)
      {
        ::recv(socket_descriptor, &temp_char, sizeof(char), 0);
      }
    }
#endif
  }

protected:
  //initialize socket address
  bool buildAddrStructure (const std::string &address, unsigned short int port, sockaddr_in &addr)
  {
    if ( address.empty() )
    {
      return false;
    }
    else 
    {
      std::memset(&addr, 0, sizeof(addr));
      addr.sin_addr.s_addr = inet_addr( address.c_str() );
      addr.sin_family = AF_INET;
      addr.sin_port = htons(port);    
    }

    return true;
  }

  void setLocalPort (unsigned short int localPortNumber, sockaddr_in &addr)
  {
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(localPortNumber);
  }
};
#endif //YRL_SOCKET_HPP