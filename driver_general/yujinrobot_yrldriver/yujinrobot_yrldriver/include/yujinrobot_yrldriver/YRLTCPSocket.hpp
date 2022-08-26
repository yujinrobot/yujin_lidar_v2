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
#ifndef YRL_TCP_SOCKET_HPP
#define YRL_TCP_SOCKET_HPP

#ifdef _WIN32
#pragma warning(disable: 4819)
#endif

#include "YRLSocket.hpp"

class YRLTCPSocket : public YRLSocket
{
public:
  YRLTCPSocket ( const std::string & socketName) //for client
  : YRLSocket (socketName, SOCK_STREAM, IPPROTO_TCP) {}

  YRLTCPSocket ( const std::string & socketName, unsigned short int localPortNumber, int connectionQueueLength)
  : YRLSocket (socketName, SOCK_STREAM, IPPROTO_TCP) //for server
  {
#ifdef _WIN32	
    int option;
    setsockopt(socket_descriptor, SOL_SOCKET, SO_REUSEADDR, (char*)&option, sizeof(option));
#else
    int option;
    // socklen_t option_len;
    // option_len = sizeof(option);
    setsockopt(socket_descriptor, SOL_SOCKET, SO_REUSEADDR, (void*) &option, sizeof(option));
#endif
    sockaddr_in localAddr;
    setLocalPort(localPortNumber, localAddr);
    if (!Bind(localAddr)) 
    {
      LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] localPortNumber: %d, binding failed\n", socket_name.c_str(), localPortNumber));
      SocketClose();
    }
    else
    {
      Listen(connectionQueueLength);
    }
  }

#ifdef _WIN32
  YRLTCPSocket (const std::string& socketName, SOCKET socketDescriptor)
  : YRLSocket (socketName, socketDescriptor)
  {}
#else
  YRLTCPSocket (const std::string& socketName, int socketDescriptor)
  : YRLSocket (socketName, socketDescriptor)
  {}
#endif

  ~YRLTCPSocket ()
  {
    LOGPRINT(YRLTCPSocket, YRL_LOG_TRACE, ("distructor called\n"));
  }

  bool SetBlockingMode ()
  {
#ifdef _WIN32
    blocking_mode = true;
    if ( IsAvailable() )
    {
      unsigned long arg = 0;
      if (ioctlsocket(socket_descriptor, FIONBIO, &arg) == -1 )
      {
        LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] blocking mode setting failed. Close socket.\n", socket_name.c_str()));
        SocketClose();
      }
    }
    return blocking_mode;
#else
    blocking_mode = true;
    if ( IsAvailable() )
    {
      long arg = fcntl(socket_descriptor, F_GETFL, NULL);
      arg &= (~O_NONBLOCK);
      if ( fcntl(socket_descriptor, F_SETFL, arg) == -1 )
      {
        LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] blocking mode setting failed. Close socket.\n", socket_name.c_str()));
        SocketClose();
      }
    }
    return blocking_mode;
#endif
  }

  bool SetNonBlockingMode ()
  {
#ifdef _WIN32
    blocking_mode = false;
    if ( IsAvailable() )
    {
      unsigned long arg = 1;
      if (ioctlsocket(socket_descriptor, FIONBIO, &arg) == -1 )
      {
        int error = WSAGetLastError();
        LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] socket error: %d\n", socket_name.c_str(), error));
        LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] non-blocking mode setting failed. Close socket.\n", socket_name.c_str()));
        SocketClose();
      }
    }
    return blocking_mode;
#else
    blocking_mode = false;
    if ( IsAvailable() )
    {
      long arg = fcntl(socket_descriptor, F_GETFL, NULL);
      arg |= O_NONBLOCK;
      if ( fcntl(socket_descriptor, F_SETFL, arg) == -1 )
      {
        LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] non-blocking mode setting failed. Close socket.\n", socket_name.c_str()));
        SocketClose();
      }
    }
    return blocking_mode;
#endif
  }

  bool Init (const std::string &serverAddress, unsigned short int serverPort)
  {
    SetNonBlockingMode();
    bool ret = Connect(serverAddress, serverPort);
    SetBlockingMode();

    return ret;
  }

  bool Connect (const std::string &serverAddress, unsigned short int serverPort)
  {
#ifdef _WIN32
    sockaddr_in destAddr;
    if (!buildAddrStructure(serverAddress, serverPort, destAddr))
    {
      LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, building structure failed. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort));
      SocketClose();
      return false;
    }

    if (::connect(socket_descriptor, (sockaddr*)&destAddr, sizeof(destAddr)) == -1)
    {//비동기일때 -1리턴
      if (!blocking_mode && WSAGetLastError() == WSAEWOULDBLOCK) //asynchronous connect event
      {
        struct timeval tv;
        int lon;
        fd_set myset;
        int valopt;

        tv.tv_sec = 3;
        tv.tv_usec = 0;
        FD_ZERO(&myset);
        FD_SET(socket_descriptor, &myset);

        int ret = select(0, NULL, &myset, NULL, &tv);//send 할 수 있는 소켓들 목록 리턴
        if (ret > 0)
        {
          //connect event handler
          //check the SO_ERROR option
          //result is saved in valopt
          //if valopt = 0, connection is succeeded. Otherwise, connection is failed.
          lon = sizeof(int);
          getsockopt(socket_descriptor, SOL_SOCKET, SO_ERROR, (char*)(&valopt), &lon);
          if (valopt)
          {
            LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, connection failed - in progress. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort));
            SocketClose();
            return false;
          }
        }
        else
        {
          if (ret == 0)
          {
            LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, connection failed - time out. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort));
            SocketClose();
            return false;
          }
          else
          {
            LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, select() failed.\n", socket_name.c_str(), serverAddress.c_str(), serverPort));
            SocketClose();
            return false;
          }
        }
      }
      else
      {
        LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, connection failed during connection. errno:%d. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort, errno));
        SocketClose();
        return false;
      }
    }

    return true;
#else
    sockaddr_in destAddr;
    if ( buildAddrStructure(serverAddress, serverPort, destAddr) )
    {
      //:: global
      // 0: success
      // -1: fail
      if ( ::connect(socket_descriptor, (sockaddr *) &destAddr, sizeof(destAddr)) == -1 )//<sys/socket.h>
      {//Failed to connect to server
        //Is it in progress?
        if ( !blocking_mode && errno == EINPROGRESS )
        {
          struct timeval tv;
          socklen_t lon;
          fd_set myset;
          int valopt;

          tv.tv_sec = 3;
          tv.tv_usec = 0;
          FD_ZERO(&myset);
          FD_SET(socket_descriptor, &myset);
          if ( select(socket_descriptor+1, NULL, &myset, NULL, &tv) > 0 )
          {
            lon = sizeof(int);
            getsockopt(socket_descriptor, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon);
            if ( valopt )
            {
              LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, connection failed - in progress. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort));
              SocketClose();
              return false;
            }
          }
          else
          {
            LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, connection failed - time out. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort));
            SocketClose();
            return false;
          }
        }
        else
        {
          LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, connection failed during connection. errno:%d, %s. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort, errno, strerror(errno)));
          SocketClose();
          return false;
        }
      }
    }
    else
    {
      LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] serverAddress: %s serverPort: %d, building structure failed. Close socket.\n", socket_name.c_str(), serverAddress.c_str(), serverPort));
      SocketClose();
      return false;
    }

    return true;
#endif
  }

  int Write (const void *buffer, int bufferLen)
  {
    //success: return num of data
    //fail: return -1
    return ::send(socket_descriptor, (char *) buffer, bufferLen, 0);//send: window, linux 동일
  }

  bool Bind (sockaddr_in &localAddr)
  {
    if ( ::bind(socket_descriptor, (sockaddr *) &localAddr, sizeof(localAddr)) < 0 ) //-1
    {
      return false;
    }
    return true;
  } 

  void Listen ( int connectionQueueLength )
  {
    if ( ::listen( socket_descriptor, connectionQueueLength ) < 0 ) //-1
    {
        LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] connectionQueueLength = %d listen failed.\n", socket_name.c_str(), connectionQueueLength));
    }
  }

  YRLTCPSocket* Accept ()
  {
#ifdef _WIN32
    SOCKET socketDescriptor = ::accept(socket_descriptor, NULL, 0);
    if (socketDescriptor == INVALID_SOCKET)
    {
      LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] Accept failed.\n", socket_name.c_str()));
      return 0;
    }
#else
    int socketDescriptor = ::accept(socket_descriptor, NULL, 0);
    if (socketDescriptor < 0)
    {
      LOGPRINT(YRLTCPSocket, YRL_LOG_ERROR, ("[%s] Accept failed.\n", socket_name.c_str()));
      return 0;
    }
#endif
    return new YRLTCPSocket( socket_name, socketDescriptor );
  }
};
#endif //YRL_TCP_SOCKET_HPP