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

#include "YRLLOG.hpp"

class YRLSocket 
{
protected:
    /// name of the socket
    /// it will be used to identify module during debugging
    std::string socket_name;

    /// socket descriptor
    int socket_descriptor;      

    /// blocking mode or not
    bool blocking_mode;

public:
    YRLSocket( const std::string & socketName, int socketType, int protocolName)
    : socket_name( socketName )
    , socket_descriptor(-1)
    , blocking_mode(true)
    {
        SocketCreate(socketType, protocolName);
    }

    YRLSocket( const std::string & socketName, /*int socketType, int protocolName,*/ int socketDescriptor )
    : socket_name(socketName)
    , socket_descriptor(socketDescriptor)
    , blocking_mode(true)
    {}

    ~YRLSocket()
    {
        LOGPRINT(YRLSocket, LOG_TRACE, ("distructor called\n"));
        SocketClose();
    }

    void SocketCreate(int socketType, int protocolName)
    {
#ifdef _WIN32
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        {
            printf("WSAStartup() error!");
        }
#endif
        socket_descriptor = socket(PF_INET, socketType, protocolName);

        if( !IsAvailable() )
        {
            LOGPRINT(YRLSocket, LOG_ERROR, ("[%s] socketType: %d, protocolName: %d, creating socket failed.\n", socket_name.c_str(), socketType, protocolName));
            return;
        }

        // int optval;
        // socklen_t optlen = sizeof(optval);
        // getsockopt(socket_descriptor, SOL_SOCKET, SO_RCVBUF, (char*)&optval, &optlen);
        // printf("1. socket buffer size: %d\n", optval);

        // int bufSize = 6*1024;
        // setsockopt(socket_descriptor, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
        // printf("2. socket buffer size: %d\n", bufSize);
    }

    //virtual void SocketClose()
    void SocketClose()
    {
        if (IsAvailable())
        {
#ifdef _WIN32
            ::closesocket(socket_descriptor);
            socket_descriptor = -1;
            WSACleanup();
#else
            ::close(socket_descriptor);
            socket_descriptor = -1;
#endif
        }
    }

    int GetSocketDescriptor ()
    {
        return socket_descriptor;
    }

    bool IsAvailable()
    {
        return socket_descriptor >= 0;
    }

    int Read(void *buffer, int bufferLen)
    {
        int ret = ::recv(socket_descriptor, (char *) buffer, bufferLen, 0);//recv: window, linux 동일

        return ret;
    }

    int ReadWithTimeout(void* buffer, int bufferLen, unsigned int maxTimeInMsec)
    {
        fd_set set;
        struct timeval tv;
        tv.tv_sec = maxTimeInMsec / 1000;
        tv.tv_usec = (maxTimeInMsec % 1000) * 1000;

        FD_ZERO(&set);
        FD_SET(socket_descriptor, &set);
        int ret_select = select(socket_descriptor + 1, &set, NULL, NULL, &tv);

        // select error
        if (ret_select == -1)
        {
            LOGPRINT(YRLSocket, LOG_ERROR, ("select error\n"));
            return -2;
        }
        else if (ret_select == 0)
        {//time out
            LOGPRINT(YRLSocket, LOG_ERROR, ("time out\n"));
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
                if(ret < 0)
                {
                    LOGPRINT(YRLSocket, LOG_ERROR, ("Failed\n"));
                }
                return ret;
            }

            LOGPRINT(YRLSocket, LOG_ERROR, ("error\n"));
            return -4;
        }
    }

    void recvSocketBufferClear()
    {
        static int count_timeout = 0;
        char temp_char;
        long temp_long = 0;
        long i;

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
    bool buildAddrStructure(const std::string &address, unsigned short int port, sockaddr_in &addr)
    {
        if( address.empty() )
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

    void setLocalPort(unsigned short int localPortNumber, sockaddr_in &addr)
    {
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(localPortNumber);
    }
};
#endif //YRL_SOCKET_HPP