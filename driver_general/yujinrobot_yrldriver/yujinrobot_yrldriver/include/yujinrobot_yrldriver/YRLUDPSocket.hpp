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
    sockaddr_in dest_addr;

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
};
#endif //YRL_UDP_SOCKET_HPP