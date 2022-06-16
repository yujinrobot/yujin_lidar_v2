/*********************************************************************
*  Copyright (c) 2020, YujinRobot Corp.
*  
*  Ju Young Kim, jykim3@yujinrobot.com
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
#include <string.h>

#include <unistd.h>//for close()
#include <arpa/inet.h>
#include <sys/socket.h>

#include "../../include/yujinrobot_yrldriver/yujinrobot_yrldriver.hpp"

int main( int argc, char ** argv)
{
    std::cout << "========================================" << std::endl; 
    std::cout << "           Test YRL driver          " << std::endl; 
    std::cout << "========================================\n" << std::endl;
 
    // YRLTCPSocket *mTCPClientSocket = new YRLTCPSocket
    
    // TCPSocket("tcp_client");


    // int client_socket_descriptor;
    // int server_socket_descriptor;
    // sockaddr_in client_addr;
    // sockaddr_in server_addr;
    // socklen_t client_addr_size;

    // unsigned char message[1024];
    // int recvMsize = 0;

    // server_socket_descriptor = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    // if (server_socket_descriptor == -1)
    // {
    //     printf("socket() error\n");
    // }

    // memset(&server_addr, 0, sizeof(server_addr));
    // server_addr.sin_family = AF_INET;
    // server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    // server_addr.sin_port = htons(1234);

    // if (bind(server_socket_descriptor, (sockaddr *) &server_addr, sizeof(server_addr)) == -1)
    // {
    //     printf("bind() error\n");
    // }
    // else
    // {
    //     printf("bind()\n");
    // }

    // if (listen(server_socket_descriptor, 5) == -1)
    // {
    //     printf("listen() error\n");
    // }
    // else
    // {
    //     printf("listen()\n");
    // }

    // client_addr_size = sizeof(client_addr);
    
    // while (1)
    // {
    //     client_socket_descriptor = accept(server_socket_descriptor,(sockaddr *) &client_addr, &client_addr_size);
    //     if (client_socket_descriptor == -1)
    //     {
    //         printf("accept() error\n");
    //     }
    //     else
    //     {
    //         printf("Connected client\n");
    //     }
        
    //     // while ( (recvMsize = recv(client_socket_descriptor, message, sizeof(message), 0)) != 0 )
    //     // {
    //     //     send(client_socket_descriptor, message, recvMsize, 0);
    //     // }

    //     close(client_socket_descriptor);
    // }
    
    // close(server_socket_descriptor);
    
    
    return 0;
}