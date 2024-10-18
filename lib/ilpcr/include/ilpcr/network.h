/*
 * ILPCR - Indoor Localization by Point Cloud Registration
 * Copyright (C) 2024 Christian Roedlach
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * 
 * File:            network.h
 * Description:     functions for network communication 
 */

#ifndef ILPCR_NETWORK_H
#define ILPCR_NETWORK_H

#include <iostream> 
#include <netinet/in.h> 

#define ACKNOWLEDGE "ACK"
#define END_SESSION "END"
#define IP_PORT 12345

#define NW_DEBUG 0

namespace ilpcr
{
    typedef enum {
        ilpcr_nw_NONE,
        ilpcr_nw_ACK,
        ilpcr_nw_END
    } network_reply_t;

    network_reply_t check_reply(int clientSocket, char* buffer, size_t buffer_size);
    int set_socket_address(std::string ip_address, in_port_t port, sockaddr_in *serverAddress);
    int init_server(std::string ip_address, int *serverSocket, int* clientSocket);
}


#endif // ILPCR_NETWORK_H

