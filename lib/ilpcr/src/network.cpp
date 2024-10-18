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
 * File:            network.cpp
 * Description:     functions for network communication 
 */

#include "network.h"

#include <cstring> 
#include <sys/socket.h> 
#include <arpa/inet.h>

namespace ilpcr
{
    network_reply_t check_reply(int clientSocket, char* buffer, size_t buffer_size)
    {
        network_reply_t retval = ilpcr_nw_NONE;

        recv(clientSocket, buffer, buffer_size, 0); 
        if (0 == strncmp("ACK", buffer, 4))
            retval = ilpcr_nw_ACK;
        else if (0 == strncmp("END", buffer, 4))
            retval = ilpcr_nw_END;

        #if (NW_DEBUG)
        std::cout << "Received " << buffer << ", reply: " << retval
                << std::endl; 
        #endif // NW_DEBUG

        return retval;
    }

    int set_socket_address(std::string ip_address, in_port_t port, sockaddr_in *serverAddress)
    {
        serverAddress->sin_family = AF_INET; 
	    serverAddress->sin_port = htons(port); 
        return inet_aton(ip_address.c_str(), &serverAddress->sin_addr);
    }

    int init_server(std::string ip_address, int *serverSocket, int* clientSocket)
    {
        sockaddr_in serverAddress; 
        sockaddr_in clientAddress;
        int addrlen = sizeof(clientAddress); 
        char buffer[1024] = { 0 };
        int retval = EXIT_FAILURE;
        network_reply_t reply = ilpcr_nw_NONE;
        const int reusePort = 1;

	    // creating socket 
	    *serverSocket = socket(AF_INET, SOCK_STREAM, 0); 

        if (*serverSocket == -1)
        {
            std::cerr << "Error creating socket" << std::endl;
            return EXIT_FAILURE;
        }

        set_socket_address(ip_address, IP_PORT, &serverAddress);
        
        /* set SO_REUSEPORT flag to be able to listen again after client disconnection */
        setsockopt(*serverSocket, SOL_SOCKET, SO_REUSEPORT, &reusePort, sizeof(reusePort));

	    // binding socket. 
	    retval = bind(*serverSocket, (struct sockaddr*)&serverAddress, 
	    	sizeof(serverAddress)); 

        if (retval == -1)
        {
            std::cerr << "Error binding socket" << std::endl;
            return EXIT_FAILURE;
        }

        std::cout << "Listening on port " << IP_PORT << " ..." << std::endl; 

	    // listening to the assigned socket 
	    listen(*serverSocket, 5); 

        // accepting connection request 
	    *clientSocket = accept(*serverSocket, 
                (struct sockaddr*) &clientAddress, 
	    	    (socklen_t*) &addrlen); 

        std::cout << "Client connected: ip = " 
                << inet_ntoa(clientAddress.sin_addr) << ":" << ntohs(clientAddress.sin_port) << std::endl;

	    // recieving data 
	    recv(*clientSocket, buffer, sizeof(buffer), 0); 
	    std::cout << "Message from client: " << buffer 
	    		<< std::endl; 
    
        // sending data 
        const char* message = "Hello, client!"; 
        send(*clientSocket, message, strlen(message), 0); 

        reply = check_reply(*clientSocket, buffer, sizeof(buffer));

        if (ilpcr_nw_ACK == reply)
            return EXIT_SUCCESS;
        else
            return EXIT_FAILURE;
    }
}


