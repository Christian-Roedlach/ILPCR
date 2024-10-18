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
 * File:            05_tcp_server_testdata.cpp
 * Description:     test program: generate pose test data to test
 *                  07_draw_position_stream application
 */

#include <ilpcr/file_io.h>
#include <ilpcr/network.h>
#include <ilpcr/testdata.h>

#include <cstring> 
#include <iostream> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <chrono>
#include <cstdlib>

#include <pcl/common/impl/angles.hpp>
#include <arpa/inet.h>

#define SENDING_PERIOD_US 250000
#define LISTENING_ADDRESS "0.0.0.0"

using namespace ilpcr;

int main() 
{ 
    int serverSocket = 0;
    int clientSocket = 0;
    network_reply_t reply = ilpcr_nw_NONE;
    char buffer[1024] = { 0 };
    uint32_t i = 0;
    pose_data_t estimated_data;

    if (init_server(LISTENING_ADDRESS, &serverSocket, &clientSocket) == EXIT_SUCCESS)
    {
        do
        {
            estimated_data.tf_matrix = makeTransformationMatrix(i);
            /* simulate not converging pose estimations */
            estimated_data.id += (rand() % 5);
            estimated_data.successful_count = i;
            estimated_data.timestamp = std::chrono::high_resolution_clock::now();
            estimated_data.converged = true;

            // sending matrix 
            send(clientSocket, (char*) &estimated_data, sizeof(estimated_data), 0); 
                    
            reply = check_reply(clientSocket, buffer, sizeof(buffer));
            
            usleep(SENDING_PERIOD_US);
            estimated_data.id++;
            i++;
        } while(ilpcr_nw_ACK == reply);
    }

	// closing the socket. 
	close(serverSocket); 

	return 0; 
}


