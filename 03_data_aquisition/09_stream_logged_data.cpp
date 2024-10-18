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
 * File:            09_stream_logged_data.cpp
 * Description:     real-time scenario: stream log data acquired by 
 *                  real-time application (pipeline) for replaying pose
 *                  estimations  
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
#include <kbhit.h>

#include <pcl/console/parse.h>

#include <arpa/inet.h>

#define LISTENING_ADDRESS "0.0.0.0"

#define USAGE   "usage: ./09_stream_logged_data <log filename> [-nowait]" \
                "   <log filename>  filename of binary logfile" \
                "   -nowait         don't replay temporal data"

#define ILPCR_DEBUG 0
#define THREAD_SLEEP_US 5000

using namespace ilpcr;
using namespace std;

// --------------
// -----Main-----
// --------------
int
main(int argc, char** argv) 
{ 
    int retval = EXIT_FAILURE;
    string filename = "";
    string line = "";
    ifstream input_file;
    int serverSocket = 0;
    int clientSocket = 0;
    network_reply_t reply = ilpcr_nw_NONE;
    char buffer[1024] = { 0 };
    pose_data_t data_buffer;
    bool replay_timing = true;

    if (1 < argc)
    {
        filename = argv[1];
        retval = EXIT_SUCCESS;
    }
    else
    {
        cerr << "ERROR: no argument found - name of logfile is required!" << endl;
        cout << USAGE << endl;
        retval = EXIT_FAILURE;
    }

    if (pcl::console::find_argument (argc, argv, "-nowait") >= 0) {
        replay_timing = false;
        std::cout << "--> Timing replay DISABLED - sending messages instantly" << endl;
    } else {
        replay_timing = true;
        std::cout << "--> Timing replay ENABLED - sending messages as recorded" << endl;
    }

    if (EXIT_SUCCESS == retval)
    {
        retval = pose_log_check_header(&input_file, &filename);
    }

    if (EXIT_SUCCESS == retval)
    {
        if (init_server(LISTENING_ADDRESS, &serverSocket, &clientSocket) == EXIT_SUCCESS)
        {
            input_file.read((char *)(&data_buffer), sizeof(pose_data_t));

            const std::chrono::high_resolution_clock::time_point connection_time_zero = std::chrono::high_resolution_clock::now();
            const std::chrono::high_resolution_clock::time_point logfile_time_zero = data_buffer.timestamp;
            const auto time_to_add = connection_time_zero - logfile_time_zero;

            std::chrono::high_resolution_clock::time_point next_message;

            // send first pose data 
            send(clientSocket, (char*) &data_buffer, sizeof(data_buffer), 0);             
            reply = check_reply(clientSocket, buffer, sizeof(buffer));

            while(ilpcr_nw_ACK == reply)
            {
                input_file.read((char *)(&data_buffer), sizeof(pose_data_t));

                if (!(input_file.good()))
                    break;

                /* dismiss, if current pose did not converge */
                if (!data_buffer.converged)
                    continue;

                /* wait for next timestamp */
                if (replay_timing)
                {
                    next_message = data_buffer.timestamp + time_to_add;
                    while (next_message > std::chrono::high_resolution_clock::now())
                    {
                        // wait for next message
                        usleep(THREAD_SLEEP_US);
                    }
                }

                // sending pose data 
                send(clientSocket, (char*) &data_buffer, sizeof(data_buffer), 0); 
                        
                reply = check_reply(clientSocket, buffer, sizeof(buffer));
            } 
        }
    }

    cout << endl << "-----------------------------------------------------------------------------------" << endl;
    cout << "End of logfile" << filename << " reached" << endl << endl;
    cout << "Hit x [Enter] to close server!" << endl;
    
    char input = '\0';

    while (true)
    {
        while(!_kbhit()) {}
        input = getchar();

        if ('x' == input || 'X' == input)
        {
            break;
        }

        usleep(THREAD_SLEEP_US);
    }

	// closing the socket. 
	close(serverSocket); 
    // closing the file
    input_file.close();

	return retval; 
}


