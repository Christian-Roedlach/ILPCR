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
 * File:            07_draw_position_stream.cpp
 * Description:     real-time scenario: draw position stream received by
 *                  real-time pose estimation (pipeline)
 */

#include <ilpcr/types.h>
#include <ilpcr/viewer.h>
#include <ilpcr/testdata.h>
#include <ilpcr/network.h>
#include <ilpcr/file_io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> 

#include <pcl/common/impl/angles.hpp>
#include <Eigen/Eigen>

#include <thread>
#include <date/date.h>

#define DEBUG 0

#define COLOR_ARROW         0,0,255   // blue
#define COLOR_PATH          0,255,0   // green
#define COLOR_TEXT          0,0,0     // black
#define SCALE_TEXT_POS      0.03
#define SCALE_COORD_POINT   0.03
#define SCALE_ORIENTATION   0.25
#define CONNECTION_RETRIES  20

using namespace ilpcr;

int parse_cmd_args(int argc, char** argv, dph_settings_t* settings);
void print_usage(void);
int load_3d_model(dph_settings_t* settings);
int receive_pose(dph_settings_t* settings);

void print_usage(void) {
    const std::string usage = {
"Usage: ./07_draw_position_stream <3D model PCD [.pcd]> [server ip address] \n\
\n\
\t- [server ip adderss]:  IP address of the server (e.g. pipeline) \n\
\t                        default: localhost (127.0.0.1) \n\
\n"
    };

    std::cout << usage;
}

// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
    int retval = EXIT_FAILURE;
    dph_settings_t settings;
    
    dph_display_settings_t display_settings = 
    {
        .color_arrow = {COLOR_ARROW},
        .color_path = {COLOR_PATH},
        .color_text = {COLOR_TEXT},
        .scale_text_pos = SCALE_TEXT_POS,
        .scale_coord_point = SCALE_COORD_POINT,
        .scale_orientation = SCALE_ORIENTATION,
        .display_pos_numbers = true,
        .display_orientation_history = true,
    };

    settings.display_settings = display_settings;

    retval = parse_cmd_args(argc, argv, &settings);    

    if (EXIT_SUCCESS == retval)
        retval = load_3d_model(&settings);

    if (EXIT_SUCCESS == retval)
        retval = init_points_corr_pcd_colors(&settings);
    
    if (EXIT_SUCCESS == retval)
        retval = display_3d_model(&settings);

    std::thread receive_data(receive_pose, &settings);

    // main loop for visualizer
    while (!settings.viewer->wasStopped () && EXIT_SUCCESS == retval)
    {
        {
            std::lock_guard<std::mutex> lock(settings.mutex);
            settings.viewer->spinOnce ();
        }
        
        pcl_sleep(0.01);
    }

    receive_data.join();
    return retval;
}


int parse_cmd_args(int argc, char** argv, dph_settings_t* settings) 
{
    int retval = EXIT_FAILURE;
    std::string server_ip;
    
    std::vector<int> flist_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

    if (!flist_filename_indices.empty ())
    {
        settings->filename_3d_model = argv[flist_filename_indices[0]];
        std::cout << "3D model filename: " << settings->filename_3d_model << std::endl;
        retval = EXIT_SUCCESS;
    }
    else
    {
        std::cout << "ERROR: no .pcd file specified!" << std::endl;
        print_usage();

        retval = EXIT_FAILURE;
    }

    if (pcl::console::parse (argc, argv, "-s", server_ip) >= 0)
    {
        set_socket_address(server_ip, IP_PORT, &settings->serverAddress);    
        std::cout << "Server address: " << server_ip << ":" << IP_PORT << std::endl;
    } 
    else
    {
        set_socket_address("127.0.0.1", IP_PORT, &settings->serverAddress);    
        std::cout << "Server address not specified (-s <SERVER>)! Using: " << "127.0.0.1" << ":" << IP_PORT << std::endl;
    }

    /*
    if (pcl::console::find_argument (argc, argv, "-p") >= 0) {
        settings->displ_corr_pcd = true;
        std::cout << "\n --> Display corresponding .pcd enabled.\n";
    } else {
        settings->displ_corr_pcd = false;
        std::cout << "\n --> Display corresponding .pcd disabled.\n";
    }
    */

    settings->displ_corr_pcd = false;

    return retval;
}



int load_3d_model(dph_settings_t* settings)
{
    int retval = EXIT_FAILURE;

    CloudTypeXYZ::Ptr model_3d_pcd_xyz(new ( CloudTypeXYZ));
    settings->model_3d_pcd_xyz = model_3d_pcd_xyz;

    if (pcl::io::loadPCDFile (settings->filename_3d_model, *settings->model_3d_pcd_xyz) == -1)
    {
        std::cerr << "ERROR: was not able to open file " << settings->filename_3d_model << std::endl;
        retval = EXIT_FAILURE;
    }
    else
        retval = EXIT_SUCCESS;

    return retval;
}

int receive_pose(dph_settings_t* settings)
{
    int retval = EXIT_FAILURE;
    char buffer[1024] = { 0 }; 
    bool viewer_alive = true;
    uint32_t i = 0;
    uint32_t connection_retries = CONNECTION_RETRIES;
    int tcp_error_code;
    socklen_t error_code_size = sizeof(tcp_error_code);

    // creating socket 
	int clientSocket = socket(AF_INET, SOCK_STREAM, 0); 

    if (clientSocket == -1)
    {
        std::cerr << "Error creating socket" << std::endl;
        return EXIT_FAILURE;
    }

    while (connection_retries)
    {
        // sending connection request 
        retval = connect(clientSocket, (struct sockaddr*)&settings->serverAddress, 
                sizeof(settings->serverAddress)); 

        if (retval == -1)
        {
            std::cerr << "Error connecting to server - remaining retries: " << connection_retries << "..." << std::endl;
            connection_retries--;
            sleep(3);
        } 
        else
            break;
    }

	// sending data 
	const char* message = "Hello, server!"; 
	send(clientSocket, message, strlen(message), 0); 

    // recieving data 
    recv(clientSocket, buffer, sizeof(buffer), 0); 
	std::cout << "Message from server: " << buffer 
			<< std::endl; 

    while (viewer_alive)
	{
        pose_data_t received_data;

        // std::cout << "test1" << std::endl;
        
        getsockopt(clientSocket, SOL_SOCKET, SO_ERROR, &tcp_error_code, &error_code_size);

        // std::cout << "error_code = " << tcp_error_code << std::endl;

        if (EXIT_SUCCESS != tcp_error_code)
            break;
        
        // ssize_t send_retval = send(clientSocket, ACKNOWLEDGE, strlen(ACKNOWLEDGE) + 1, 0); 
        // std::cout << "send_retval = " << send_retval << std::endl;
        
        send(clientSocket, ACKNOWLEDGE, strlen(ACKNOWLEDGE) + 1, 0); 

        // receive matrix
        recv(clientSocket, (char*) &received_data, sizeof(received_data), 0); 
        {
            std::lock_guard<std::mutex> lock(settings->mutex);
            settings->points_tf_matrices.push_back(received_data.tf_matrix);
            settings->points_names.push_back("position " + std::to_string(i));
            settings->point_identifier.push_back(received_data.successful_count);
            retval = draw_pos_and_path(settings);
            viewer_alive = !settings->viewer->wasStopped();
        }

        // std::cout << "test3" << std::endl;

        std::cout << "Pose ID: " << received_data.id << ", message count: " << received_data.successful_count << ", timestamp: " << 
                //std::asctime(std::localtime(&received_data.timestamp)) << 
                date::format("%F %T UTC", time_point_cast<std::chrono::milliseconds>(received_data.timestamp)) << 
                std::endl;
        print_transformation(received_data.tf_matrix.matrix());

        // std::cout << "test4" << std::endl;

        if (EXIT_FAILURE == retval)
        {
            std::cerr << "ERROR: receive_post exited!" << std::endl;
            break;
        }
            
        i++;
    }

    // std::cout << "test5" << std::endl;

    if (EXIT_SUCCESS == tcp_error_code)
    {
        send(clientSocket, END_SESSION, strlen(END_SESSION) + 1, 0); 
    }

	// closing socket 
	close(clientSocket); 
    
    return retval;
}
