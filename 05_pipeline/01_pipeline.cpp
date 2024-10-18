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
 * File:            01_pipeline.cpp
 * Description:     real-time scenario: pose estimation pipeline
 *                  process real-time pose estimation with applied 
 *                  \ac{PCR} presets and \ac{C-Presets}
 */

#include <ilpcr/types.h>
#include <ilpcr/camera.h>
#include <ilpcr/file_io.h>
#include <ilpcr/feature_estimation.h>
#include <ilpcr/corr_estimation.h>
#include <ilpcr/network.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <kbhit.h>

#include <thread>
#include <date/date.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/narf.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/file_io.h> 
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/common/angles.h>
#include <Eigen/Geometry> 
#include <pcl/common/copy_point.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <pcl/features/fpfh_omp.h>

#define ILPCR_DEBUG 1
#include <ilpcr/debug.h>


using namespace ilpcr;

//#define _OPENMP
#define res1280x720 1280, 720 
#define res640x480  640, 480
#define res848x480  848, 480
#define LISTENING_ADDRESS "0.0.0.0"
#define NEW_DATA_POLLING_PERIOD_US 1000
#define STORAGE_BUFFER_SIZE 20
#define LOG_ONLY_CONVERGED false
#define USER_INPUT_THREAD_SLEEP_US  5000


void print_usage(void);
int parse_cmd_args(int argc, char** argv, capture_data_t* settings);
void viewer_actualize(pcl::visualization::PCLVisualizer* viewer_ptr, capture_data_t *capture_data, narf_data_t *narf_pcd);
int data_transmit_server(pose_data_t *transmit_data, std::mutex *transmit_data_mutex, capture_data_t *capture_data);
void server_thread(pose_data_t *transmit_data, std::mutex *transmit_data_mutex, capture_data_t *capture_data);
int data_log_thread(pose_data_t *data, std::mutex *data_mutex, capture_data_t *capture_data);
int write_log_data(std::ofstream *logfile, std::vector<pose_data_t> *storage, std::string *logfile_name);
int write_log_header(std::ofstream &logfile, capture_data_t *capture_data);
void user_input_thread(capture_data_t *capture_data);

void print_usage(void) {
    const std::string usage = {
"\n Pipeline for position estimation \
\n\t \
\n\tusage: ./02_pipeline <target_pointcloud_descriptor [.pcd]> <-t <fpfh|narf>> <-p <preset name>> [-v] [-s] [-rsp <rs2 preset name>] \
\n\t- -t <fpfh|narf>: feature type \
\n\t- -p <preset name>: sample consensus preset name \
\n\t- -v: open viewer (optional) \
\n\t- -s: save files (optional) - files are stored to ../../files/ \
\n\t- -rsp: use RS2 filter preset (optional)\n\
\n\n \
\n"
    };

    cout << usage;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* cookie)
{
    //pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    bool* finished = (bool* ) cookie;
    if ((event.getKeySym () == "8") && event.keyDown ())
    {
        std::cout << "8 was pressed => session finished" << std::endl;  
        *finished = true;
    }
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
    int retval = EXIT_FAILURE;
    capture_data_t capture_data;
    narf_data_t narf_pcd;
    fpfh_data_t fpfh_data;
    bool converged = false;
    pose_data_t pose_data;
    pose_data_t pose_data_cache_network;
    std::mutex pose_data_cache_network_mutex;
    pose_data_t pose_data_cache_log;
    std::mutex pose_data_cache_log_mutex;

    capture_data.exit = false;
    capture_data.image_number = 0;
    capture_data.resX = 848;
    capture_data.resY = 480;
    capture_data.filename = "";
    capture_data.framerate = 30;
    capture_data.enableRGBstream = false;
    capture_data.capture_from_bag_file = false;  
    set_r2s_filter_preset(&capture_data, rs2_filter_preset_1);

    pcl::visualization::PCLVisualizer* viewer_ptr = NULL;

    retval = parse_cmd_args(argc, argv, &capture_data);
    
    if (EXIT_FAILURE != retval)
    {
        switch (capture_data.feature_type)
        {
            case ilpcr_narf:
                ilpcr::init_pcd(&narf_pcd, capture_data.pcr_preset_name);
                retval = ilpcr::loadDestinationCloud(&capture_data, &narf_pcd);
                capture_data.narf_data = &narf_pcd;
                break;

            case ilpcr_fpfh:
                ilpcr::init_pcd(&fpfh_data, capture_data.pcr_preset_name);
                retval = ilpcr::loadDestinationCloud(&capture_data, &fpfh_data);
                capture_data.fpfh_data = &fpfh_data;
                break;

            default:
                std::cerr << "ERROR: Feature type unknown" << std::endl;
                retval = EXIT_FAILURE;
        }
    }
    
    if (EXIT_FAILURE != retval)
    {
        capture_data.pipe = ilpcr::initCamera(&capture_data);
        if (capture_data.display_viewer)
        {
            viewer_ptr = (new pcl::visualization::PCLVisualizer("3D Viewer Filtered"));// viewer_filtered ("3D Viewer Filtered");
            viewer_ptr->setBackgroundColor (1, 1, 1);
            viewer_ptr->addCoordinateSystem (0.1);
            viewer_ptr->initCameraParameters ();
            viewer_ptr->setCameraPosition(0.60, -0.15, -0.40,        0.14, -0.11, 1.31,     0.01, -1.0, 0.026);
        }

        /* set application state to running */
        capture_data.application_state = pipeline_state_running;

        /* start TCP server thread */
        std::thread thread_tcp_server(server_thread, &pose_data_cache_network, &pose_data_cache_network_mutex, &capture_data);
        ILPCR_DEBUG_EXECUTION(size_t frame_number = 0)

        /* start pose logging thread */
        std::thread thread_pose_logging(data_log_thread, &pose_data_cache_log, &pose_data_cache_log_mutex, &capture_data);

        /* start user input thread */
        std::thread thread_user_input(user_input_thread, &capture_data);
       
        /* pose estimation loop */
        while (pipeline_state_running == capture_data.application_state) 
        {
            ILPCR_DEBUG_EXECUTION(std::cout << "----- starting frame number: " << ++frame_number << " -----" << std::endl)
            pcl::ScopeTime t("Pose estimation frame");

            /* take timestamp and reset pose data */
            pose_data.converged = false;
            pose_data.tf_matrix.matrix() = Eigen::Matrix4f::Zero(4,4);   
            pose_data.timestamp = std::chrono::high_resolution_clock::now();

            ilpcr::convertRealSenceToPCL(&capture_data);

            if (capture_data.save_pcd)
            {
                std::string tempFileName;
                tempFileName = "../../files/" + capture_data.filename + std::to_string(capture_data.image_number) + "_filtered.pcd";
                pcl::io::savePCDFileASCII(tempFileName, *capture_data.realsense_pcl_filtered);
                cout << "PCD stored to: " << tempFileName << endl;
            }

            pose_data.timestamp_post_capture = std::chrono::high_resolution_clock::now();

            capture_data.image_number++;

            switch (capture_data.feature_type)
            {
                case ilpcr_narf:
                    ilpcr::feature_estimation(&capture_data, &narf_pcd);
                    pose_data.timestamp_post_fest = std::chrono::high_resolution_clock::now();
                    converged = ilpcr::matchPointClouds(&capture_data, &narf_pcd);
                    pose_data.timestamp_post_pose_est = std::chrono::high_resolution_clock::now();

                    if (capture_data.display_viewer && NULL != viewer_ptr && converged)
                    {
                        viewer_actualize(viewer_ptr, &capture_data, &narf_pcd);
                    }

                    break;

                case ilpcr_fpfh:
                    {
                        ILPCR_DEBUG_EXECUTION(pcl::ScopeTime t("Normals and FPFH estimation"))
                        ilpcr::feature_estimation(&capture_data, &fpfh_data);
                    }

                    pose_data.timestamp_post_fest = std::chrono::high_resolution_clock::now();

                    {
                        ILPCR_DEBUG_EXECUTION(pcl::ScopeTime t3("Registration"))
                        converged = ilpcr::matchPointClouds(&capture_data, &fpfh_data);
                    }
                    
                    pose_data.timestamp_post_pose_est = std::chrono::high_resolution_clock::now();

                    ILPCR_DEBUG_EXECUTION(std::cout << "converged: " << converged << 
                            ", fitness score: " << fpfh_data.fitnessScore << 
                            ", inliers: " << fpfh_data.inliers << std::endl)

                    break;

                default:
                    std::cerr << "ERROR: Feature type unknown" << std::endl;
                    converged = false;
                    retval = EXIT_FAILURE;
            }

            /* get memory info */
            if (EXIT_SUCCESS == retval)
            {
                pose_data.memory = capture_data.memory;
                retval = get_process_memory(&pose_data.memory.ram_VmSize, &pose_data.memory.ram_VmRSS);
            }

            /* copy results */
            if (EXIT_SUCCESS == retval && converged)
            {
                ILPCR_DEBUG_EXECUTION(print_transformation(capture_data.transformation.matrix()))
                
                pose_data.converged = converged;
                pose_data.tf_matrix = capture_data.transformation;
                pose_data.successful_count++;

                /* copy estimated pose data for network transmission */
                std::lock_guard<std::mutex> lock(pose_data_cache_network_mutex);
                pose_data_cache_network = pose_data;
            }

            /* copy estimated pose data for logging */
            std::lock_guard<std::mutex> lock(pose_data_cache_log_mutex);
            pose_data_cache_log = pose_data;

            /* increment pose id */
            pose_data.id++;

            if (EXIT_FAILURE == retval)
            {
                capture_data.application_state = pipeline_state_failure_capturing;
                break;
            }
        }

        if (viewer_ptr)
            viewer_ptr->close();

        capture_data.pipe.stop();
        thread_pose_logging.join();
        thread_user_input.join();
        
        // don't wait for thread_tcp_server to be finished, as it might be listening for an incoming connection forever
        // thread_tcp_server.join();
    }
    
    return retval;
}

void viewer_actualize(pcl::visualization::PCLVisualizer* viewer_ptr, capture_data_t *capture_data, narf_data_t *narf_pcd)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> cloud_XYZ_color_handler_filtered (capture_data->realsense_pcl_filtered, 0, 0, 0);
    viewer_ptr->addPointCloud (capture_data->realsense_pcl_filtered, cloud_XYZ_color_handler_filtered, "filtered input cloud");
    viewer_ptr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered input cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> keypoints_color_handler (narf_pcd->src_narf_keypoints_XYZ_ptr, 0, 255, 0);
    viewer_ptr->addPointCloud<PointTypeXYZ> (narf_pcd->src_narf_keypoints_XYZ_ptr, keypoints_color_handler, "keypoints");
    viewer_ptr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
    
    bool finished = false;
    cout << "Viewer started. Press 8 to continue..." << endl; 
    //--------------------
    //-----Viewer loop----
    //--------------------
    while (!finished && !viewer_ptr->wasStopped ())
    {
        viewer_ptr->spinOnce ();
        viewer_ptr->registerKeyboardCallback(keyboardEventOccurred, (void*) &finished);
        pcl_sleep(0.01);
    }

    viewer_ptr->removePointCloud("filtered input cloud");
    viewer_ptr->removePointCloud("keypoints");

    if (viewer_ptr->wasStopped())
        capture_data->display_viewer = false;
}

int parse_cmd_args(int argc, char** argv, capture_data_t* settings) 
{
    int retval = EXIT_FAILURE;
    std::string text = "";

    std::vector<int> flist_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

    if (!flist_filename_indices.empty ())
    {
        settings->filename = pcl::getFilenameWithoutExtension(argv[flist_filename_indices[0]]);
        cout << "filename: " << settings->filename << endl;
        retval = EXIT_SUCCESS;
    }
    else
    {
        cerr << "ERROR: no .pcd file for destination point cloud specified!" << endl;
        print_usage();

        retval = EXIT_FAILURE;
    }

    if (pcl::console::find_argument (argc, argv, "-s") >= 0) {
        settings->save_pcd = true;
        std::cout << "--> Storage of data enabled." << endl;
    } else {
        settings->save_pcd = false;
        std::cout << "--> Storage of data disabled." << endl;
    }

    if (pcl::console::find_argument (argc, argv, "-v") >= 0) {
        settings->display_viewer = true;
        std::cout << "--> Viewer enabled. --> endl";
    } else {
        settings->display_viewer = false;
        std::cout << "--> Viewer disabled." << endl;
    }

    if (pcl::console::parse (argc, argv, "-rsp", text) >= 0) {
		retval = parse_preset(text, settings);
		if (EXIT_SUCCESS != retval)
			return retval;
	}

    if (pcl::console::parse (argc, argv, "-p", text) >= 0) {
		retval = parse_preset(text, &settings->pcr_preset_name);
		if (EXIT_SUCCESS != retval)
			return retval;
	}
    else
    {
        cerr << "ERROR: no sample consensus preset specified!" << endl;
        print_usage();

        retval = EXIT_FAILURE;
    }


    retval = parse_feature_type(argc, argv, &settings->feature_type);

    return retval;
}

void server_thread(pose_data_t *transmit_data, std::mutex *transmit_data_mutex, capture_data_t *capture_data)
{
    int retval = EXIT_FAILURE;
    while (pipeline_state_running == capture_data->application_state)
    {
        retval = data_transmit_server(transmit_data, transmit_data_mutex, capture_data);

        if (EXIT_SUCCESS != retval)
            capture_data->application_state = pipeline_state_failure_log;

        sleep(1);
    } 
}

int data_transmit_server(pose_data_t *transmit_data, std::mutex *transmit_data_mutex, capture_data_t *capture_data)
{
    int serverSocket = 0;
    int clientSocket = 0;
    network_reply_t reply = ilpcr_nw_NONE;
    char buffer[1024] = { 0 };
    uint32_t count = 0;
    pose_data_t transmit_data_local_copy;
    int retval = EXIT_FAILURE;

    retval = init_server(LISTENING_ADDRESS, &serverSocket, &clientSocket);

    if (EXIT_SUCCESS == retval)
    {
        do
        {
            {
                std::lock_guard<std::mutex> lock(*transmit_data_mutex);
                if (transmit_data->successful_count > count)
                    transmit_data_local_copy = *transmit_data;
            }

            //std::cout << "count: " << count << " transmit.count: " << transmit_data_local_copy.count << std::endl;

            if (transmit_data_local_copy.successful_count > count)
            {
                count = transmit_data_local_copy.successful_count;
                // sending matrix 
                /*ssize_t bytes_sent = */send(clientSocket, (char*) &transmit_data_local_copy, sizeof(transmit_data_local_copy), 0); 
                //std::cout << "data sent: " << bytes_sent << std::endl;
                reply = check_reply(clientSocket, buffer, sizeof(buffer));
                //std::cout << "reply: " << reply << std::endl;
            }
            else
                usleep(NEW_DATA_POLLING_PERIOD_US);
        } while(pipeline_state_running == capture_data->application_state && 
                (ilpcr_nw_ACK == reply || ilpcr_nw_NONE == reply));
    }
    else
        /* set state to failure if server initialization or handshake failed */
        capture_data->application_state = pipeline_state_failure_network;
    
    std::cout << "Client disconnected" << std::endl;

    retval = shutdown(serverSocket, SHUT_RDWR);
    // closing the socket. 
	close(serverSocket); 

    return retval;
}

int data_log_thread(pose_data_t *data, std::mutex *data_mutex, capture_data_t *capture_data)
{
    int retval = EXIT_FAILURE;
    uint32_t count = 0;
    // uint32_t count_successful = 0;
    pose_data_t data_local_copy;
    std::vector<pose_data_t> storage;

    auto now = std::chrono::system_clock::now();
    time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream now_datetime;
    now_datetime << std::put_time(std::gmtime(&now_time_t), "%Y-%m-%d_%X");
    std::string logfile_name = "../log/pose_log_" + now_datetime.str() + "_UTC.bin";

    std::ofstream logfile{logfile_name, std::ofstream::binary | std::ofstream::out};

    if (logfile.is_open())
    {
        retval = write_log_header(logfile, capture_data);
    }
    else
    {
        retval = EXIT_FAILURE;
        std::cerr << "ERROR: opening pose logfile " << logfile_name << " FAILED!" << std::endl;
    }
    
    while (pipeline_state_running == capture_data->application_state)
    {
        {
            std::lock_guard<std::mutex> lock(*data_mutex);
            if (data->id != count)
                data_local_copy = *data;
        }

        #if (LOG_ONLY_CONVERGED)
        {
            if (data_local_copy.successful_count != count_successful)
            {
                count_successful = data_local_copy.successful_count;

                storage.push_back(data_local_copy);
            }
            else
                usleep(NEW_DATA_POLLING_PERIOD_US);
        } 
        #else // LOG_ONLY_CONVERGED
        {
            if (data_local_copy.id != count)
            {
                count = data_local_copy.id;

                storage.push_back(data_local_copy);
            }
            else
                usleep(NEW_DATA_POLLING_PERIOD_US);
        }
        #endif // LOG_ONLY_CONVERGED

        if (STORAGE_BUFFER_SIZE <= storage.size())
        {
            retval = write_log_data(&logfile, &storage, &logfile_name);
        }

        if (EXIT_SUCCESS != retval)
            capture_data->application_state = pipeline_state_failure_log;
    } 

    /* write last block of data on graceful exit */
    if (0 < storage.size())
    {
        retval = write_log_data(&logfile, &storage, &logfile_name);
    }

    logfile.close();

    return retval;
}

int write_log_data(std::ofstream *logfile, std::vector<pose_data_t> *storage, std::string *logfile_name)
{
    int retval = EXIT_FAILURE;
    
    if (logfile->is_open())
    {
        logfile->write(
                reinterpret_cast<const char *>(storage->data()),
                storage->size() * sizeof(pose_data_t));
        logfile->flush();

        if (logfile->bad())
        {
            retval = EXIT_FAILURE;
            std::cerr << "ERROR: writing to pose logfile " << *logfile_name << " FAILED!" << std::endl;
        }
        else
        {
            // clear content of storage for following data
            storage->clear();
            retval = EXIT_SUCCESS;
        }
    }
    else
    {
        retval = EXIT_FAILURE;
        std::cerr << "ERROR: writing to pose logfile " << *logfile_name << " FAILED!" << std::endl;
    }

    return retval;
}

int write_log_header(std::ofstream &logfile, capture_data_t *capture_data)
{
    using namespace std;

    int retval = EXIT_FAILURE;

    logfile << POSE_LOG_HEADER_START << endl;

    logfile << "Starting time: " << 
                date::format("%F %T UTC", time_point_cast<std::chrono::milliseconds>(
                        std::chrono::high_resolution_clock::now())) << endl;
    logfile << "Destination cloud: " << capture_data->filename << endl;
    logfile << "RS2 preset: " << capture_data->rs2_filter_preset->preset_name << endl;

    switch (capture_data->feature_type)
    {
        case ilpcr_narf:
            logfile << "Feature type: NARF" << endl;

            if (NULL != capture_data->narf_data)
            {
                logfile << "Registration preset: " << 
                        capture_data->narf_data->pcr_preset->preset_name << endl;
                retval = EXIT_SUCCESS;
            }
            else
            {
                logfile << "ERROR: NARF preset == NULL POINTER" << endl;
                std::cerr << "ERROR: NARF preset == NULL POINTER" << endl;
                retval = EXIT_FAILURE;
            }

            break;

        case ilpcr_fpfh:
            logfile << "Feature type: FPFH" << endl;
            
            if (NULL != capture_data->fpfh_data)
            {
                logfile << "Registration preset: " << 
                        capture_data->fpfh_data->pcr_preset->preset_name << endl;
                logfile << "GPU Memory: Destination Cloud: " << 
                        capture_data->fpfh_data->memory_gpu_pre_dst_cloud -
                        capture_data->fpfh_data->memory_gpu_post_dst_cloud << " bytes" << endl;
                retval = EXIT_SUCCESS;
            }
            else
            {
                logfile << "ERROR: FPFH preset == NULL POINTER" << endl;
                std::cerr << "ERROR: FPFH preset == NULL POINTER" << endl;
                retval = EXIT_FAILURE;
            }

            break;

        default:
            logfile << "Feature type: ERROR: UNKNOWN" << endl;
            std::cerr << "ERROR: Feature type unknown" << std::endl;
            retval = EXIT_FAILURE;
    }

    logfile << POSE_LOG_HEADER_END << endl;

    return retval;
}

void user_input_thread(capture_data_t *capture_data)
{
    char input = '\0';

    while (pipeline_state_running == capture_data->application_state)
    {
        while(!_kbhit()) {}
        input = getchar();

        if ('x' == input || 'X' == input)
        {
            capture_data->application_state = pipeline_state_stopped_by_user;
            break;
        }

        usleep(USER_INPUT_THREAD_SLEEP_US);
    }
}
