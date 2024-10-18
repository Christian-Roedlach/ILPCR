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
 * File:            01_RealSense_Capture_PCD_RGB_BAG.cpp
 * Description:     static research scenario: RGB-D capturing
 */

#include <ilpcr/camera.h>
#include <ilpcr/types.h>
#include <ilpcr/file_io.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <string>
#include <kbhit.h>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/file_io.h> 
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>
#include <Eigen/Geometry> 
#include <pcl/common/copy_point.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_export.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

// librealsense convenience funcions for rendering
#include "example/example.hpp"

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third-party/stb_image_write.h"

#define DEBUG   0

using namespace ilpcr;

#define CAPTURE_PERIOD_MIN 0.15f
#define CAPTURE_PERIOD_MAX 10.0f
#define CAPTURE_PERIOD_DEFAULT 2.0f
#define STORE_BAG_DATA 0
#define STORE_DEPTH_DATA 0

//#define _OPENMP

typedef struct {
    std::shared_ptr<window> capture_window;
    bool exitCapture = false;
    bool interactive_mode = true;
    float capture_period = CAPTURE_PERIOD_DEFAULT;
    bool headless_mode = false;
    bool timer_finished = true;
    bool timer_trigger = false;
} capture_settings_t;

void captureStoreData(capture_data_t *capture_data, 
                      capture_settings_t *capture_settings,
                      std::mutex *timer_mutex);

void print_usage(void);
int parse_cmd_args(int argc, char** argv, 
        capture_data_t *capture_data,
        capture_settings_t *capture_settings);
int store_data(rs2::frameset *frames, 
        capture_data_t* settings);
void timer_thread(capture_settings_t *settings,
        std::mutex *timer_mutex);

void print_usage(void) {
    const std::string usage = {
"\n Capture PCD, RGB, depth, and RAW data with Realsense camera (D415 or D455) \
\n\t \
\n\tUsage: \
\n\t\t ./01_RealSense_Capture_PCD_RGB_BAG <filename.pcd> [-v] [-s] [-hl] [-p <period in s>] [-rsp <rs2 preset name>] \n \
\n\t\t filename:         filename to store data to - files are stored to ../../files/ \
\n\t\t -s:               save .pcd files (optional) \
\n\t\t -hl:              headless mode: do not display capture window (optional) \
\n\t\t -rsp:             use RS2 filter preset (optional)\
\n\t\t -p <period in s>: automatic mode: capture every given time interval (optional) \
\n\t\t                   CAUTION: if capture window is shown, timing might be incorrect at small periods. \
\n\t\t                            use headless mode for better timing [-hl] \
\n\t\t -v:               open Point Cloud Viewer on last captured data (optional) \n\
\n\texample:  ./01_RealSense_Capture_PCD_RGB_BAG -s -hl -p 0.5 \
\n\n \
\n"
    };

    cout << usage;
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
    capture_data_t capture_data;
    capture_settings_t capture_settings;
    std::mutex timer_mutex;

    // capture realsence PCL 
    CloudTypeXYZ::Ptr realsense_pcl_original (new CloudTypeXYZ);
    capture_data.realsense_pcl_original = realsense_pcl_original;
    CloudTypeXYZ::Ptr realsense_pcl_filtered (new CloudTypeXYZ);
    capture_data.realsense_pcl_filtered = realsense_pcl_original;
    capture_data.image_number = 0;  
    capture_data.resX = 848;
    capture_data.resY = 480;
    capture_data.filename = "output_filename";
    capture_data.framerate = 30;
    capture_data.enableRGBstream = true;
    capture_data.capture_from_bag_file = false;
    set_r2s_filter_preset(&capture_data, rs2_filter_preset_1);

    capture_settings.interactive_mode = true;
    capture_settings.exitCapture = false;
    capture_settings.headless_mode = false;
    capture_settings.timer_finished = true;  // set to true for first entry
    capture_settings.timer_trigger = false;

    if (EXIT_FAILURE==parse_cmd_args(argc, argv, &capture_data, &capture_settings))
        return EXIT_FAILURE;

    // Create a simple OpenGL window for rendering:
    if (!capture_settings.headless_mode)
    {
        std::shared_ptr<window> capture_window (new window(1900, 720, "Scene Capture"));
        capture_settings.capture_window = capture_window;
    }

    capture_data.pipe = ilpcr::initCamera(&capture_data);

    bool exitCapture = false;

    std::thread timer_thread_handler(timer_thread, &capture_settings, &timer_mutex);

    while (!exitCapture) {
        pcl::ScopeTime t("  Frame capture");

        captureStoreData(   &capture_data,
                            &capture_settings,
                            &timer_mutex);

        capture_data.image_number++;

        {
            std::lock_guard<std::mutex> lock(timer_mutex);
            exitCapture = capture_settings.exitCapture;
        }
    }

    if (capture_settings.capture_window)
        capture_settings.capture_window->close();
    capture_data.pipe.stop();
    
    if (capture_data.display_viewer) {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        #ifdef ORIGINAL
        pcl::visualization::PCLVisualizer viewer_orig ("3D Viewer Original");
        viewer_orig.setBackgroundColor (1, 1, 1);
        pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> cloud_XYZ_color_handler (realsense_pcl, 0, 0, 0);
        viewer_orig.addPointCloud (realsense_pcl, cloud_XYZ_color_handler, "input cloud");
        viewer_orig.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
        viewer_orig.addCoordinateSystem (0.1);
        viewer_orig.initCameraParameters ();
        viewer_orig.setCameraPosition(0.60, -0.15, -0.40,        0.14, -0.11, 1.31,     0.01, -1.0, 0.026);
        #endif // ORIGINAL
        
        pcl::visualization::PCLVisualizer viewer_filtered ("3D Viewer Filtered");
        viewer_filtered.getRenderWindow()->GlobalWarningDisplayOff(); 
        viewer_filtered.setBackgroundColor (1, 1, 1);
        pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> cloud_XYZ_color_handler_filtered (capture_data.realsense_pcl_filtered, 0, 0, 0);
        viewer_filtered.addPointCloud (capture_data.realsense_pcl_filtered, cloud_XYZ_color_handler_filtered, "filtered input cloud");
        viewer_filtered.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered input cloud");
        viewer_filtered.addCoordinateSystem (0.1);
        viewer_filtered.initCameraParameters ();
        viewer_filtered.setCameraPosition(0.60, -0.15, -0.40,        0.14, -0.11, 1.31,     0.01, -1.0, 0.026);
        //--------------------
        //-----Viewer loop----
        //--------------------
        while (
            #ifdef ORIGINAL
            !viewer_orig.wasStopped () && 
            #endif // ORIGINAL
            !viewer_filtered.wasStopped ())
        {
            #ifdef ORIGINAL
            viewer_orig.spinOnce ();
            #endif // ORIGINAL
            viewer_filtered.spinOnce ();
            pcl_sleep(0.01);
        }
    }

    timer_thread_handler.join();

    return EXIT_SUCCESS;
}

int parse_cmd_args(int argc, char** argv, 
        capture_data_t *capture_data,
        capture_settings_t *capture_settings)
{
    int retval = EXIT_FAILURE;
    float capture_period;
    std::string text = "";

    std::vector<int> flist_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

    if (!flist_filename_indices.empty ())
    {
        capture_data->filename = pcl::getFilenameWithoutExtension(argv[flist_filename_indices[0]]);
        cout << "filename: " << capture_data->filename << endl;
        retval = EXIT_SUCCESS;
    }
    else
    {
        pcl::console::print_error("ERROR: no .pcd file specified!\n");
        print_usage();

        return EXIT_FAILURE;
    }

    if (pcl::console::find_argument (argc, argv, "-s") >= 0) {
        capture_data->save_pcd = true;
        std::cout << "--> Storage of data enabled." << std::endl;
    } else {
        capture_data->save_pcd = false;
        std::cout << "--> Storage of data disabled." << std::endl;
    }

    if (pcl::console::find_argument (argc, argv, "-v") >= 0) {
        capture_data->display_viewer = true;
        std::cout << "--> Viewer enabled." << endl;
    } else {
        capture_data->display_viewer = false;
        std::cout << "--> Viewer disabled." << endl;
    }

    if (pcl::console::parse (argc, argv, "-p", capture_period) >= 0) {
        capture_settings->interactive_mode = false;
        if (CAPTURE_PERIOD_MIN <= capture_period &&
                CAPTURE_PERIOD_MAX >= capture_period)
        {
            capture_settings->capture_period = capture_period;
            std::cout << "Entering automatic mode: capture period = " << 
                    capture_settings->capture_period << " s" << std::endl;
        } else {
            capture_settings->capture_period = CAPTURE_PERIOD_DEFAULT;
            std::cout << "ERROR: Capture period out of bounds: " << 
                    CAPTURE_PERIOD_MIN << " < p < " << CAPTURE_PERIOD_MAX << 
                    " - using default value" << std::endl;
            std::cout << "Entering automatic mode: capture period = " << 
                    capture_settings->capture_period << " s" << std::endl;
        }
    } else {
        capture_settings->interactive_mode = true;
        std::cout << "Entering interactive mode" << std::endl;
    }

    if (pcl::console::find_argument (argc, argv, "-hl") >= 0) {
        capture_settings->headless_mode = true;
        std::cout << "--> Headless mode (no capturing window displayed)" << endl;
    } 

    if (pcl::console::parse (argc, argv, "-rsp", text) >= 0) {
		retval = parse_preset(text, capture_data);
		if (EXIT_SUCCESS != retval)
			return retval;
	}

    return retval;
}

// ------------------------------------------------------------------
// Capture pc data from Realsense camera
// ------------------------------------------------------------------
void captureStoreData( capture_data_t *capture_data, 
                       capture_settings_t *capture_settings,
                       std::mutex *timer_mutex)
{
    rs2::frameset frames;
    rs2::colorizer color_map;
    // rs2::rates_printer printer;

    if (capture_settings->interactive_mode)
    {
        bool take_new_scene = true;
        char input = '\0';

        while (take_new_scene) // Application still alive?
        {
            rs2::frameset frames = capture_data->pipe.wait_for_frames().    // Wait for next set of frames from the camera
                                // apply_filter(printer).     // Print each enabled stream frame rate
                                apply_filter(color_map);   // Find and colorize the depth data

            if (!capture_settings->headless_mode)
            {
                // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
                // Each texture is displayed on different viewport according to it's stream unique id
                capture_settings->capture_window->show(frames);

                if(*capture_settings->capture_window) {} // update window
            }
        
            cout << "\nHit <any key> to take another sample, <s> to store scene, or <x> to store scene and exit program: " << endl;
            //char input = cin.get();
            while(!_kbhit()) {}
        
            input = getchar();
    
            if (input == 's' || input == 'S')
                take_new_scene = false;
            if (input == 'x' || input == 'X')
            {
                take_new_scene = false;
                {
                    std::lock_guard<std::mutex> lock(*timer_mutex);
                    capture_settings->exitCapture = true;
                }
            }

            // store data if capture is finished
            if (!take_new_scene)
            {
                // flush input
                cin.clear();

                store_data(&frames, capture_data);            
            }
        }
    }
    else
    {
        bool timer_finished = false;
        while (!timer_finished)
        {
            {
                std::lock_guard<std::mutex> lock(*timer_mutex);
                timer_finished = capture_settings->timer_finished;
                if (timer_finished)
                {
                    capture_settings->timer_finished = false;
                    capture_settings->timer_trigger = true;
                }
            }
            if (!timer_finished)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        rs2::frameset frames = capture_data->pipe.wait_for_frames().    // Wait for next set of frames from the camera
                                // apply_filter(printer).     // Print each enabled stream frame rate
                                apply_filter(color_map);   // Find and colorize the depth data

        if (!capture_settings->headless_mode)
        {
            // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
            // Each texture is displayed on different viewport according to it's stream unique id
            capture_settings->capture_window->show(frames);

            if(*capture_settings->capture_window) {} // update window
        }

        store_data(&frames, capture_data);
    }
}

int store_data(rs2::frameset *frames, capture_data_t* settings)
{
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points, points_filtered; 

#if (STORE_BAG_DATA)
    // saving RAW data (.bag)
    std::string filename_raw_data = "../../files/" + settings->filename + std::to_string(settings->image_number) + "_";
    rs2::save_single_frameset single_saver( filename_raw_data );
    frames->apply_filter(single_saver);
    cout << "raw data stored to: " << filename_raw_data << "<frame_number>.bag" << endl;
#endif //(STORE_BAG_DATA)

    rs2::depth_frame depth_frame = frames->get_depth_frame();
    rs2::video_frame video_frame = frames->get_color_frame();
    
    std::stringstream png_file;
    std::string png_filename = "../../files/" + settings->filename + std::to_string(settings->image_number) + ".png";
    png_file << png_filename;
    stbi_write_png(png_file.str().c_str(), video_frame.get_width(), video_frame.get_height(),
            video_frame.get_bytes_per_pixel(), video_frame.get_data(), video_frame.get_stride_in_bytes());

    cout << "PNG stored to: " << png_filename << ", ";

#if (STORE_DEPTH_DATA)
    // save depth_frame
    std::string depth_filename = "../../files/" + settings->filename + std::to_string(settings->image_number) + ".depth";
    std::ofstream depth_frame_out(depth_filename, std::ofstream::binary);
    depth_frame_out.write(static_cast<const char*>
            (depth_frame.get_data()), 
                depth_frame.get_height() * 
                depth_frame.get_stride_in_bytes());
    depth_frame_out.close();   
#endif //(STORE_DEPTH_DATA)  

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth_frame);
    // Convert to PCL data
    auto realsense_pcl_temp = points_to_pcl(points);
    copyPointCloud(*realsense_pcl_temp, *settings->realsense_pcl_original);

    // Filter depth frame
    auto filtered = ilpcr::filterRS2depthImage(depth_frame, settings->rs2_filter_preset);
    // Generate the pointcloud and texture mappings
    points_filtered = pc.calculate(filtered);  

    // Convert to PCL data
    auto realsense_pcl_temp_filtered = points_to_pcl(points_filtered);
    copyPointCloud(*realsense_pcl_temp_filtered, *settings->realsense_pcl_filtered);

    if (settings->save_pcd)
    {
        std::string tempFileName;
        tempFileName = "../../files/" + settings->filename + std::to_string(settings->image_number) + "_original.pcd";
        pcl::io::savePCDFileBinary(tempFileName, *settings->realsense_pcl_original);
        std::cout << "PCD stored to: " << tempFileName << " and ";
        tempFileName = "../../files/" + settings->filename + std::to_string(settings->image_number) + "_filtered.pcd";
        pcl::io::savePCDFileBinary(tempFileName, *settings->realsense_pcl_filtered);
        std::cout << tempFileName << std::endl;
    }

    /* PCL throws exeption in case of write failure */
    return EXIT_SUCCESS;
}

void timer_thread(capture_settings_t *settings, std::mutex *timer_mutex)
{
    uint32_t period_ms = (uint32_t) (settings->capture_period * 1000);
    capture_settings_t settings_local_copy;
    settings_local_copy.exitCapture = false;
    
    while(!settings_local_copy.exitCapture)
    {
        {
            std::lock_guard<std::mutex> lock(*timer_mutex);
            settings_local_copy = *settings;
        }

        if (true == settings_local_copy.timer_trigger)
        {
            {
                std::lock_guard<std::mutex> lock(*timer_mutex);
                settings->timer_trigger = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
            {
                std::lock_guard<std::mutex> lock(*timer_mutex);
                settings->timer_finished = true;
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}