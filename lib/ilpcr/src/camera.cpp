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
 * File:            camera.cpp
 * Description:     functions for capturing and filtering 
 *                  from Intel RealSense cameras
 */

#include "camera.h"
#include <fstream>
#include <string.h>
#include <iostream>
#include <omp.h>

namespace ilpcr {
    rs2::pipeline initCamera(capture_data_t *capture_data) 
    // int resX, int resY, uint8_t framerate, bool enableRGBstream
    {        
        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        rs2::config cfg;
                    
        if (!capture_data->capture_from_bag_file)
        {
            bool load_camera_settings = false;

            cfg.enable_stream(RS2_STREAM_DEPTH, capture_data->resX, capture_data->resY, 
                    RS2_FORMAT_Z16, capture_data->framerate);
            if (capture_data->enableRGBstream) {
                cfg.enable_stream(RS2_STREAM_COLOR, capture_data->resX, capture_data->resY, 
                        RS2_FORMAT_RGB8, capture_data->framerate);
            }
            auto camera_profile = pipe.start(cfg);

            // load config from file
            std::string json_file_name;
            rs2::device dev = camera_profile.get_device();

            std::cout << "Connected camera: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

            if (capture_data->rs2_filter_preset->preset == rs2_filter_preset_undefined)
            {
                std::cerr << "ERROR: rs2_filter_preset undefined: advanced camera settings NOT loaded!" << std::endl;
            } 
            else
            {
                if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D415") == 0)
                {
                    json_file_name = "../camera_settings/D415_" + capture_data->rs2_filter_preset->rs2_config_file;
                    load_camera_settings = true;
                }
                else if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D455") == 0)
                {
                    json_file_name = "../camera_settings/D455_" + capture_data->rs2_filter_preset->rs2_config_file;
                    load_camera_settings = true;
                }
                else {
                    std::cerr << "ERROR: Unsupported Camera: advanced settings NOT loaded!" << std::endl;
                }
            }

            auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
            if (load_camera_settings) 
            {
                std::ifstream t(json_file_name);
                std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
                advanced_mode_dev.load_json(preset_json);
            }

            // Enable Laser
            auto depth_sensor = dev.first<rs2::depth_sensor>();
            if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
            {
                depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
            }
            if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
            {
                // Query min and max values:
                auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
                depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
            }

            rs2::frameset frames;
            
            for(int i = 0; i < 2 * capture_data->framerate; i++)
            {
                //Wait for all configured streams to produce a frame
                frames = pipe.wait_for_frames();
            }
        } 
        else
        {
            if ("" != capture_data->bag_filename)
            {
                cfg.enable_device_from_file(capture_data->bag_filename);
                pipe.start(cfg); 
            } else {
                std::cerr << "ERROR: no bag filename specified!" << std::endl;
            }
        }

        return pipe;
    }

    rs2::video_frame filterRS2depthImage(rs2::video_frame depth_frame, 
            rs2_filter_settings_t const *rs2_filter_preset)
    {
        assertm(rs2_filter_preset->preset != rs2_filter_preset_undefined, "RS2 filter preset was not initialized!");

        rs2::video_frame filtered = depth_frame; // Does not copy the frame, only adds a reference

        rs2::threshold_filter threshold_filter(rs2_filter_preset->limit_min, rs2_filter_preset->limit_max);
        filtered = threshold_filter.process(filtered);
                
        // Decalre filters - doc: https://dev.intelrealsense.com/docs/post-processing-filters
        if (rs2_filter_preset->dec_fil_enable) 
        {
            rs2::decimation_filter decimation_filter;  // Decimation - reduces depth frame density   
            decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, rs2_filter_preset->dec_fil_magnitude);
            filtered = decimation_filter.process(filtered);
        }
        
        rs2::disparity_transform depth_to_disparity(true);
        filtered = depth_to_disparity.process(filtered);
        
        if (rs2_filter_preset->spa_fil_enable)
        {
            rs2::spatial_filter spatial_filter;    // Spatial    - edge-preserving spatial smoothing
            spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, rs2_filter_preset->spa_fil_magnitude);
            spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, rs2_filter_preset->spa_fil_alpha);
            spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, rs2_filter_preset->spa_fil_delta);
            //(0=none, 1=2 pixels, 2=4 pixels, 3=8 pixels, 4=16 pixels, 5=unlimited).
            spatial_filter.set_option(RS2_OPTION_HOLES_FILL, rs2_filter_preset->spa_fil_holes_fill);  
            filtered = spatial_filter.process(filtered); 
        }

        if (rs2_filter_preset->tem_fil_enable)
        {
            rs2::temporal_filter temporal_filter(rs2_filter_preset->tem_fil_alpha,
                    rs2_filter_preset->tem_fil_delta,
                    rs2_filter_preset->tem_fil_pers_ctl);   
                    // 3rd argument: 3: valid in 2/last 4; Temporal   - reduces temporal noise
                    //temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.09);
                    //temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 24);
                    //temporal_filter.set_option(RS2_OPTION_MODE, 0.09);
            filtered = temporal_filter.process(filtered);
        }

        rs2::disparity_transform disparity_to_depth(false);
        filtered = disparity_to_depth.process(filtered); 

        //rs2::hole_filling_filter hole_filling_filter(2); // mode 1: farest from around, mode 2: nearest from around
        //filtered = hole_filling_filter.process(filtered);
    
        return filtered;
    }

    void convertRealSenceToPCL ( capture_data_t* capture_data ) 
    {
        // ------------------------------------------------------------------
        // Capture pc data from Realsense camera
        // ------------------------------------------------------------------

        // Declare pointcloud object, for calculating pointclouds and texture mappings
        rs2::pointcloud pc;
        // We want the points object to be persistent so we can display the last cloud when a frame drops
        rs2::points points_filtered; 
            
        rs2::frameset frames = capture_data->pipe.wait_for_frames();    // Wait for next set of frames from the camera
        
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        rs2::depth_frame filtered = ilpcr::filterRS2depthImage(depth_frame, 
                capture_data->rs2_filter_preset);

        // Generate the pointcloud and texture mappings
        points_filtered = pc.calculate(filtered);
            
        capture_data->realsense_pcl_filtered = points_to_pcl(points_filtered);
    }

    CloudTypeXYZ::Ptr points_to_pcl(const rs2::points& points)
    {
        CloudTypeXYZ::Ptr cloud(new CloudTypeXYZ);

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();

        #pragma omp parallel for
        for (auto& p : cloud->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            ptr++;
        }

        return cloud;
    }

    void set_r2s_filter_preset(capture_data_t* capture_data, rs2_filter_preset_names_t filter_preset)
    {
        capture_data->rs2_filter_preset = &rs2_filter_presets[filter_preset];
        assertm(capture_data->rs2_filter_preset->preset == filter_preset, "RS2 filter preset integrity check failed!");
    }
}

