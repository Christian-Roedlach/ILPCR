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
 * File:            camera.h
 * Description:     functions for capturing and filtering 
 *                  from Intel RealSense cameras
 */

#ifndef ILPCR_CAMERA_H
#define ILPCR_CAMERA_H

#include "types.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
namespace ilpcr
{
    rs2::pipeline initCamera(capture_data_t *capture_data);
    rs2::video_frame filterRS2depthImage(rs2::video_frame depth_frame, 
            rs2_filter_settings_t const *rs2_filter_preset);
    void convertRealSenceToPCL ( capture_data_t* capture_data );
    CloudTypeXYZ::Ptr points_to_pcl(const rs2::points& points);
    void set_r2s_filter_preset(capture_data_t* capture_data, rs2_filter_preset_names_t filter_preset);
}

#endif // ILPCR_CAMERA_H