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
 * File:            viewer.h
 * Description:     functions for PCD and pose visualization
 */

#ifndef ILPCR_VIEWER_H
#define ILPCR_VIEWER_H

#include "types.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>
#include <netinet/in.h> 

#define COLOR_ARROW_DEFAULT 0,0,255   // blue
#define COLOR_PATH_DEFAULT  0,255,0   // green
#define COLOR_TEXT_DEFAULT  0,0,0     // black
#define SCALE_TEXT_POS_DEFAULT 0.1
#define SCALE_COORD_POINT_DEFAULT 0.1
#define KEEP_LAST_NO_OF_ITEMS 10

namespace ilpcr
{
    typedef struct {
        double r;
        double g;
        double b;
    } color_RGB_t;

    typedef struct {
        color_RGB_t color_arrow = {COLOR_ARROW_DEFAULT};
        color_RGB_t color_path = {COLOR_PATH_DEFAULT};
        color_RGB_t color_text = {COLOR_TEXT_DEFAULT};
        double scale_text_pos = SCALE_TEXT_POS_DEFAULT;
        double scale_coord_point = SCALE_COORD_POINT_DEFAULT;
        double scale_orientation = SCALE_COORD_POINT_DEFAULT * 3;
        bool display_pos_numbers = true;
        bool display_orientation_history = true;
    } dph_display_settings_t;

    typedef struct {
        bool displ_corr_pcd = false;
        std::string filename_flist;
        std::string filename_3d_model;
        CloudTypeXYZ::Ptr model_3d_pcd_xyz;
        std::vector<std::string> points_names;
        std::vector<Eigen::Affine3f> points_tf_matrices;
        std::vector<uint32_t> point_identifier;
        size_t points_already_drawn = 0;
        std::vector<CloudTypeXYZ::Ptr> points_corr_pcd;
        CloudTypeXYZ::Ptr positions;
        pcl::visualization::PCLVisualizer::Ptr viewer;
        std::vector<color_RGB_t> points_corr_pcd_colors;
        ilpcr_feature_t feature_type = ilpcr_narf;
        dph_display_settings_t display_settings;
        std::mutex mutex;
        sockaddr_in serverAddress;
        pcr_names_t pcr_preset = pcr_preset_undefined;
        bool legacy = false;
        std::string customPresetCfg;
    } dph_settings_t;

    /* dph stands for draw position history */
    int transform_pcd_data(dph_settings_t* settings);
    int display_3d_model(dph_settings_t* settings);
    int draw_pos_and_path(dph_settings_t* settings);
    int init_points_corr_pcd_colors(dph_settings_t* settings);
}

#endif // ILPCR_VIEWER_H