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
 * File:            file_io.h
 * Description:     functions for file and console input output 
 */

#ifndef ILPCR_FILE_IO_H
#define ILPCR_FILE_IO_H

#define POSE_LOG_HEADER_START "--- HEADER BEGIN ---"
#define POSE_LOG_HEADER_END   "---- HEADER END ----"

#include "types.h"
#include <pcl/common/common_headers.h>
#include <fstream>

namespace ilpcr {
    bool store_tf_matrix_to_file(Eigen::Affine3f tf_matrix, std::string input_filename);
    void print_transformation(Eigen::Matrix4f transformation);
    void printAlignmentInfo(capture_data_t* capture_data, alignType_Narf * align);
    void printAlignmentInfo(capture_data_t* capture_data, alignType_Fpfh * align);
    int loadDestinationCloud(capture_data_t* capture_data, narf_data_t *narf_pcd);
    int loadDestinationCloud(capture_data_t* capture_data, fpfh_data_t *fpfh_data);
    int loadSourceCloud(capture_data_t* capture_data, fpfh_data_t *fpfh_data);
    int parse_feature_type(int argc, char** argv, ilpcr_feature_t *feature_type);
    std::string filename_add_feature_desc_dir(std::string *filename);
    int parse_preset(std::string text, pcr_names_t *preset);
    int parse_preset(std::string text, fpfh_data_t * fpfh_data);
    int parse_preset(std::string text, narf_data_t * narf_data_t);
    int parse_preset(std::string text, const pcr_settings_t **preset);
    int parse_preset(std::string text, capture_data_t *capture_data);
    int pose_log_check_header(std::ifstream *input_file, std::string *filename);
    int get_process_memory(uint64_t *vmSize, uint64_t *vmRSS);

    /* ATTENTION: do not use in production - for debugging only - no ERROR handling */
    int parse_preset_config_file(std::string filename, lidar_data_t *settings);
    int parse_preset_config_file(std::string filename, fpfh_data_t *fpfh_data);
    int parse_preset_config_file(std::string filename, pcr_settings_t *preset);
}

#endif // ILPCR_FILE_IO_H