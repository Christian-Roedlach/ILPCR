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
 * File:            file_io.cpp
 * Description:     functions for file and console input output 
 */

#include "file_io.h"
#if CUPOCH_ENABLED
#include "ilpcrCupoch.h"
#endif // CUPOCH_ENABLED
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <algorithm>
#include <memory>
#include <unistd.h>

using namespace std;

namespace ilpcr {
    bool store_tf_matrix_to_file(Eigen::Affine3f tf_matrix, std::string input_filename) {
        std::ofstream tf_matrix_file;
        bool retval = false;
        string output_file = pcl::getFilenameWithoutExtension(input_filename) + "_tfmatrix.Affine3f";

        tf_matrix_file.open(output_file, ios::out | ios::binary);
        if (tf_matrix_file.is_open()) {
            tf_matrix_file.write((char*)&tf_matrix, sizeof(tf_matrix));
            tf_matrix_file.close();

            std::cout << "Transformation Matrix written to " << output_file << std::endl;

            #if (DEBUG)
            cout << transformation.matrix() << endl;
            #endif // DEBUG

            retval = true;
        } else {
            cout << "Was not able to open file " << output_file << " for writing transformation matrix." << endl;
            retval = false;
        }

        return retval;
    }

    void print_transformation(Eigen::Matrix4f transformation) 
    {
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
    }

    void printAlignmentInfo(capture_data_t* capture_data, alignType_Narf * align)
    {
        Eigen::Affine3f transformation = capture_data->transformation;
        Eigen::Matrix4f transformation_matrix = transformation.matrix();
        Eigen::Matrix3d rotationMatrix = transformation.rotation().cast<double> ();
        Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0,1,2);

        print_transformation(transformation_matrix);
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Euler angles: psi = %6.3f deg, phi = %6.3f, theta = %6.3f\n", 
                                    pcl::rad2deg(eulerAngles(0)), pcl::rad2deg(eulerAngles(1)), pcl::rad2deg(eulerAngles(2)));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align->getInliers ().size (), align->getInputSource().get()->size());
        cout << "Inlier fraction: " << align->getInliers ().size () / (double) align->getInputSource().get()->size() * 100 << "%" << endl;
    }

    void printAlignmentInfo(capture_data_t* capture_data, alignType_Fpfh * align)
    {
        Eigen::Affine3f transformation = capture_data->transformation;
        Eigen::Matrix4f transformation_matrix = transformation.matrix();
        Eigen::Matrix3d rotationMatrix = transformation.rotation().cast<double> ();
        Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0,1,2);

        print_transformation(transformation_matrix);
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Euler angles: psi = %6.3f deg, phi = %6.3f, theta = %6.3f\n", 
                                    pcl::rad2deg(eulerAngles(0)), pcl::rad2deg(eulerAngles(1)), pcl::rad2deg(eulerAngles(2)));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align->getInliers ().size (), align->getInputSource().get()->size());
        cout << "Inlier fraction: " << align->getInliers ().size () / (double) align->getInputSource().get()->size() * 100 << "%" << endl;
    }

    int loadDestinationCloud(capture_data_t* capture_data, narf_data_t *narf_pcd)
    {
        string filename_XYZ = capture_data->filename + "_XYZ.pcd";
        string filename_narf36 = capture_data->filename + "_narf36.pcd";

        if (pcl::io::loadPCDFile (filename_XYZ, *(narf_pcd->dst_narf_keypoints_XYZ_ptr)) < 0 )
        {
            cerr << "Was not able to open file \"" << filename_XYZ << "\".\n";
            return EXIT_FAILURE;
        } 
        else
            cout << "Successfully read " << filename_XYZ << endl;

        if (pcl::io::loadPCDFile (filename_narf36, *(narf_pcd->dst_narf_descriptors_ptr)) < 0 )
        {
            cerr << "Was not able to open file \"" << filename_narf36 << "\".\n";
            return EXIT_FAILURE;
        } 
        else
        {
            cout << "Successfully read " << filename_narf36 << endl;
            cout << "Number of destination keypoints: " << narf_pcd->dst_narf_keypoints_XYZ_ptr->size() << endl;
        }
        
        return EXIT_SUCCESS;
    }

    int loadDestinationCloud(capture_data_t* capture_data, fpfh_data_t *fpfh_data) {
        assertm(fpfh_data->pcr_preset->preset != pcr_preset_undefined, "RANSAC sample consensus prerejective preset was not initialized!");

        string filename_PointNT = capture_data->filename + "_PointNormal.pcd";
        string filename_fpfh33 = capture_data->filename + "_fpfh33.pcd";

        if (pcl::io::loadPCDFile (filename_PointNT, *(fpfh_data->dst_fpfh_pcd_PointNT_ptr)) < 0 )
        {
            cerr << "Was not able to open file \"" << filename_PointNT << "\".\n";
            return EXIT_FAILURE;
        } 
        else
            cout << "Successfully read " << filename_PointNT << endl;

        if (pcl::io::loadPCDFile (filename_fpfh33, *(fpfh_data->dst_fpfh_descriptors_ptr)) < 0 )
        {
            cerr << "Was not able to open file \"" << filename_fpfh33 << "\".\n";
            return EXIT_FAILURE;
        } 
        else
        {
            cout << "Successfully read " << filename_fpfh33 << endl;
            cout << "Size of destination PCD: " << fpfh_data->dst_fpfh_pcd_PointNT_ptr->size() << endl;
        }

    #if CUPOCH_ENABLED
        if (ilpcr_proc_type_fgr_gpu == fpfh_data->pcr_preset->processing_unit ||
                ilpcr_proc_type_fpfh_fgr_gpu == fpfh_data->pcr_preset->processing_unit)
        {
            return loadDestinationCloud_cupoch(fpfh_data);
        }
        else
        {
            return EXIT_SUCCESS;
        }
    #else // CUPOCH_ENABLED
        return EXIT_SUCCESS;
    #endif // CUPOCH_ENABLED
    }

    int loadSourceCloud(capture_data_t* capture_data, fpfh_data_t *fpfh_data) {
        string filename_PointNT = capture_data->filename + "_PointNormal.pcd";
        string filename_fpfh33 = capture_data->filename + "_fpfh33.pcd";

        if (pcl::io::loadPCDFile (filename_PointNT, *(fpfh_data->src_fpfh_pcd_PointNT_ptr)) < 0 )
        {
            cerr << "Was not able to open file \"" << filename_PointNT << "\".\n";
            return EXIT_FAILURE;
        } 
        else
            cout << "Successfully read " << filename_PointNT << endl;

        if (pcl::io::loadPCDFile (filename_fpfh33, *(fpfh_data->src_fpfh_descriptors_ptr)) < 0 )
        {
            cerr << "Was not able to open file \"" << filename_fpfh33 << "\".\n";
            return EXIT_FAILURE;
        } 
        else
        {
            cout << "Successfully read " << filename_fpfh33 << endl;
            cout << "Size of source PCD: " << fpfh_data->src_fpfh_pcd_PointNT_ptr->size() << endl;
        }
        
        return EXIT_SUCCESS;
    }

    int parse_feature_type(int argc, char** argv, ilpcr_feature_t *feature_type)
    {
        std::string type;
        int retval = EXIT_FAILURE;

        if (pcl::console::parse (argc, argv, "-t", type) >= 0) {
            if (type == "narf" || type == "NARF") 
            {
                std::cout << "Feature Type: NARF Keypoints" << std::endl;
                *feature_type = ilpcr_narf;
                retval = EXIT_SUCCESS;
            } 
            else if ((type == "fpfh" || type == "FPFH"))
            {
                std::cout << "Feature Type: FPFH - Fast Point Feature Histogram" << std::endl;
                *feature_type = ilpcr_fpfh;
                retval = EXIT_SUCCESS;
            }
            else 
            {
                cerr << "ERROR: -t <Feature Type> unknown - available types: narf, fpfh" << endl;
                retval = EXIT_FAILURE;
            }
        } 
        else 
        {
            cerr << "ERROR: -t <Feature Type> required but not specified - available types: narf, fpfh" << endl;
            retval = EXIT_FAILURE;
        }

        return retval;
    }

    std::string filename_add_feature_desc_dir(std::string *filename)
    {
        std::string filename_desc_out_dir = *filename;
        size_t filename_slash_pos = filename_desc_out_dir.rfind("/");
        if (filename_slash_pos != std::string::npos) 
        {
            filename_desc_out_dir.insert(filename_slash_pos + 1, "feature_desc/");
        } 
        else
        {
            filename_desc_out_dir.insert(0, "feature_desc/");
        }

        return filename_desc_out_dir;
    }
    
    int parse_preset(std::string text, pcr_names_t *preset)
    {
        int retval = EXIT_FAILURE;

        for (size_t i = pcr_preset_first; i < pcr_names_count; i++)
        {
            if (text == pcr_presets[i].preset_name)
            {
                *preset = pcr_presets[i].preset;
                std::cout << "Sample consensus preset: " << text << std::endl;
                retval = EXIT_SUCCESS;
                
                break;
            }
        }

        if (EXIT_SUCCESS != retval)
            std::cerr << "ERROR: failed to parse sample consensus preset: " << text << std::endl;

        return retval;
    }

    int parse_preset(std::string text, fpfh_data_t *fpfh_data)
    {
        int retval = EXIT_FAILURE;

        for (size_t i = pcr_preset_first; i < pcr_names_count; i++)
        {
            if (text == pcr_presets[i].preset_name)
            {
                fpfh_data->pcr_preset = &pcr_presets[i];
                std::cout << "Sample consensus preset: " << fpfh_data->pcr_preset->preset_name << std::endl;
                retval = EXIT_SUCCESS;
                
                break;
            }
        }

        if (EXIT_SUCCESS != retval)
            std::cerr << "ERROR: failed to parse sample consensus preset: " << text << std::endl;

        return retval;
    }

    int parse_preset(std::string text, narf_data_t *narf_data_t)
    {
        int retval = EXIT_FAILURE;

        for (size_t i = pcr_preset_first; i < pcr_names_count; i++)
        {
            if (text == pcr_presets[i].preset_name)
            {
                narf_data_t->pcr_preset = &pcr_presets[i];
                std::cout << "Sample consensus preset: " << narf_data_t->pcr_preset->preset_name << std::endl;
                retval = EXIT_SUCCESS;
                
                break;
            }
        }

        if (EXIT_SUCCESS != retval)
            std::cerr << "ERROR: failed to parse sample consensus preset: " << text << std::endl;
        
        return retval;
    }

    int parse_preset(std::string text, const pcr_settings_t **preset)
    {
        int retval = EXIT_FAILURE;

        for (size_t i = pcr_preset_first; i < pcr_names_count; i++)
        {
            if (text == pcr_presets[i].preset_name)
            {
                *preset = &pcr_presets[i];
                std::cout << "Sample consensus preset: " << (*preset)->preset_name << std::endl;
                retval = EXIT_SUCCESS;
                
                break;
            }
        }

        if (EXIT_SUCCESS != retval)
            std::cerr << "ERROR: failed to parse sample consensus preset: " << text << std::endl;
        
        return retval;
    }

    int parse_preset(std::string text, capture_data_t *capture_data)
    {
        int retval = EXIT_FAILURE;

        for (size_t i = rs2_filter_preset_undefined; i < rs2_filter_preset_count; i++)
        {
            if (text == rs2_filter_presets[i].preset_name)
            {
                capture_data->rs2_filter_preset = &rs2_filter_presets[i];
                std::cout << "RS2 filter preset: " << capture_data->rs2_filter_preset->preset_name << std::endl;
                retval = EXIT_SUCCESS;
                
                break;
            }
        }

        if (EXIT_SUCCESS != retval)
            std::cerr << "ERROR: failed to parse RS2 filter preset: " << text << std::endl;
        
        return retval;
    }

    
    int pose_log_check_header(std::ifstream *input_file, std::string *filename)
    {
        using namespace std;

        int retval = EXIT_FAILURE;
        string line = "";
    
        input_file->open(*filename, ifstream::in | ifstream::binary);
    
        if (input_file->is_open())
        {
            // checking header
            getline(*input_file, line);
        
            if (POSE_LOG_HEADER_START == line)
            {
                retval = EXIT_SUCCESS;
            } 
            else
            {
                retval = EXIT_FAILURE;
                cerr << "HEADER declaration not found: " << POSE_LOG_HEADER_START << " required at first line of logfile!" << endl;
            }
        } else {
            cerr << "ERROR: failed to open file " << *filename << endl;
        }
    
        if (EXIT_SUCCESS == retval)
        {
            
            cout << "Filename: " << *filename << endl;
            cout << "Printing estimation logfile information: " << endl;
            cout << "-----------------------------------------------------------------------------------------------------------" << endl;
            
            while (input_file->good())
            {
                getline(*input_file, line);
    
                if (POSE_LOG_HEADER_END == line)
                {
                    retval = EXIT_SUCCESS;
                    break;
                }
    
                cout << line << endl; 
            }
    
            if (EXIT_FAILURE == retval)
            {
                cerr << "ERROR: header declaration not found: " << POSE_LOG_HEADER_END << " required at the end of the logfile header!" << endl;
            }
            else
            {
                cout << "-----------------------------------------------------------------------------------------------------------" << endl;
                cout << "Lofgile header was read successfully." << endl << endl;
            }
        }
    
        return retval;
    }

    int get_process_memory(uint64_t *vmSize, uint64_t *vmRSS)
    {
        int retval = EXIT_FAILURE;
        unsigned int pagesize = getpagesize();
        std::ifstream statm("/proc/self/statm");

        if (statm.is_open())
        {
            statm >> *vmSize >> *vmRSS;
            statm.close();

            /* resulting values are page sizes, but bytes are required */
            *vmSize *= pagesize;
            *vmRSS *= pagesize;

            retval = EXIT_SUCCESS;
        } else {
            std::cerr << "ERROR: opening file /proc/self/statm FAILED" << std::endl;
        }

        return retval;
    }

    /* ATTENTION: do not use in production - for debugging only - no ERROR handling */
    int parse_preset_config_file(std::string filename, pcr_settings_t *preset)
    {
        int retval = EXIT_FAILURE;

        if (NULL != preset)
        {
            preset->preset = pcr_names_custom;
            std::cout << "... parsing preset config file " << filename << std::endl;
            retval = EXIT_SUCCESS;
        }
        else
        {
            std::cerr << "ERROR: NULL pointer on: pcr_settings_t *preset" << std::endl;
            retval = EXIT_FAILURE;
        }
        
        if (EXIT_SUCCESS == retval)
        {
            std::ifstream config_file (filename);

            if (config_file.is_open())
            {
                std::string line;

                if(getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("preset_name" == name) {
                        preset->preset_name = value;
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: preset_name failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    }    
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("max_iterations" == name) {
                        preset->max_iterations = std::stoi(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: max_iterations failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    }    
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("nr_of_samples" == name) {
                        preset->nr_of_samples = std::stoi(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: nr_of_samples failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("corr_rand" == name) {
                        preset->corr_rand = std::stoi(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: corr_rand failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("sim_threshold" == name) {
                        preset->sim_threshold = std::stof(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: sim_threshold failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("max_corr_distance" == name) {
                        preset->max_corr_distance = std::stod(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: max_corr_distance failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("inlier_fraction" == name) {
                        preset->inlier_fraction = std::stof(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: inlier_fraction failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("raster_size" == name) {
                        preset->raster_size = std::stod(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: raster_size failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }

                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("feature_search_radius" == name) {
                        preset->feature_search_radius = std::stod(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: feature_search_radius failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }
                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("nest_search_radius" == name) {
                        preset->nest_search_radius = std::stod(value);
                        retval = EXIT_SUCCESS; 
                    } else {
                        std::cerr << "ERROR: nest_search_radius failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }
                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("processing_unit" == name) {
                        if ("ilpcr_proc_type_cpu" == value) {
                            preset->processing_unit = ilpcr_proc_type_cpu;
                            retval = EXIT_SUCCESS;
                        } else if ("ilpcr_proc_type_fgr_gpu" == value) {
                            preset->processing_unit = ilpcr_proc_type_fgr_gpu;
                            retval = EXIT_SUCCESS;
                        } else if ("ilpcr_proc_type_fpfh_fgr_gpu" == value) {
                            preset->processing_unit = ilpcr_proc_type_fpfh_fgr_gpu;
                            retval = EXIT_SUCCESS;
                        } else {
                            std::cerr << "ERROR: processing_unit (typename) failed!" << std::endl;
                            retval = EXIT_FAILURE;
                        }                        
                    } else {
                        std::cerr << "ERROR: processing_unit (line not found) failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }
                if(EXIT_SUCCESS == retval && getline(config_file, line)) {
                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);
                    if ("use_normal_surface_search" == name) {
                        if ("true" == value) {
                            preset->use_normal_surface_search = true;
                            retval = EXIT_SUCCESS;
                        } else if ("false" == value) {
                            preset->use_normal_surface_search = false;
                            retval = EXIT_SUCCESS;
                        } else {
                            std::cerr << "ERROR: use_normal_surface_search (true or false) failed!" << std::endl;
                            retval = EXIT_FAILURE;
                        }                        
                    } else {
                        std::cerr << "ERROR: use_normal_surface_search (line not found) failed!" << std::endl;
                        retval = EXIT_FAILURE;
                    } 
                }                                          

                if (EXIT_FAILURE == retval) {
                    std::cerr << "ERROR: parsing preset config file " << filename << " failed!" << std::endl;
                }
            }
            else {
                std::cerr << "ERROR: could not open file " << filename << " for reading.\n";
            }
        }

        return retval;
    }

    /* ATTENTION: do not use in production - for debugging only - no ERROR handling */
    int parse_preset_config_file(std::string filename, fpfh_data_t *fpfh_data)
    {
        int retval = EXIT_FAILURE;

        pcr_settings_t *preset (new pcr_settings_t);
        fpfh_data->pcr_preset = preset;

        retval = parse_preset_config_file(filename, preset);

        return retval;
    }

    /* ATTENTION: do not use in production - for debugging only - no ERROR handling */
    int parse_preset_config_file(std::string filename, lidar_data_t *settings)
    {
        int retval = EXIT_FAILURE;

        pcr_settings_t *preset (new pcr_settings_t);
        
        settings->pcr_preset_name = pcr_names_custom;
        retval = parse_preset_config_file(filename, preset);

        settings->pcr_preset = preset;
        
        return retval;
    }
}