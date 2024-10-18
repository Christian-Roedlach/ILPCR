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
 * File:            08_draw_position_history.cpp
 * Description:     static research scenario: Viszalization
 */

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> 

#include <ilpcr/types.h>
#include <ilpcr/viewer.h>
#include <ilpcr/file_io.h>
#include <ilpcr/corr_estimation.h>

#define DEBUG 0

#define COLOR_ARROW 0,0,255   // blue
#define COLOR_PATH  0,255,0   // green
#define COLOR_TEXT  0,0,0     // black

#define SCALE_TEXT_POS      0.1
#define SCALE_COORD_POINT   0.1

using namespace ilpcr;

void print_usage(void);
int parse_cmd_args(int argc, char** argv, dph_settings_t* settings);
int read_flist_files(dph_settings_t* settings);


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
        .scale_coord_point = SCALE_COORD_POINT
    };

    settings.display_settings = display_settings;
    
    retval = parse_cmd_args(argc, argv, &settings);
    if (EXIT_SUCCESS != retval)
        goto failure;

    retval = read_flist_files(&settings);
    if (EXIT_SUCCESS != retval)
        goto failure;

    retval = transform_pcd_data(&settings);
    if (EXIT_SUCCESS != retval)
        goto failure;

    retval = init_points_corr_pcd_colors(&settings);
    if (EXIT_SUCCESS != retval)
        goto failure;

    retval = display_3d_model(&settings);
    if (EXIT_SUCCESS != retval)
        goto failure;

    retval = draw_pos_and_path(&settings);
    if (EXIT_SUCCESS != retval)
        goto failure; 

    retval = EXIT_SUCCESS; // set return value to success if no error occured before

    // main loop for visualizer
    while (!settings.viewer->wasStopped ())
    {
        settings.viewer->spinOnce ();
        pcl_sleep(0.01);
    }

    return retval;

    failure:
    return retval;
}


void print_usage(void) {
    const std::string usage = {
"\nDisplay position history using following data: \
\n\t- Destination pointcloud (3D-Model) \
\n\t- Transformation matrix of every matching point to display \
\n\t- Pointcloud corresponding to the transformation matrix (optional) \
\n\nUsage: ./08_draw_position_history <fileList> [-p] [-fpfh|-narf] \
\n\t<fileList>    ... ASCII file containing source files to display - file extension must be .flist (details below) \
\n\t[-pcd]        ... Display corresponding transformed pointcloud (optional; if activated, .pcd data must be present) \
\n\t[-fpfh|-narf] ... Set feature type (NARF or FPFH) \
\n\n<fileList> data format (file extension: .flist): \
\n\t1st line: \
\n\t\tfileName.flist of 3D-Modell without path and including file extension \
\n\t\tpath must be ../ relative to <fileList>; file extention must be .pcd \
\n\t2nd to nth line: \
\n\t\tfile name of matching point transformation matrix without path, without _tfmatrix appendix and without file extension: \
\n\t\t    path must be ../feature_desc/ relative to <fileList>; \
\n\t\t    NARF features: file extention must be _tfmatrix.Affine3f \
\n\t\t    FPFH features: file extention must be _PointNormal_tfmatrix.Affine3f \
\n\t\tcorresponding PCD file (in case of [-p]): path must be ../ relative to <fileList>; file extention must be .pcd \
\n\n\tEXAMPLE: \
\n\t\t01 3dmodel.pcd \
\n\t\t02 point1 \
\n\t\t03 point2 \
\n\t\t04 point3 \
\n"
    };

    std::cout << usage;
}

int parse_cmd_args(int argc, char** argv, dph_settings_t* settings) 
{
    int retval = EXIT_FAILURE;
    

    std::vector<int> flist_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "flist");

    if (!flist_filename_indices.empty ())
    {
        settings->filename_flist = argv[flist_filename_indices[0]];
        std::cout << ".flist filename: " << settings->filename_flist << std::endl;
        retval = EXIT_SUCCESS;
    }
    else
    {
        std::cout << "ERROR: no .flist file specified!" << std::endl;
        print_usage();

        retval = EXIT_FAILURE;
    }

    if (pcl::console::find_argument (argc, argv, "-pcd") >= 0) {
        settings->displ_corr_pcd = true;
        std::cout << "\n --> Display corresponding .pcd enabled.\n";
    } else {
        settings->displ_corr_pcd = false;
        std::cout << "\n --> Display corresponding .pcd disabled.\n";
    }

    if (pcl::console::find_argument (argc, argv, "-legacy") >= 0) {
        settings->legacy = true;
        std::cout << "\n --> Entering LEGACY mode (no preset files)\n";
    }

    if (pcl::console::find_argument (argc, argv, "-fpfh") >= 0) {
        settings->feature_type = ilpcr_fpfh;
        std::cout << "\n --> Feature Type: FPFH\n\n";
    } else if (pcl::console::find_argument (argc, argv, "-narf") >= 0) {
        settings->feature_type = ilpcr_narf;
        std::cout << "\n --> Feature Type: NARF\n\n";
    } else {
        settings->feature_type = ilpcr_narf;
        std::cout << "\n --> Using default feature type: NARF\n\n";
    }

    /* parse sample consense prerejective preset */
    std::string text;
    if (pcl::console::parse (argc, argv, "-p", text) >= 0) {
        retval = parse_preset(text, &settings->pcr_preset);
    } else if (pcl::console::parse (argc, argv, "-pcfg", text) >= 0) {
        settings->customPresetCfg = text;
	}

    return retval;
}

int read_flist_files(dph_settings_t* settings)
{
    int retval = EXIT_FAILURE;
    std::ifstream flist (settings->filename_flist);
    std::string path = settings->filename_flist;
    path.erase(path.end() - pcl::getFilenameWithoutPath(path).length(), path.end()); // extract path from filename
    std::string fn_model_3d;
    //string model3d = 

    #if DEBUG
    std::cout << "path = " << path << std::endl;
    #endif // DEBUG
    
    if (!flist.is_open())
    {
        std::cout << "ERROR: Unable to open file: " << settings->filename_flist << std::endl;
        print_usage();
        retval = EXIT_FAILURE;
    }
    else
    {
        std::string tmp;
        // read 3d model (1st line)
        if (!getline (flist, tmp))
        {
            std::cout << "ERROR: " << settings->filename_flist << " is empty!" << std::endl;
        }
        else
        {
            fn_model_3d = path + "../" + tmp;
            CloudTypeXYZ::Ptr model_3d_pcd_xyz(new ( CloudTypeXYZ));
            settings->model_3d_pcd_xyz = model_3d_pcd_xyz;

            if (pcl::io::loadPCDFile (fn_model_3d, *settings->model_3d_pcd_xyz) == -1)
            {
                std::cerr << "ERROR: was not able to open 3D model file " << fn_model_3d << std::endl;
            }
            else
            {
                uint32_t count = 0;
                Eigen::Affine3f tf_matrix_tmp;
                std::string filename_tf_matrix;
                std::string filename_corr_pcd;
                narf_data_t narf_data;
                fpfh_data_t fpfh_data;

                if ("" != settings->customPresetCfg) {
                    settings->pcr_preset = pcr_names_custom;
                    retval = parse_preset_config_file(settings->customPresetCfg, &fpfh_data);

                    if (EXIT_SUCCESS != retval)
                        return retval;
                } else {
                    if (pcr_preset_undefined == settings->pcr_preset)
                        settings->pcr_preset = pcr_preset_fpfh_1;
                    
                    set_sample_cons_prerej_preset(&fpfh_data, settings->pcr_preset);
                }

                // load tf_matrices and pcd data from files reading line by line
                while (getline (flist, tmp))
                {
                    settings->points_names.push_back(tmp);
                    settings->point_identifier.push_back(count);

                    switch (settings->feature_type) {
                        case ilpcr_fpfh:                           
                            if (settings->legacy)
                                filename_tf_matrix = path + "../feature_desc/" + tmp + "_PointNormal_tfmatrix.Affine3f";
                            else     
                                filename_tf_matrix = path + "../feature_desc/" + tmp + "_PointNormal_" +
                                fpfh_data.pcr_preset->preset_name + 
                                "_tfmatrix.Affine3f";
                        
                            break;
                        case ilpcr_narf:
                            if (pcr_preset_undefined == settings->pcr_preset)
                                settings->pcr_preset = pcr_preset_narf_1;

                            set_sample_cons_prerej_preset(&narf_data, settings->pcr_preset);

                            if (settings->legacy)
                                filename_tf_matrix = path + "../feature_desc/" + tmp + "_tfmatrix.Affine3f";
                            else  
                                filename_tf_matrix = path + "../feature_desc/" + tmp + "_" +
                                narf_data.pcr_preset->preset_name +
                                "_tfmatrix.Affine3f";

                            break;
                        default:
                            filename_tf_matrix = path + "../feature_desc/" + tmp + "_tfmatrix.Affine3f";
                    }
                    
                    std::ifstream tf_matrix;
                    tf_matrix.open(filename_tf_matrix, std::ios::binary);
                    
                    if (tf_matrix.is_open())
                    {
                        tf_matrix.read((char*) &tf_matrix_tmp, sizeof(tf_matrix_tmp));
                        tf_matrix.close();
                        settings->points_tf_matrices.push_back(tf_matrix_tmp);
                        #if DEBUG
                        std::cout << "Transfer matrix " << count << ":\n" << tf_matrix_tmp.matrix() << std::endl;
                        #endif // DEBUG
                        retval = EXIT_SUCCESS;
                    } 
                    else 
                    {
                        std::cerr << "ERROR: was not able to open file " << filename_tf_matrix << std::endl;
                        retval = EXIT_FAILURE;
                        break;
                    }

                    // load corresponding pcd data if activated
                    if (settings->displ_corr_pcd)
                    {                       
                        filename_corr_pcd = path + "../" + tmp + ".pcd";
                        CloudTypeXYZ::Ptr corr_pcd_tmp(new ( CloudTypeXYZ));

                        if (pcl::io::loadPCDFile (filename_corr_pcd, *corr_pcd_tmp) == -1)
                        {
                            std::cerr << "ERROR: was not able to open file " << filename_corr_pcd << std::endl;
                            retval = EXIT_FAILURE;
                            break;
                        }
                        else{
                            settings->points_corr_pcd.push_back(corr_pcd_tmp);
                            #if DEBUG
                            std::cout << "Corresponding .pcd data " << count << " loaded." << std::endl;
                            #endif // DEBUG
                            retval = EXIT_SUCCESS;
                        }
                    }  

                    count ++;             
                }
                if (pcr_names_custom == fpfh_data.pcr_preset->preset) {
                    delete fpfh_data.pcr_preset;
                    fpfh_data.pcr_preset = nullptr;
                }
            }
        }      
        flist.close();
    }  

    return retval;
}




