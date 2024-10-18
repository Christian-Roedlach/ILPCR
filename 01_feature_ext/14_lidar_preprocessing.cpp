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
 * File:            14_lidar_preprocessing.cpp
 * Description:     pre-processing of the 3D model \ac{PCD} using the 
 *                  ILPCR library
 */

#include <ilpcr/types.h>
#include <ilpcr/feature_estimation.h>
#include <ilpcr/filters.h>
#include <ilpcr/file_io.h>

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/file_io.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace ilpcr;

int parse_cmd_args(int argc, char** argv, 
                   lidar_data_t *settings);
int process_features(lidar_data_t *settings, narf_data_t *narf_data);
int process_features(lidar_data_t *settings, fpfh_data_t *fpfh_data);
void show_viewer(lidar_data_t *settings, narf_data_t *narf_data);
void show_viewer(lidar_data_t *settings, fpfh_data_t *fpfh_data);

const std::string usage = "Usage: ./14_lidar_preprocessing <input cloud [.pcd]> [-t <feature type>] [-s scale_factor] \n \
     <-p <samp_cons_preset> | -pcfg <filename>> \n \
     [-x rot_x_degrees] [-y rot_y_degrees] [-z rot_z_degrees] [-v ... show 3Dviewer]\n \
     [bf <cube length>] [bfw <width X >,<width Y>,<width Z> -bfc <center X>,<center Y>,<center Z>] \n\n \
        -t      <feature type>\n \
        -p      <sample consensus preset>\n \
        -pcfg   <sample consensus custom file preset name> (DEBUG only)\n \
        -s      <scale> scale size\n \
        -x      <rotation x-axis [deg]>\n \
        -y      <rotation y-axis [deg]>\n \
        -z      <rotation z-axis [deg]>\n \
        -v      show viewer\n \
        -bf     <cube length> cube remove filter (origin: 0,0,0) - after transformation\n \
        -bfw    <width X >,<width Y>,<width Z> cuboid remove filter width (requires -bfc) - after transformation\n \
        -bfc    <center X>,<center Y>,<center Z> cuboid remove filter center (requires -bfw) - after transformation\n";

// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
    int retval = EXIT_FAILURE;
    lidar_data_t settings;
    CloudTypeXYZ::Ptr cloud_in (new(CloudTypeXYZ));
    CloudTypeXYZ::Ptr cloud_transformed (new(CloudTypeXYZ));
    CloudTypeXYZ::Ptr cloud_filtered (new(CloudTypeXYZ));

    settings.cloud_in = cloud_in;
    settings.cloud_transformed = cloud_transformed;
    settings.cloud_filtered = cloud_filtered;
    settings.pcr_preset_name = pcr_preset_undefined;

    std::cout << usage << std::endl;

    // parse command line arguments
    retval = parse_cmd_args(argc, argv, &settings);

    if (EXIT_SUCCESS == retval)
    {
        // apply transformation
        pcl::transformPointCloud(*settings.cloud_in, *settings.cloud_transformed, settings.transformation);
        std::cout << "Transformation Matrix: "<< endl << settings.transformation.matrix() << endl;

        settings.cloud_filtered = settings.cloud_transformed;

        if (settings.cube_remove_filter)
        {
            Eigen::Vector3f offset(0,0,0);

            std::cout << "Applying cube remove filter: length = " << settings.cube_filter_length << std::endl;
            remove_cube_filter(settings.cloud_filtered, settings.cloud_filtered, settings.cube_filter_length, &offset);
        } 
        else if (settings.cuboid_remove_filter)
        {
            if (settings.cuboid_filter_center != NULL && settings.cuboid_filter_width != NULL)
            {
                std::cout << "Applying cuboid remove filter: " << std::endl <<
                        "center = " << settings.cuboid_filter_center << std::endl <<
                        "width =  " << settings.cuboid_filter_width << std::endl;

                remove_cuboid_filter(settings.cloud_filtered, settings.cloud_filtered, settings.cuboid_filter_center, settings.cuboid_filter_width);

                delete settings.cuboid_filter_center;
                delete settings.cuboid_filter_width;
            }
            else
            {
                std::cerr << "ERROR: applying cuboid remove filter failed: Vectors not set!" << std::endl;
            }
            
        }

       
        // apply voxel grid filter
        if (settings.pcr_preset->raster_size > 0.0)
        {
            std::cout << "Applying initial voxel grid filter: raster size = " << settings.pcr_preset->raster_size << std::endl;
            voxel_grid_filter(settings.cloud_filtered, settings.cloud_filtered, settings.pcr_preset->raster_size);
        }
        else
            std::cout << "Voxel grid filter DISABLED: raster size was set to " << settings.pcr_preset->raster_size << std::endl;
        
        // apply statistical outlier removal filter
        stat_removal_filter(settings.cloud_filtered, settings.cloud_filtered);

        std::string file_output_filtered = pcl::getFilenameWithoutExtension(settings.filename_in) +  "_tf_filtered.pcd";
        std::cout << "Saving transformed and filtered PCD to " << file_output_filtered << std::endl;

        retval = pcl::io::savePCDFileBinary (file_output_filtered, *settings.cloud_filtered);
        if (EXIT_SUCCESS != retval)
        {
            std::cerr << "ERROR writing PCD file " << file_output_filtered << std::endl;
        } 
        else 
        {
            narf_data_t narf_data;
            fpfh_data_t fpfh_data;
        
            switch (settings.feature_type)
            {
                case ilpcr_narf:
                    if (pcr_preset_undefined == settings.pcr_preset_name)
                        settings.pcr_preset_name = pcr_preset_narf_1;

                    retval = process_features(&settings, &narf_data);
                    if (EXIT_SUCCESS == retval && settings.viewer)
                        show_viewer(&settings, &narf_data);

                    break;

                case ilpcr_fpfh:
                    if (pcr_preset_undefined == settings.pcr_preset_name)
                        settings.pcr_preset_name = pcr_preset_fpfh_1;

                    retval = process_features(&settings, &fpfh_data);
                    if (EXIT_SUCCESS == retval && settings.viewer)
                        show_viewer(&settings, &fpfh_data);

                    break;

                default:
                    retval = EXIT_FAILURE;
                    std::cerr << "ERROR: feature extracrion type mismatch!" << std::endl;
            }
        }
    }
    else 
    {
        std::cerr << "ERROR: lidar preprocessing FAILED!" << std::endl;
    }
  
    return retval;
}

// --------------------------------------
// -----Parse Command Line Arguments-----
// --------------------------------------
int parse_cmd_args(int argc, char** argv, 
                   lidar_data_t *settings)
{
    int retval = EXIT_FAILURE;
    double rotate_x = 0.;
    double rotate_y = 0.;
    double rotate_z = 0.;
    float input_float[3] = {0};
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

    using namespace std;

    // parse filename
    if (!pcd_filename_indices.empty ())
    {
        settings->filename_in = argv[pcd_filename_indices[0]];
        cout << "Input filename: " << settings->filename_in << endl;
        settings->filename_desc_out = filename_add_feature_desc_dir(&settings->filename_in);
        retval = EXIT_SUCCESS;
    } else {
        cout << "no .pcd file specified" << endl;
        cout << usage;
        retval = EXIT_FAILURE;
    }

    // load pointcloud
    if (EXIT_SUCCESS == retval)
    {
        if (pcl::io::loadPCDFile (settings->filename_in, *settings->cloud_in) == -1)
        {
            cerr << "Was not able to open file \"" << settings->filename_in << "\".\n";
            retval = EXIT_FAILURE;
        } else {
            retval = EXIT_SUCCESS;
        }
    }

    // parse feature type
    if (EXIT_SUCCESS == retval)
        retval = parse_feature_type(argc, argv, &settings->feature_type);

    if (EXIT_SUCCESS == retval)
    {
        /* parse sample consense prerejective preset */
        std::string text;
        if (pcl::console::parse (argc, argv, "-p", text) >= 0) {
            retval = parse_preset(text, &settings->pcr_preset_name);
            retval = parse_preset(text, &(settings->pcr_preset));
        }
        else if (pcl::console::parse (argc, argv, "-pcfg", text) >= 0) 
        {
	    	retval = parse_preset_config_file(text, settings);
	    	if (EXIT_SUCCESS == retval)
	    	{
	    		std::cout << "Using custom preset " << settings->pcr_preset->preset_name << " from file " << text << std::endl;
	    	} else {
	    		std::cerr << "ERROR: parsing custom preset from file " << text << " failed! " << std::endl;
	    		retval == EXIT_FAILURE;
	    	}
	    }
        else
        {
            std::cerr << "ERROR: setting preset OR preset config file required " << std::endl;
            std::cerr << "       <-p <samp_cons_preset> | -pcfg <filename>>" << std::endl;
            retval = EXIT_FAILURE;
        }
    }

    // parse viewer argument
    if (EXIT_SUCCESS == retval && pcl::console::find_switch(argc, argv, "-v"))
    {
        settings->viewer = true;
        cout << "Point cloud viewer ENABLED" << endl;  
    } 
    else if (EXIT_SUCCESS == retval)
    {
        cout << "Point cloud viewer DISABLED" << endl;  
        settings->viewer = false;
    }

    // parse scale argument
    if (EXIT_SUCCESS == retval &&  pcl::console::parse (argc, argv, "-s", settings->dst_scale) >= 0) {
        cout << "Setting destination cloud scale to "<< settings->dst_scale <<".\n";
        settings->transformation.scale(settings->dst_scale);
    }

    // parse rotation around x-axis
    if (EXIT_SUCCESS == retval && pcl::console::parse (argc, argv, "-x", rotate_x) >= 0) {
        cout << "Setting rotation around X-Axis to "<< rotate_x <<" degree.\n";
        rotate_x = pcl::deg2rad (rotate_x);
        settings->transformation.rotate(Eigen::AngleAxisf (rotate_x, Eigen::Vector3f::UnitX()));
    }

    // parse rotation around y-axis
    if (EXIT_SUCCESS == retval && pcl::console::parse (argc, argv, "-y", rotate_y) >= 0) {
        cout << "Setting rotation around Y-Axis to "<< rotate_y <<" degree.\n";
        rotate_y = pcl::deg2rad (rotate_y);
        settings->transformation.rotate(Eigen::AngleAxisf (rotate_y, Eigen::Vector3f::UnitY()));
    }

    // parse rotation around z-axis
    if (EXIT_SUCCESS == retval && pcl::console::parse (argc, argv, "-z", rotate_z) >= 0) {
        cout << "Setting rotation around Y-Axis to "<< rotate_z <<" degree.\n";
        rotate_z = pcl::deg2rad (rotate_z);
        settings->transformation.rotate(Eigen::AngleAxisf (rotate_z, Eigen::Vector3f::UnitY()));
    }

    // parse cube removing filter
    if (EXIT_SUCCESS == retval && pcl::console::parse (argc, argv, "-bf", input_float[0]) >= 0) {
        if (input_float[0] <= 0) {
            cerr << "ERROR: cube filter length has to be > 0 - cube filter DISABLED! Data received: " << input_float[0] << endl;
            settings->cube_remove_filter = false;
        } else {
            settings->cube_remove_filter = true;
            cout << "Cube remove filter ENABLED. Length = "<< input_float[0] <<" m.\n";
            settings->cube_filter_length = input_float[0];
        }
    }

    // parse cuboid removing filter
    if (EXIT_SUCCESS == retval &&
            pcl::console::parse_3x_arguments (argc, argv, "-bfw", input_float[0], input_float[1], input_float[2]) >= 0) 
    {
        settings->cuboid_remove_filter = true;

        for (size_t i = 0; i < 3; i++)
        {
            if (input_float[i] <= 0) {
                cerr << "ERROR: cube width has to be > 0 - cuboid filter DISABLED! Data received: " << input_float[0] << endl;
                settings->cuboid_remove_filter = false;
            }
        } 

        if (true == settings->cuboid_remove_filter) {
            settings->cuboid_filter_width = new Eigen::Vector3f(input_float[0], input_float[1], input_float[2]);
        }

        if (true == settings->cuboid_remove_filter &&
            pcl::console::parse_3x_arguments (argc, argv, "-bfc", input_float[0], input_float[1], input_float[2]) >= 0) 
        {
            settings->cuboid_filter_center = new Eigen::Vector3f(input_float[0], input_float[1], input_float[2]);
            cout << "Cuboid remove filter ENABLED.";
            settings->cuboid_remove_filter = true;
        }
        else 
        {
            cerr << "ERROR: input failure -> cuboid filter DISABLED! " << endl;
            cerr << "Usage: -bfw <width X >,<width Y>,<width Z> -bfc <center X>,<center Y>,<center Z>" << endl;
            cerr << "       all width values >= 0" << endl;
            settings->cuboid_remove_filter = false;
        }
    }

    return retval;
}


int process_features(lidar_data_t *settings, narf_data_t *narf_data)
{
    int retval = EXIT_FAILURE;
    std::string output_file_1;
    std::string output_file_2;
    std::string filname_out_preset;

    init_pcd(narf_data, settings->pcr_preset_name);

    // handle custom preset
    if (settings->pcr_preset_name == pcr_names_custom)
    {
        narf_data->pcr_preset = settings->pcr_preset;
    }

    ilpcr::feature_estimation(settings, narf_data);

    filname_out_preset = pcl::getFilenameWithoutExtension(settings->filename_desc_out) + 
			"_" + 
            narf_data->pcr_preset->preset_name;

    output_file_1 = filname_out_preset +  "_XYZ.pcd";
    std::cout << "Output Keypoint PCD:     " << output_file_1 << std::endl;
    output_file_2 = filname_out_preset + "_narf36.pcd";
    std::cout << "Output NARF descriptors: " << output_file_2 << std::endl;

    retval = pcl::io::savePCDFileBinary (output_file_1, *narf_data->dst_narf_keypoints_XYZ_ptr);
    if (EXIT_SUCCESS != retval)
    {
        std::cerr << "ERROR writing PCD file " << output_file_1 << std::endl;
        return EXIT_FAILURE;
    }

    retval = pcl::io::savePCDFileBinary (output_file_2, *narf_data->dst_narf_descriptors_ptr);
    if (EXIT_SUCCESS != retval)
    {
        std::cerr << "ERROR writing PCD file " << output_file_2 << std::endl;
        return EXIT_FAILURE;
    }

    return retval;
}

int process_features(lidar_data_t *settings, fpfh_data_t *fpfh_data)
{
    int retval = EXIT_FAILURE;
    std::string output_file_1;
    std::string output_file_2;
    std::string filname_out_preset;

    init_pcd(fpfh_data, settings->pcr_preset_name);

    // handle custom preset
    if (settings->pcr_preset_name == pcr_names_custom)
    {
        fpfh_data->pcr_preset = settings->pcr_preset;
    }

    feature_estimation(settings, fpfh_data);

    filname_out_preset = pcl::getFilenameWithoutExtension(settings->filename_desc_out) + 
            "_" + 
            fpfh_data->pcr_preset->preset_name;

    output_file_1 = filname_out_preset + "_PointNormal.pcd";
    std::cout << "Output PointNormal PCD:    " << output_file_1 << std::endl;
    output_file_2 = filname_out_preset + "_fpfh33.pcd";
    std::cout << "Output FPFH33 descriptors: " << output_file_2 << std::endl;

    retval = pcl::io::savePCDFileBinary (output_file_1, *fpfh_data->dst_fpfh_pcd_PointNT_ptr);
    if (EXIT_SUCCESS != retval)
    {
        std::cerr << "ERROR writing PCD file " << output_file_1 << std::endl;
        return EXIT_FAILURE;
    }

    retval = pcl::io::savePCDFileBinary (output_file_2, *fpfh_data->dst_fpfh_descriptors_ptr);
    if (EXIT_SUCCESS != retval)
    {
        std::cerr << "ERROR writing PCD file " << output_file_2 << std::endl;
        return EXIT_FAILURE;
    }

    return retval;
}

// -------------------------
// -----Open 3D viewer -----
// -------------------------
void show_viewer(lidar_data_t *settings, narf_data_t *narf_data) 
{
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.getRenderWindow()->GlobalWarningDisplayOff(); 
    viewer.setBackgroundColor (1, 1, 1);
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();

    // ---------------------------------------
    // -----Show point cloud in 3D viewer-----
    // ---------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> filtered_cloud_color_handler (settings->cloud_filtered, 0, 0, 0);
    viewer.addPointCloud<PointTypeXYZ> (settings->cloud_filtered, filtered_cloud_color_handler, "filtered point cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered point cloud");
        
    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> keypoints_color_handler (narf_data->dst_narf_keypoints_XYZ_ptr, 0, 255, 0);
    viewer.addPointCloud<PointTypeXYZ> (narf_data->dst_narf_keypoints_XYZ_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    //--------------------
    // -----Viewer loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
      pcl_sleep(0.01);
    }
}

// -------------------------
// -----Open 3D viewer -----
// -------------------------
void show_viewer(lidar_data_t *settings, fpfh_data_t *fpfh_data) 
{
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.getRenderWindow()->GlobalWarningDisplayOff(); 
    viewer.setBackgroundColor (1, 1, 1);
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();

    // ---------------------------------------
    // -----Show point cloud in 3D viewer-----
    // ---------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> filtered_cloud_color_handler (settings->cloud_filtered, 0, 0, 0);
    viewer.addPointCloud<PointTypeXYZ> (settings->cloud_filtered, filtered_cloud_color_handler, "filtered point cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered point cloud");
        
    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<PointTypePointNT> keypoints_color_handler (fpfh_data->dst_fpfh_pcd_PointNT_ptr, 0, 255, 0);
    viewer.addPointCloud<PointTypePointNT> (fpfh_data->dst_fpfh_pcd_PointNT_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");

    //--------------------
    // -----Viewer loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
      pcl_sleep(0.01);
    }
}

    