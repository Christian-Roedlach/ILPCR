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
 * File:            07_iss_keypoints_from_file.cpp
 * Description:     static research scenario: extracting ISS keypoints 
 *                  and destcriptors
 */

#include <ilpcr/types.h>
#include <ilpcr/feature_estimation.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/file_io.h>
#include <pcl/keypoints/iss_3d.h>


using namespace ilpcr;

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
    char filename[255] = "../../files/coor_cap_rgb.pcd";
    double model_resolution = 0.11; 

    std::cout << "Arguments:" << std::endl 
            << "\tpointcloud <filename> (default: ../files/coor_cap_rgb.pcd)" << std::endl
            << "\textraction resolution <double> (default: 0.11)" << std::endl << std::endl;
    
    if (argc > 1) {
        strncpy(filename, argv[1], sizeof(filename) - 1);
    }

    if (argc > 2) {
        model_resolution = atof(argv[2]);
    }
        

    std::cout << "Input: " << filename << std::endl;

/*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (filename, *cloud_XYZRGB);
    //std::cout << "File1" << std::endl;
*/

    CloudTypeXYZ::Ptr cloud_XYZ (new CloudTypeXYZ);
    pcl::io::loadPCDFile (filename, *cloud_XYZ);
    //std::cout << "File2" << std::endl;

//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_XYZRGBA (new pcl::PointCloud<pcl::PointXYZRGBA>);
//    pcl::io::loadPCDFile (filename, *cloud_XYZRGBA);
//    //std::cout << "File1" << std::endl;

    /*
    if (argc > 2)
    	strncpy(filename, argv[2], sizeof(filename) - 1);
    else
        strncpy(filename, "../files/testoutput.pcd", sizeof("../files/testoutput.pcd"));

    std::cout << "Output: " << filename << std::endl;
    */

    //pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
//    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

    //pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    CloudTypeXYZ::Ptr cloud_downsampled = downsample(cloud_XYZ, model_resolution);
   
    // defines max radius - bigger results to more keypoints
    // double model_resolution = 0.11; 
    
    // Compute model_resolution
    
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (18 * model_resolution);
    iss_detector.setNonMaxRadius (4 * model_resolution);
    iss_detector.setNormalRadius (7 * model_resolution);
    iss_detector.setThreshold21 (0.975);
    iss_detector.setThreshold32 (0.975);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (16);
    iss_detector.setInputCloud (cloud_downsampled);
    iss_detector.compute (*model_keypoints);

    std::cout << "Detected keypoints:" << std::endl;

    std::cout << "\twidth = " << model_keypoints->width << ", height = " << model_keypoints->height << std::endl;
    std::cout << "\ttotal = " << model_keypoints->width * model_keypoints->height  << std::endl;

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.getRenderWindow()->GlobalWarningDisplayOff();
    viewer.setBackgroundColor (1, 1, 1);
    viewer.addCoordinateSystem (1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_XYZ_color_handler (cloud_downsampled, 0, 0, 0);
    viewer.addPointCloud (cloud_downsampled, cloud_XYZ_color_handler, "input cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");
    //viewer.addCoordinateSystem (1.0f, "global");
    //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
    //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (model_keypoints, 200, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (model_keypoints, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");

    viewer.initCameraParameters ();
    viewer.addCoordinateSystem (1.0);
    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }

    return 0;

}