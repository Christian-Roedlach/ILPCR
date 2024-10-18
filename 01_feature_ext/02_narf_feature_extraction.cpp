/*
 * ILPCR - Indoor Localization by Point Cloud Registration
 * Copyright (C) 2024 Christian Roedlach
 * Adapted from Bastian Steder
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
 * File:            02_narf_feature_extraction.cpp
 * Description:     static research scenario: extracting NARF keypoints 
 *                  and destcriptors from file
 */

#include <iostream>
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <string>

#define LOGFILE_NAME "../log/02_logfile.csv"

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/narf.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.3f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

// --------------
// -----Help-----
// --------------
void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
                                                                  "default "<<support_size<<")\n"
            << "-o <0/1>     switch rotational invariant version of the feature on/off"
            <<               " (default "<< (int)rotation_invariant<<")\n"
            << "-v           display 3D viewer\n"
            << "-min <float> range image minimum range for point cloud data\n"
            << "-noise <fl.> range image noise level\n"
            << "-boarder <i> range image boarder size\n"
            << "-h           this help\n"
            << "\n\n";
}


void 
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
    std::string filename = "09_realsense_filter";
    std::ofstream logfile;
    std::ifstream logfile_read;
    bool logfile_empty = false;
    timespec ts;
    char timeStamp[100];
    char hostname[100];

    auto timestamp_start = std::chrono::high_resolution_clock::now();
    auto timestamp_inter = std::chrono::high_resolution_clock::now();
    auto timestamp_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = 0;
    double total_time_ms = 0;

    bool start_viewer = false;

    // Range Image Parameters:

    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 0;  // formerly 1

    timespec_get(&ts, TIME_UTC);
    std::strftime(timeStamp, sizeof timeStamp, "%D %T", std::gmtime(&ts.tv_sec));
    
    logfile_read.open (LOGFILE_NAME);
    if (!logfile_read.is_open()) {
        /*std::getline(logfile_read,read_line);
        std::cout << read_line << read_line.length();
        if (read_line.length() == 0)*/
            logfile_empty = true;
    }
    logfile_read.close();

    logfile.open (LOGFILE_NAME, ios::out | ios::app);

    if (logfile_empty)
    {
        if (logfile.is_open())
            logfile << "hostname,executable,timestamp,cmd_args,PCD_filename,Range Image,NARF keyp. detection,detected keyp.,NARF keyp. extraction,extracted keyp.,Total comp. time,desc. filename,keypoints filename\n";
    }

        if (0 != gethostname(hostname, sizeof(hostname)))
          std::cerr << "Reading hostname FAILED!\n";

    if (logfile.is_open())
        /* hostname */
        logfile << hostname << ",";
    else
        std::cerr << "Opening logfile FAILED!\n";

    if (argc > 0)
    {
        if (logfile.is_open())
            /* executable */
            logfile << argv[0] << ",";
    }

    if (logfile.is_open()) {
        /* timestamp */
        logfile << timeStamp << '.' << ts.tv_nsec << " UTC,";
        /* cmd line args */
        for(int i = 1; i < argc; ++i)
            logfile << argv[i] << ' ';
        logfile << ",";
    }


    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
      printUsage (argv[0]);
      return 0;
    }
    if (pcl::console::find_argument (argc, argv, "-m") >= 0)
    {
      setUnseenToMaxRange = true;
      std::cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
      std::cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
    int tmp_coordinate_frame;
    if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
      coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
      std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
    }
    if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
      std::cout << "Setting support size to "<<support_size<<".\n";
    if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
      std::cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
    angular_resolution = pcl::deg2rad (angular_resolution);
    if (pcl::console::find_argument (argc, argv, "-v") >= 0) {
      start_viewer = true;
      std::cout << "Viewer enabled.\n";
    } else {
      start_viewer = false;
      std::cout << "Viewer disabled.\n";
    }
    if (pcl::console::parse (argc, argv, "-min", min_range) >= 0) {
        cout << "Setting range image minimum range to "<< min_range <<".\n";
    }
    if (pcl::console::parse (argc, argv, "-noise", noise_level) >= 0) {
        cout << "Setting range image noise level to "<< noise_level <<".\n";
    }
    if (pcl::console::parse (argc, argv, "-boarder", border_size) >= 0) {
        cout << "Setting range image boarder size to "<< border_size <<".\n";
    }
      
  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
      std::cerr << "Was not able to open file \""<<filename<<"\".\n";
      /* PCD_filename */
        logfile << "file not found" << ",";
      printUsage (argv[0]);
      return 0;
    } else {
      if (logfile.is_open())
          /* PCD_filename */
          logfile << filename << ",";
    }
    // save pcd file to binary
    // pcl::io::savePCDFileBinary("../files/binout.pcd", point_cloud);

    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                              point_cloud.sensor_origin_[1],
                                                              point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
        std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else
  {
    logfile << "no file specified";
    std::cout << "\nNo *.pcd file given.\n\n";
    return 0;
  }

  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------

  timestamp_start = std::chrono::high_resolution_clock::now();

  pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  
  timestamp_end = std::chrono::high_resolution_clock::now();
  elapsed_time_ms = std::chrono::duration<double, std::milli>(timestamp_end - timestamp_start).count();
  timestamp_inter = timestamp_end;
  
  if (logfile.is_open())
    /* Range Image */
    logfile << elapsed_time_ms << " ms,";
  
  /*
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  */
  
  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;

/*
  pcl::PointCloud<pcl::BorderDescription> range_image_border_descriptions;
  range_image_border_extractor.compute (range_image_border_descriptions);
  */
  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute (keypoint_indices);

  timestamp_end = std::chrono::high_resolution_clock::now();
  elapsed_time_ms = std::chrono::duration<double, std::milli>(timestamp_end - timestamp_inter).count();
  timestamp_inter = timestamp_end;
  if (logfile.is_open())
  {
    /* NARF key points detected */
    logfile << elapsed_time_ms << " ms,";
    /* No of detected key points */
    logfile << keypoint_indices.size() << " key points,";
  }

  std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";

  // ------------------------------------------------------
  // -----Extract NARF descriptors for interest points-----
  // ------------------------------------------------------
  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize (keypoint_indices.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices[i];
  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narf_descriptor.compute (narf_descriptors);

  timestamp_end = std::chrono::high_resolution_clock::now();
  elapsed_time_ms = std::chrono::duration<double, std::milli>(timestamp_end - timestamp_inter).count();
  total_time_ms = std::chrono::duration<double, std::milli>(timestamp_end - timestamp_start).count();

  if (logfile.is_open())
  {
    /* NARF key points extraction */
    logfile << elapsed_time_ms << " ms,";
    /* No of key points extracted */
    logfile << narf_descriptors.size() << " key points,";
    /* Total comp. time */
    logfile << total_time_ms << " ms,";
  }
  
  std::cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.size ()<< " keypoints.\n";
  std::string filename_desc_out = filename;
  size_t filename_slash_pos = filename_desc_out.rfind("/");

  if (filename_slash_pos != std::string::npos) 
  {
    filename_desc_out.insert(filename_slash_pos + 1, "feature_desc/");
  } 
  else
  {
    filename_desc_out.insert(0, "feature_desc/");
  }

  std::string narf36_filename = pcl::getFilenameWithoutExtension(filename_desc_out) + "_narf36.pcd";

  //std::cout << "filename_slash_pos = " << filename_slash_pos << "\n";
/*
  auto narf_class_object = new(pcl::Narf);
  narf_class_object->extractForEveryRangeImagePointAndAddToList(range_image, 36, support_size, rotation_invariant, )
  narf_descriptor
*/

  if (0 != pcl::io::savePCDFile (narf36_filename, narf_descriptors, true))
  {
    std::cout << "Error writing narf_descriptors to file!";
    if (logfile.is_open())
    {
      logfile << "descriptors storage FAILURE,";
    }
  } else
  {
    if (logfile.is_open())
    {
      logfile << narf36_filename << ",";
    }
    std::cout << "Descriptors output file: " << narf36_filename << "\n";
  }

  // Extract keypoints position (XYZ data)

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  std::size_t descr_size = narf_descriptors.size();
  keypoints.resize(descr_size);
  for (std::size_t i=0; i<descr_size; ++i)
  {
    keypoints[i].x = narf_descriptors.points[i].x;
    keypoints[i].y = narf_descriptors.points[i].y;
    keypoints[i].z = narf_descriptors.points[i].z;
  }

   // Save Keypoints to file
  std::string keypoints_filename = pcl::getFilenameWithoutExtension(filename_desc_out) + "_XYZ.pcd";
  
  if (0 != pcl::io::savePCDFile (keypoints_filename, keypoints, true))
  {
    std::cout << "Error writing keypoints to file!";
    if (logfile.is_open())
    {
      logfile << "keypoints storage FAILURE";
    }
  } else
  {
    if (logfile.is_open())
    {
      logfile << keypoints_filename;
    }
    std::cout << "Keypoints output file: " << keypoints_filename << "\n";
  }
  
  
  
  if (logfile.is_open())
      logfile << "\n";
  logfile.close();

  if (start_viewer)
  {
    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image);

/*
    //-------------------------------------
    // -----Show points on range image-----
    // ------------------------------------
    pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
    range_image_borders_widget =
        pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
                                                                              range_image_border_descriptions, "Range image with borders");
  // -------------------------------------
*/
    // ----------------------------------------------
    // -----Show keypoints in range image widget-----
    // ----------------------------------------------
    for (std::size_t i=0; i<keypoint_indices.size (); ++i)
        range_image_widget.markPoint ((size_t) (keypoint_indices[i]%range_image.width),
                                      (size_t) (keypoint_indices[i]/range_image.width),
                                      // Set the color of the pixel to red (the background
						                          // circle is already that color). All other parameters
						                          // are left untouched, check the API for more options.
						                          pcl::visualization::Vector3ub(1.0f, 0.0f, 0.0f));

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.getRenderWindow()->GlobalWarningDisplayOff(); 
    viewer.setBackgroundColor (1, 1, 1);
    viewer.addCoordinateSystem (1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
    viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "range image");
    //viewer.addCoordinateSystem (1.0f, "global");
    //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
    //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters ();
    setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 200, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");

    //--------------------
    // -----Viewer loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
      //range_image_widget.spinOnce (100,true);  // process GUI events
      viewer.spinOnce ();
      pcl_sleep(0.01);
    }
  }

  return EXIT_SUCCESS;
}