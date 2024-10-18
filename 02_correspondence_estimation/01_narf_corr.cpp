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
 * File:            06_narf_feature_extraction_and_RANSAC.cpp
 * Description:     static research scenario: NARF feature extraction 
 *                  and P-RANSAC pose estimation (using NARF descriptors)
 */

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/file_io.h> 
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>
#include <Eigen/Geometry> 
#include <fstream>

#define DEBUG   0

typedef pcl::PointXYZ PointTypeXYZ;
typedef pcl::Narf36 PointTypeNarf36;
typedef pcl::PointCloud<PointTypeXYZ> CloudTypeXYZ;
typedef pcl::PointCloud<PointTypeNarf36> CloudTypeNarf36;

using namespace std;

// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
    string filename_src = "../../01_feature_ext/files/feature_desc/narf_desc.pcd";
    string filename_dst = "../../01_feature_ext/files/feature_desc/narf_desc.pcd";
    CloudTypeNarf36::Ptr narf_descriptors_src_ptr(new ( CloudTypeNarf36));
    CloudTypeNarf36::Ptr narf_descriptors_dst_ptr(new ( CloudTypeNarf36));
    CloudTypeXYZ::Ptr narf_xyz_src_ptr(new ( CloudTypeXYZ));
    CloudTypeXYZ::Ptr narf_xyz_dst_ptr(new ( CloudTypeXYZ));
    CloudTypeXYZ::Ptr cloud_src_ptr(new ( CloudTypeXYZ));
    CloudTypeXYZ::Ptr cloud_dst_ptr(new ( CloudTypeXYZ));
    CloudTypeXYZ::Ptr point_cloud_src_aligned_ptr (new CloudTypeXYZ);
    CloudTypeXYZ::Ptr draw_shapes (new CloudTypeXYZ);
    int retval = EXIT_FAILURE;
    bool start_viewer = false;

    //narf_descriptors_src_ptr = shared_ptr

    //CloudTypeNarf36 narf_descriptors_src;

    cout << "Usage: ./01_narf_corr <src cloud [.pcd]> <dest cloud [.pcd]> [-v]\n";

    vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

    if (!pcd_filename_indices.empty ())
    {
        filename_src = argv[pcd_filename_indices[0]];
        if (pcd_filename_indices.size() >= 2)
            filename_dst = argv[pcd_filename_indices[1]];
    }
    cout << "Source:      " << filename_src << 
          "\nDestination: " << filename_dst << "\n";

    if (pcl::console::find_argument (argc, argv, "-v") >= 0) {
        start_viewer = true;
        std::cout << "\n --> Viewer enabled.\n\n";
    } else {
        start_viewer = false;
        std::cout << "\n --> Viewer disabled.\n\n";
    }

    string folder_descr = "../../files/feature_desc/";
    string folder_cloud = "../../files/";

    filename_src = pcl::getFilenameWithoutPath(filename_src);
    filename_dst = pcl::getFilenameWithoutPath(filename_dst);
    
    string filename_src_narf36 = folder_descr + pcl::getFilenameWithoutExtension(filename_src) + "_narf36.pcd";
    string filename_src_xyz = folder_descr + pcl::getFilenameWithoutExtension(filename_src) + "_XYZ.pcd";
    string filename_dst_narf36 = folder_descr + pcl::getFilenameWithoutExtension(filename_dst) + "_narf36.pcd";
    string filename_dst_xyz = folder_descr + pcl::getFilenameWithoutExtension(filename_dst) + "_XYZ.pcd";
    string filename_src_cloud = folder_cloud + filename_src;
    string filename_dst_cloud = folder_cloud + filename_dst;
    string filenema_tf_matrix = folder_descr + pcl::getFilenameWithoutExtension(filename_src) + "_tfmatrix.Affine3f";
    

    if (pcl::io::loadPCDFile (filename_src_narf36, *narf_descriptors_src_ptr) == -1)
    {
        cerr << "Was not able to open file \"" << filename_src_narf36 << "\".\n";
        return -1;
    } 

    if (pcl::io::loadPCDFile (filename_dst_narf36, *narf_descriptors_dst_ptr) == -1)
    {
        cerr << "Was not able to open file \"" << filename_dst_narf36 << "\".\n";
        return -1;
    } 

    if (pcl::io::loadPCDFile (filename_src_xyz, *narf_xyz_src_ptr) == -1)
    {
        cerr << "Was not able to open file \"" << filename_src_xyz << "\".\n";
        return -1;
    } 

    if (pcl::io::loadPCDFile (filename_dst_xyz, *narf_xyz_dst_ptr) == -1)
    {
        cerr << "Was not able to open file \"" << filename_dst_xyz << "\".\n";
        return -1;
    } 

    if (pcl::io::loadPCDFile (filename_src_cloud, *cloud_src_ptr) == -1)
    {
        cerr << "Was not able to open file \"" << filename_src_cloud << "\".\n";
        return -1;
    } 

    if (pcl::io::loadPCDFile (filename_dst_cloud, *cloud_dst_ptr) == -1)
    {
        cerr << "Was not able to open file \"" << filename_dst_cloud << "\".\n";
        return -1;
    } 



    cout << "Number of source keypoints:      " << narf_xyz_src_ptr->size() << endl;
    cout << "Number of destination keypoints: " << narf_xyz_dst_ptr->size() << endl;

    pcl::SampleConsensusPrerejective<PointTypeXYZ, PointTypeXYZ, PointTypeNarf36> align;
    align.setInputSource (narf_xyz_src_ptr);
    align.setSourceFeatures (narf_descriptors_src_ptr);
    align.setInputTarget (narf_xyz_dst_ptr);
    align.setTargetFeatures(narf_descriptors_dst_ptr);
    align.setMaximumIterations (1000000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (20); // Number of nearest features to use
    align.setSimilarityThreshold (0.85f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (0.20); // Inlier threshold
    align.setInlierFraction (0.85f); // Required inlier fraction for accepting a pose hypothesis

    {
        pcl::ScopeTime t("Alignment");
        align.align (*point_cloud_src_aligned_ptr);
        pcl::console::print_info ("  ... ", t.getTime());
    }

    cout << "Fitness score:   " << align.getFitnessScore() << endl;

    if (align.hasConverged ())
    {
        #define ROTMATRIX_DOUBLE 1

        #if (1 == ROTMATRIX_DOUBLE)
            #define CAST .cast<double> ()
            typedef Eigen::Matrix3d rotm_matrix;
            typedef Eigen::Vector3d rotm_vector;
            typedef Eigen::Quaterniond rotm_quaterion;
            cout << "rotation matrix is <double> precicion !" << endl;
        #else // (ROTMATRIX_DOUBLE)
            #define CAST
            typedef Eigen::Matrix3f rotm_matrix;
            typedef Eigen::Vector3f rotm_vector;
            typedef Eigen::Quaternionf rotm_quaterion;
        #endif // (ROTMATRIX_DOUBLE)

        // Print results
        printf ("\n");
        //Eigen::Matrix4f transformation = align.getFinalTransformation ();
        Eigen::Affine3f transformation;
        transformation.matrix() = align.getFinalTransformation ();
        rotm_matrix rotationMatrix = transformation.rotation() CAST;
        rotm_vector eulerAngles = rotationMatrix.eulerAngles(0,1,2);

        ofstream transformation_file;
        transformation_file.open(filenema_tf_matrix, ios::out | ios::binary);
        if (transformation_file.is_open()) {
            transformation_file.write((char*)&transformation, sizeof(transformation));
            transformation_file.close();
            #if (DEBUG)
            cout << transformation.matrix() << endl;
            #endif // DEBUG
        } else {
            cout << "Was not able to open file " << filenema_tf_matrix << " for writing transformation matrix." << endl;
        }

        #if (DEBUG)
        ifstream test;
        test.open(filenema_tf_matrix, ios::binary);
        if (test.is_open())
        {
            Eigen::Affine3f testmatrix;
            test.read((char*) &testmatrix, sizeof(testmatrix));
            test.close();
            cout << testmatrix.matrix() << endl;
        } else {
            cout << "Was not able to open file " << filenema_tf_matrix << " for reading transformation matrix." << endl;
        }
        #endif // DEBUG

        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Euler angles: alpha = %6.3f deg, beta = %6.3f, gamma = %6.3f\n", 
                                    pcl::rad2deg(eulerAngles(0)), pcl::rad2deg(eulerAngles(1)), pcl::rad2deg(eulerAngles(2)));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("transformation <x, y, z> = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), narf_descriptors_src_ptr->size ());
        cout << "Inlier fraction: " << align.getInliers ().size () / (double) narf_descriptors_src_ptr->size ()  * 100 << "%" << endl;

        rotm_quaterion quaternion(rotationMatrix);
        cout << endl << "quaternion  " << "x: " << quaternion.x() << ", y: " << quaternion.y()<< ", z: "
                 << quaternion.z() << ", w: " << quaternion.w() << endl;

        rotm_matrix rotationMatrixN = rotationMatrix;
        rotationMatrixN.normalize();
        cout << "Rotation Martix normalizized: \n" << rotationMatrixN << endl;
        rotm_quaterion quaternionN(rotationMatrixN);
        cout << "quaternionN " << "x: " << quaternionN.x() << ", y: " << quaternionN.y()<< ", z: "
                 << quaternionN.z() << ", w: " << quaternionN.w() << endl;

        float yaw, pitch, roll;
        float yaw2, pitch2, roll2;

        yaw = atan(rotationMatrix(1,0)/rotationMatrix(0,0));
        pitch = atan(-rotationMatrix(2,0)/sqrt(pow(rotationMatrix(2,1),2)+pow(rotationMatrix(2,2),2)));
        roll = atan(rotationMatrix(2,1)/rotationMatrix(2,2));

        yaw2 = atan2(rotationMatrix(2,0),rotationMatrix(2,1));
        pitch2 = acos(rotationMatrix(2,2));
        roll2 = -atan2(rotationMatrix(0,2),rotationMatrix(1,2));

        pcl::console::print_info ("Rotation v2: Roll(x) = %6.3f deg, Pitch(y) = %6.3f, Yaw(z) = %6.3f\n", 
                                    pcl::rad2deg(roll), pcl::rad2deg(pitch), pcl::rad2deg(yaw));
        pcl::console::print_info ("Rotation v3: Roll(x) = %6.3f deg, Pitch(y) = %6.3f, Yaw(z) = %6.3f\n", 
                                    pcl::rad2deg(roll2), pcl::rad2deg(pitch2), pcl::rad2deg(yaw2));

        pcl::transformPointCloud(*cloud_src_ptr, *cloud_src_ptr, transformation);
        pcl::PointCloud<PointTypeXYZ>::Ptr inliers (new pcl::PointCloud<PointTypeXYZ>);
        pcl::copyPointCloud(*point_cloud_src_aligned_ptr, align.getInliers(), *inliers);
        retval = EXIT_SUCCESS;
        
        if (start_viewer)
        {
            PointTypeXYZ point;
            point.x = 0; point.y = 0; point.z = 0;
            draw_shapes->push_back(point);
            point.x = transformation (0,3); point.y = transformation (1,3); point.z = transformation (2,3);
            draw_shapes->push_back(point);

            // Show alignment
            pcl::visualization::PCLVisualizer visu("Alignment");
            visu.getRenderWindow()->GlobalWarningDisplayOff(); 
            visu.setBackgroundColor (1, 1, 1);
            visu.addCoordinateSystem (1.0);
            visu.addCoordinateSystem(0.2, transformation, "RealSenseD415");
            visu.addPointCloud<PointTypeXYZ> (cloud_dst_ptr, pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (cloud_dst_ptr, 0, 0, 0), "destination");
            visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "destination");
            visu.addPointCloud<PointTypeXYZ> (cloud_src_ptr, pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (cloud_src_ptr, 0, 255, 0), "source_alligned");
            visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_alligned");
            visu.addPointCloud<PointTypeXYZ> (inliers, pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (inliers, 0, 100, 0), "inliers");
            visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "inliers");
            visu.addPointCloud<PointTypeXYZ> (narf_xyz_dst_ptr, pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (narf_xyz_dst_ptr, 0, 0, 255), "destination_keypoints");
            visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "destination_keypoints");
            visu.addPointCloud<PointTypeXYZ> (narf_xyz_src_ptr, pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (narf_xyz_src_ptr, 255, 0, 0), "source_keypoints");
            visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source_keypoints");
            visu.addPointCloud<PointTypeXYZ> (point_cloud_src_aligned_ptr, pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (point_cloud_src_aligned_ptr, 0, 255, 0), "source_keypoints_aligned");
            visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source_keypoints_aligned");
            visu.addPointCloud<PointTypeXYZ> (draw_shapes, pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (draw_shapes, 0, 255, 0), "draw_shapes");
            visu.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "draw_shapes");
            visu.addArrow(draw_shapes->points[1], draw_shapes->points[0],0,255,0,true,"position_vector",0);
            visu.spin ();
        
            while (!visu.wasStopped ())
            {
                //range_image_widget.spinOnce ();  // process GUI events
                visu.spinOnce ();
                pcl_sleep(0.01);
            }
        }
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
        retval = EXIT_FAILURE;
    }  
   
    return retval;
}