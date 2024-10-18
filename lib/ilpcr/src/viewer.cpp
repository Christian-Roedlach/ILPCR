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
 * File:            viewer.cpp
 * Description:     functions for PCD and pose visualization
 */

#include "viewer.h"

namespace ilpcr
{
    int transform_pcd_data(dph_settings_t* settings)
    {
        int retval = EXIT_FAILURE;

        if (settings->displ_corr_pcd)
        {
            for (size_t i = 0; i < settings->points_tf_matrices.size(); i++)
            {
                pcl::transformPointCloud(*settings->points_corr_pcd.at(i), 
                        *settings->points_corr_pcd.at(i), 
                        settings->points_tf_matrices.at(i));
            }
            retval = EXIT_SUCCESS; // no validation yet - exeption by std::vector<T,Allocator>::at if boundary check fails
        }
        else
            retval = EXIT_SUCCESS; // nothing had to be done

        return retval;
    }

    int display_3d_model(dph_settings_t* settings)
    {
        int retval = EXIT_FAILURE;

        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("ILPCR 3d Path"));
        settings->viewer = viewer;

        viewer->setSize(1200,900);

        viewer->getRenderWindow()->GlobalWarningDisplayOff(); 
        viewer->setBackgroundColor (1, 1, 1);
        viewer->addCoordinateSystem (1.0);
        if (viewer->addPointCloud<PointTypeXYZ> (settings->model_3d_pcd_xyz, 
                pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (settings->model_3d_pcd_xyz, 0, 0, 0),
                "3d-model"))
        {
            if (viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "3d-model"))
                retval = EXIT_SUCCESS;
        }
        
        return retval;
    }

    bool add_single_pos_and_path(dph_settings_t* settings, size_t position_number)
    {
        PointTypeXYZ point;
        CloudTypeXYZ::Ptr positions = settings->positions;
        auto viewer = settings->viewer;
        color_RGB_t c;
        color_RGB_t ca = settings->display_settings.color_arrow;
        color_RGB_t cp = settings->display_settings.color_path;
        bool viewer_retval = true;
        static bool first_time = true;
        size_t i = position_number;

        point.x = settings->points_tf_matrices.at(i) (0,3);
        point.y = settings->points_tf_matrices.at(i) (1,3);
        point.z = settings->points_tf_matrices.at(i) (2,3);
        positions->push_back(point);

        // get point color (for text and pcd display)
        c = settings->points_corr_pcd_colors.at(i % settings->points_corr_pcd_colors.size());

        // draw transformation coordinate system (representing rotation)
        if (settings->display_settings.display_orientation_history)
        {
            viewer->addCoordinateSystem(settings->display_settings.scale_coord_point,
                    settings->points_tf_matrices.at(i), settings->points_names.at(i));
        }

        /* REMOVING OBJECTS CURRENTLY IS NOT POSSIBLE - Error of failed request:  BadAccess (attempt to access private resource denied) */
        if (i > KEEP_LAST_NO_OF_ITEMS)
        {
            //viewer->removeShape("to_" + settings->points_names.at(i-KEEP_LAST_NO_OF_ITEMS));
            //viewer->removeCoordinateSystem(settings->points_names.at(i-KEEP_LAST_NO_OF_ITEMS));
        }

        // display point number
        if (settings->display_settings.display_pos_numbers)
        {
            viewer_retval = viewer->addText3D(std::to_string(settings->point_identifier[i]),
                    positions->points[i], 
                    settings->display_settings.scale_text_pos, c.r, c.g, c.b, 
                    "text_" + settings->points_names.at(i));
        }
        
        // Draw arrow to starting point
        if (i == 0)
        {
            if (viewer_retval)
                viewer_retval = viewer->addArrow(positions->points[0], PointTypeXYZ(0,0,0), ca.r, ca.g, ca.b, false,"position_start");
        }
        else
        {
            if (viewer_retval)
                viewer_retval = viewer->addLine(positions->points[i-1],
                        positions->points[i], cp.r, cp.g, cp.b, "to_" + settings->points_names.at(i));
            if (viewer_retval)
                viewer_retval = viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                        5, "to_" + settings->points_names.at(i));
        }

        if (settings->displ_corr_pcd)
        {
            if (viewer_retval)
                viewer_retval = viewer->addPointCloud<PointTypeXYZ> (settings->points_corr_pcd.at(i), 
                    pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (settings->points_corr_pcd.at(i),
                    c.r, c.g, c.b), "pcd_" + settings->points_names.at(i));
            if (viewer_retval)
                viewer_retval = viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                        1, "pcd_" + settings->points_names.at(i));
        }

        if (first_time)
        {
            first_time = false;
            if (viewer_retval)
                viewer_retval = viewer->addPointCloud<PointTypeXYZ> (positions, 
                        pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (positions, 0, 50, 0),
                        "positions");
            if (viewer_retval)
                viewer_retval = viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "positions");
            if (viewer_retval)
            {
                viewer->addCoordinateSystem(settings->display_settings.scale_orientation,
                        Eigen::Affine3f::Identity(), "actual_orientation");
                viewer_retval = viewer->updateCoordinateSystemPose("actual_orientation",
                        settings->points_tf_matrices.at(i));     
            }
        }   
        else
        {
            if (viewer_retval)
                viewer_retval = viewer->updatePointCloud(positions, 
                        pcl::visualization::PointCloudColorHandlerCustom<PointTypeXYZ> (positions, 0, 50, 0), 
                        "positions");
            if (viewer_retval)
            {
                        viewer_retval = viewer->updateCoordinateSystemPose("actual_orientation",
                        settings->points_tf_matrices.at(i));
            }
        }

        return viewer_retval;
    }

    int draw_pos_and_path(dph_settings_t* settings)
    {
        int retval = EXIT_FAILURE;
        size_t nr_of_positions = settings->points_tf_matrices.size();
        
        if (!settings->positions)
        {
            CloudTypeXYZ::Ptr positions(new ( CloudTypeXYZ));
            settings->positions = positions;
        }

        bool viewer_retval = false;

        for (size_t i = settings->points_already_drawn; i < nr_of_positions; i++)
        {
            viewer_retval = add_single_pos_and_path(settings, i);
                        
            if (!viewer_retval)
            {
                std::cout << "ERROR: was not able to draw position number: " << i << std::endl;
                break;
            }
        }

        if (viewer_retval)
            retval = EXIT_SUCCESS;
        else
        {
            std::cout << "ERROR: was not able to draw positions point cloud " << std::endl;
            retval = EXIT_FAILURE;
        } 

        if (nr_of_positions > 0)
            settings->points_already_drawn = nr_of_positions;
        
        return retval;
    }

    int init_points_corr_pcd_colors(dph_settings_t* settings)
    {
        int retval = EXIT_FAILURE;

        settings->points_corr_pcd_colors.push_back(color_RGB_t{255,0,0});   // red
        settings->points_corr_pcd_colors.push_back(color_RGB_t{0,255,0});   // green
        settings->points_corr_pcd_colors.push_back(color_RGB_t{0,0,255});   // blue
        settings->points_corr_pcd_colors.push_back(color_RGB_t{200,200,0}); // (dark)yellow
        settings->points_corr_pcd_colors.push_back(color_RGB_t{255,0,255}); // pink
        settings->points_corr_pcd_colors.push_back(color_RGB_t{0,255,255}); // cyan
        settings->points_corr_pcd_colors.push_back(color_RGB_t{255,127,0}); // orange
        settings->points_corr_pcd_colors.push_back(color_RGB_t{127,0,255}); // purple

        retval = EXIT_SUCCESS; // should always work 

        return retval;
    }
}