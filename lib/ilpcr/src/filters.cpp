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
 * File:            filters.cpp
 * Description:     functions for filtering point cloud datasets
 */

#include "filters.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

namespace ilpcr {
    static void box_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, Eigen::Vector4f *min, Eigen::Vector4f *max, bool negative);
    
    pcl::StatisticalOutlierRemoval<PointTypeXYZ>::Ptr stat_removal_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out)
    {
        std::cout << "Statistical removal: cloud before filtering: " << std::endl;
        std::cout << *cloud_in << std::endl;

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<PointTypeXYZ>::Ptr sor (new pcl::StatisticalOutlierRemoval<PointTypeXYZ>);
        sor->setInputCloud(cloud_in);
        sor->setMeanK(50);
        sor->setStddevMulThresh(1.0);
        sor->filter(*cloud_out);

        std::cout << "Statistical removal: cloud after filtering: " << std::endl;
        std::cout << *cloud_out << std::endl;

        return sor;
    }

    void voxel_grid_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, double raster_size)
    {
        // create the voxel filtering object
        pcl::VoxelGrid<PointTypeXYZ> voxel_grig_filter;
        voxel_grig_filter.setInputCloud (cloud_in);
        voxel_grig_filter.setLeafSize (raster_size, raster_size, raster_size);
        // apply voxel grid filter
        voxel_grig_filter.filter (*cloud_out);
    }

    static void box_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, Eigen::Vector4f *min, Eigen::Vector4f *max, bool negative)
    {
        pcl::CropBox<PointTypeXYZ> boxfilter;
        boxfilter.setInputCloud(cloud_in);
        boxfilter.setMin(*min);
        boxfilter.setMax(*max);
        boxfilter.setNegative(negative);
        boxfilter.filter(*cloud_out);
    }

    void remove_cube_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, float box_length, Eigen::Vector3f *offset)
    {
        float length = box_length / 2;
        Eigen::Vector4f max(offset->x() + length, offset->y() + length, offset->z() + length, 1.0);
        Eigen::Vector4f min(offset->x() - length, offset->y() - length, offset->z() - length, 1.0);

        box_filter(cloud_in, cloud_out, &min, &max, true);
    }

    void remove_cuboid_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, Eigen::Vector3f *center, Eigen::Vector3f *width)
    {
        Eigen::Vector4f max(center->x() + width->x() / 2,
                center->y() + width->y() / 2,
                center->z() + width->z() / 2,
                1.0);
                
        Eigen::Vector4f min(center->x() - width->x() / 2,
                center->y() - width->y() / 2,
                center->z() - width->z() / 2,
                1.0);

        box_filter(cloud_in, cloud_out, &min, &max, true);
    }
}