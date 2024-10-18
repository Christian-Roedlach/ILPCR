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
 * File:            filters.h
 * Description:     functions for filtering point cloud datasets
 */

#ifndef ILPCR_FILTERS_H
#define ILPCR_FILTERS_H

#include "types.h"
#include <pcl/filters/statistical_outlier_removal.h>

namespace ilpcr {
    pcl::StatisticalOutlierRemoval<PointTypeXYZ>::Ptr stat_removal_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out);
    void voxel_grid_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, double raster_size);
    void remove_cube_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, float length, Eigen::Vector3f *offset);
    void remove_cuboid_filter(CloudTypeXYZ::Ptr cloud_in, CloudTypeXYZ::Ptr cloud_out, Eigen::Vector3f *center, Eigen::Vector3f *width);
}

#endif // ILPCR_FILTERS_H