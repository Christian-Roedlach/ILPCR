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
 * File:            faature_estimation.h
 * Description:     functions for key point and feature destcriptors estimation
 */

#ifndef ILPCR_FEATURE_ESTIMATION_H
#define ILPCR_FEATURE_ESTIMATION_H


#include "types.h"
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

namespace ilpcr {
    CloudTypeFPFH::Ptr fpfh_estimation(CloudTypePointNT::Ptr input_cloud, double search_radius);
    CloudTypeXYZ::Ptr downsample(CloudTypeXYZ::Ptr input_cloud, double raster_size);
    CloudTypePointNT::Ptr downsample_normal_estimation(
            CloudTypeXYZ::Ptr input_cloud, 
            double raster_size, 
            double search_radius, 
            bool use_surface_search=true);
    void feature_estimation (capture_data_t *capture_data, narf_data_t *narf_pcd);
    void feature_estimation (lidar_data_t *lidar_data, narf_data_t *narf_pcd);
    void feature_estimation (capture_data_t *capture_data, fpfh_data_t *fpfh_pcd);
    void feature_estimation (lidar_data_t *lidar_data, fpfh_data_t *fpfh_pcd);
    void init_pcd (narf_data_t *narf_pcd, pcr_names_t preset);
    void init_pcd (fpfh_data_t *fpfh_pcd, pcr_names_t preset);
}

#endif // ILPCR_FEATURE_ESTIMATION_H