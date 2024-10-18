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
 * File:            corr_estimation.h
 * Description:     pose estimation functions for PCR
 */

#ifndef ILPCR_CORR_ESTIMATION_H
#define ILPCR_CORR_ESTIMATION_H

#include "types.h"

namespace ilpcr {
    bool matchPointClouds(capture_data_t *capture_data, narf_data_t *narf_pcd);
    bool matchPointClouds(capture_data_t *capture_data, narf_data_t *narf_pcd, alignType_Narf *align);
    bool matchPointClouds(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd);
    bool matchPointClouds(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd, alignType_Fpfh *align);
    void set_sample_cons_prerej_preset(narf_data_t *narf_pcd, pcr_names_t preset);
    void set_sample_cons_prerej_preset(fpfh_data_t *fpfh_pcd, pcr_names_t preset);
    bool matchPointCloudsTest(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd);
    double validateTransformationEuclidean(
            CloudTypeXYZ::Ptr source, 
            CloudTypeXYZ::Ptr target, 
            Eigen::Affine3f *tf_matrix);

    double validateTransformationEuclidean(
            CloudTypePointNT::Ptr source, 
            CloudTypePointNT::Ptr target, 
            Eigen::Affine3f *tf_matrix);
}

#endif // ILPCR_CORR_ESTIMATION_H