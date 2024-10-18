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
 * File:            types.h
 * Description:     type definitions 
 */

#ifndef ILPCR_TYPES_H
#define ILPCR_TYPES_H

#if CUPOCH_ENABLED
#include "cupoch/cupoch.h"
#endif // CUPOCH_ENABLED
#include <pcl/impl/point_types.hpp>
#include <string>
#include <ctime>
#include <librealsense2/rs.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/fpfh_omp.h>
#include <Eigen/Eigen>

#include "presets.h"

#include <cassert>
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

namespace ilpcr 
{
    typedef pcl::PointXYZ PointTypeXYZ;
    typedef pcl::PointCloud<PointTypeXYZ> CloudTypeXYZ;
    typedef pcl::Narf36 FeatureTypeNarf36;
    typedef pcl::PointCloud<FeatureTypeNarf36> CloudTypeNarf36;
    typedef pcl::SampleConsensusPrerejective<PointTypeXYZ, PointTypeXYZ, FeatureTypeNarf36> alignType_Narf;

    typedef pcl::PointNormal PointTypePointNT;
    typedef pcl::PointCloud<PointTypePointNT> CloudTypePointNT;
    typedef pcl::FPFHSignature33 FeatureTypeFPFH;
    typedef pcl::FPFHEstimationOMP<PointTypePointNT,PointTypePointNT,FeatureTypeFPFH> featEstTypeFPFH;
    typedef pcl::PointCloud<FeatureTypeFPFH> CloudTypeFPFH;
    typedef pcl::SampleConsensusPrerejective<PointTypePointNT,PointTypePointNT,FeatureTypeFPFH> alignType_Fpfh;

    using timestamp_t = std::chrono::high_resolution_clock::time_point;

#if CUPOCH_ENABLED
    using cupoch_fpfh = cupoch::registration::Feature<33>;
    using cupoch_fpfh_ptr = std::shared_ptr<cupoch_fpfh>;
    using cupoch_pointNT = cupoch::geometry::PointCloud;
    using cupoch_pointNT_ptr = std::shared_ptr<cupoch_pointNT>; 

    class CupochData {
        public:
            CupochData();            

            cupoch_fpfh_ptr fpfh_descriptors_ptr;
            cupoch_pointNT_ptr fpfh_pcd_PointNT_ptr;

        private: 
    };

    using CupochData_ptr = std::shared_ptr<CupochData>;
#endif // CUPOCH_ENABLED
    typedef enum {
        ilpcr_narf,
        ilpcr_fpfh,
    } ilpcr_feature_t;

    typedef struct { // __attribute__((packed)) {
        uint64_t gpu_feat_est = 0;
        uint64_t gpu_pose_est = 0;
        uint64_t gpu_total = 0;
        uint64_t ram_VmSize = 0;
        uint64_t ram_VmRSS = 0;
    } ilpcr_memory_usage_t;

    typedef struct { //__attribute__((packed)) {
        uint64_t id = 0;
        uint64_t successful_count = 0;
        ilpcr_memory_usage_t memory;
        Eigen::Affine3f tf_matrix;
        timestamp_t timestamp;
        timestamp_t timestamp_post_capture;
        timestamp_t timestamp_post_fest;
        timestamp_t timestamp_post_pose_est;
        
        bool converged = false;
    } pose_data_t;

    ///* Ignoring warning as this unititialized preset is never used */
    //#pragma GCC diagnostic push
    //#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    //const rs2_filter_settings_t rs2_filter_preset_none = {
    //    .preset             = rs2_filter_preset_undefined,
    //};
    //#pragma GCC diagnostic pop

    typedef struct {
        CloudTypeNarf36::Ptr src_narf_descriptors_ptr;
        CloudTypeXYZ::Ptr src_narf_keypoints_XYZ_ptr;
        CloudTypeNarf36::Ptr dst_narf_descriptors_ptr;
        CloudTypeXYZ::Ptr dst_narf_keypoints_XYZ_ptr;
        CloudTypeXYZ::Ptr point_cloud_src_aligned_ptr;
        const pcr_settings_t *pcr_preset = &pcr_presets[pcr_preset_undefined];
    } narf_data_t;

    typedef struct {
        CloudTypeFPFH::Ptr src_fpfh_descriptors_ptr;
        CloudTypePointNT::Ptr src_fpfh_pcd_PointNT_ptr;
        CloudTypeFPFH::Ptr dst_fpfh_descriptors_ptr;
        CloudTypePointNT::Ptr dst_fpfh_pcd_PointNT_ptr;
        CloudTypePointNT::Ptr point_cloud_src_aligned_ptr;
    #if CUPOCH_ENABLED
        CupochData_ptr dst_cloud_cupoch_ptr;
    #endif // CUPOCH_ENABLED
        const pcr_settings_t *pcr_preset = &pcr_presets[pcr_preset_undefined];
        float raster_size;
        std::size_t inliers;
        double fitnessScore;
        uint64_t memory_gpu_pre_dst_cloud;
        uint64_t memory_gpu_post_dst_cloud;
    } fpfh_data_t;

    typedef enum {
        pipeline_state_undefined = -1,

        pipeline_state_running = 0,

        pipeline_state_failure_capturing,
        pipeline_state_failure_network,
        pipeline_state_failure_log,

        pipeline_state_stopped_by_user,
    } application_state_t;  

    typedef struct {
        application_state_t application_state = pipeline_state_undefined;
        rs2::pipeline pipe;
        CloudTypeXYZ::Ptr realsense_pcl_filtered;
        const rs2_filter_settings_t *rs2_filter_preset = &rs2_filter_presets[rs2_filter_preset_undefined];
        ilpcr_feature_t feature_type;
        pcr_names_t pcr_preset_name = pcr_preset_undefined;
        Eigen::Affine3f transformation;
        CloudTypeXYZ::Ptr realsense_pcl_original;
        narf_data_t *narf_data = NULL;
        fpfh_data_t *fpfh_data = NULL;
        bool save_pcd = false;
        bool display_viewer = false;
        std::string filename;
        bool capture_from_bag_file = false;
        std::string bag_filename = "";
        uint8_t framerate = 30;
        bool enableRGBstream = false;
        int resX = 848;
        int resY = 480;
        uint32_t image_number;
        ilpcr_memory_usage_t memory;
        bool exit;  
    } capture_data_t;

    typedef struct {
        CloudTypeXYZ::Ptr cloud_in;
        CloudTypeXYZ::Ptr cloud_transformed;
        CloudTypeXYZ::Ptr cloud_filtered;
        ilpcr_feature_t feature_type;
        pcr_names_t pcr_preset_name;
        std::string filename_in;
        std::string filename_desc_out;
        Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
        bool viewer = false;
        bool voxel_grid_filter = false;
        double vgf_scale = 0.01;
        double dst_scale = 1.;
        bool cube_remove_filter = false;
        float cube_filter_length = 0.;
        bool cuboid_remove_filter = false;
        Eigen::Vector3f *cuboid_filter_center;
        Eigen::Vector3f *cuboid_filter_width;
        const pcr_settings_t *pcr_preset = NULL;
    } lidar_data_t;
}

#endif // ILPCR_TYPES_H
