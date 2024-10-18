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
 * File:            ilpcrCupoch.cpp
 * Description:     functions utilizing the Cupoch library (FGR algorithm) 
 */

#if CUPOCH_ENABLED

#include "ilpcrCupoch.h"
#include "cupoch/registration/fast_global_registration.h"
#include <pcl/common/time.h>
#include <omp.h>

//#include <stdint.h>

//#include "cupoch/cupoch.h"
//#include "cupoch/registration/fast_global_registration.h"
//#include <climits>
//#include <chrono>
//#include <iomanip>

//using namespace std;
//using namespace std::chrono;
//using namespace cupoch;

namespace ilpcr {
    int convert_pcl_to_cupoch(CloudTypePointNT::Ptr input, cupoch_pointNT_ptr pointcloud);
    int convert_pcl_to_cupoch(CloudTypeFPFH::Ptr pcl_fpfh_cloud, cupoch_fpfh_ptr cupoch_fpfh);
    int fast_global_registration(
            fpfh_data_t *fpfh_data,
            CupochData_ptr source_ptr,
            Eigen::Affine3f *transformation);
    static inline void convert_point_FPFH33(
            vector33f_host_ptr fpfh_data_host,
            CloudTypeFPFH::Ptr pcl_fpfh_cloud,
            size_t i);
    static inline void convert_point_pointNT(
            std::shared_ptr<cupoch::io::HostPointCloud> pc_host,
            CloudTypePointNT::Ptr input,
            size_t i);
    inline void get_GPU_memory_free(uint64_t *free_mem);

    CupochData::CupochData()
    {
        fpfh_descriptors_ptr = std::make_shared<cupoch_fpfh>();
        fpfh_pcd_PointNT_ptr = std::make_shared<cupoch_pointNT>();
    }

    int loadDestinationCloud_cupoch(fpfh_data_t *fpfh_data)
    {
        int retval = EXIT_FAILURE;
        get_GPU_memory_free(&fpfh_data->memory_gpu_pre_dst_cloud);
        cupoch::utility::InitializeAllocator();
        cupoch::utility::SetVerbosityLevel(cupoch::utility::VerbosityLevel::Info);

        CupochData_ptr destination_cloud_ptr = std::make_shared<CupochData>();

        retval = convert_pcl_to_cupoch(fpfh_data->dst_fpfh_pcd_PointNT_ptr, destination_cloud_ptr->fpfh_pcd_PointNT_ptr);
        
        if (EXIT_SUCCESS == retval)
            retval = convert_pcl_to_cupoch(fpfh_data->dst_fpfh_descriptors_ptr, destination_cloud_ptr->fpfh_descriptors_ptr);
        
        if (EXIT_SUCCESS == retval)
            fpfh_data->dst_cloud_cupoch_ptr = destination_cloud_ptr;
        
        get_GPU_memory_free(&fpfh_data->memory_gpu_post_dst_cloud);

        return retval;
    }

    int matchPointClouds_cupoch(capture_data_t *capture_data, fpfh_data_t *fpfh_data)
    {
        int retval = EXIT_FAILURE;
        uint64_t gpu_memory_free_pre = 0;
        uint64_t gpu_memory_free_post = 0;

        get_GPU_memory_free(&gpu_memory_free_pre);

        capture_data->memory.gpu_feat_est = 
                fpfh_data->memory_gpu_post_dst_cloud - gpu_memory_free_pre;

        CupochData_ptr source_data_ptr = std::make_shared<CupochData>();
        

        if (nullptr != fpfh_data->dst_cloud_cupoch_ptr)
            retval = convert_pcl_to_cupoch(fpfh_data->src_fpfh_pcd_PointNT_ptr, source_data_ptr->fpfh_pcd_PointNT_ptr); 

        if (EXIT_SUCCESS == retval)
            retval = convert_pcl_to_cupoch(fpfh_data->src_fpfh_descriptors_ptr, source_data_ptr->fpfh_descriptors_ptr);

        if (EXIT_SUCCESS == retval)
            retval = fast_global_registration(
                    fpfh_data,
                    source_data_ptr,
                    &capture_data->transformation
            );

        get_GPU_memory_free(&gpu_memory_free_post);
        
        capture_data->memory.gpu_total = 
                fpfh_data->memory_gpu_pre_dst_cloud 
                - gpu_memory_free_post;

        capture_data->memory.gpu_pose_est = 
                gpu_memory_free_pre
                - gpu_memory_free_post;
    
        return retval;
    }

    int convert_pcl_to_cupoch(CloudTypePointNT::Ptr input, cupoch_pointNT_ptr pointcloud)
    {
        int retval = EXIT_FAILURE;
        std::shared_ptr<cupoch::io::HostPointCloud> pc_host = std::make_shared<cupoch::io::HostPointCloud>();

        size_t point_count = input->size();

        pc_host->points_.resize(point_count);
        pc_host->normals_.resize(point_count);

    /* TODO: not sure if using openmp is faster or not - to be investigated */
    #if USE_OMP_FOR_CUPOCH_CONVERSION
        #pragma omp parallel for
    #endif // USE_OMP_FOR_CUPOCH_CONVERSION
        for (size_t i = 0; i < point_count; i++)
        {
            convert_point_pointNT(pc_host, input, i);
        }

        if (pc_host->points_.size() == point_count
                && point_count == pc_host->normals_.size())
        {
            pc_host->ToDevice(*pointcloud);
            retval = EXIT_SUCCESS;
        }
    
        return retval;
    }

    int convert_pcl_to_cupoch(CloudTypeFPFH::Ptr pcl_fpfh_cloud, cupoch_fpfh_ptr cupoch_fpfh)
    {
        int retval = EXIT_FAILURE;

        vector33f_host_ptr fpfh_data_host = vector33f_host_ptr(new vector33f_host);
        size_t target_points_cnt = pcl_fpfh_cloud->size();
        fpfh_data_host->resize(target_points_cnt);

    /* TODO: not sure if using openmp is faster or not - to be investigated */
    #if USE_OMP_FOR_CUPOCH_CONVERSION
        #pragma omp parallel for
    #endif // USE_OMP_FOR_CUPOCH_CONVERSION
        for(size_t i = 0; i < target_points_cnt; i++)
        {
            convert_point_FPFH33(fpfh_data_host, pcl_fpfh_cloud, i);
        }

        cupoch_fpfh->SetData(*fpfh_data_host);

        if (cupoch_fpfh->Num() == target_points_cnt)
            retval = EXIT_SUCCESS;

        return retval;
    }


    int fast_global_registration(
            fpfh_data_t *fpfh_data,
            CupochData_ptr source_ptr,
            Eigen::Affine3f *transformation)
    {
        using namespace cupoch;

        int retval = EXIT_FAILURE;
        registration::FastGlobalRegistrationOption fgr_options;

        fgr_options.division_factor_ = 1.8;
        fgr_options.use_absolute_scale_ = true;
        fgr_options.decrease_mu_ = true;
        fgr_options.maximum_correspondence_distance_ = fpfh_data->pcr_preset->max_corr_distance;
        fgr_options.iteration_number_ = fpfh_data->pcr_preset->max_iterations;
        fgr_options.tuple_scale_ = fpfh_data->pcr_preset->sim_threshold;//0.95;
        fgr_options.maximum_tuple_count_ = 300;

        {
            //pcl::ScopeTime t("FGC cupoch with FPFH from pcl");
            auto fgr_result = registration::FastGlobalRegistration(
                    *source_ptr->fpfh_pcd_PointNT_ptr,
                    *fpfh_data->dst_cloud_cupoch_ptr->fpfh_pcd_PointNT_ptr,
                    *source_ptr->fpfh_descriptors_ptr,
                    *fpfh_data->dst_cloud_cupoch_ptr->fpfh_descriptors_ptr,
                    fgr_options);

            transformation->matrix() = fgr_result.transformation_;
            fpfh_data->fitnessScore = fgr_result.fitness_;
        /* Value is currently not inliers, but inlier RSME - to be validated */
            fpfh_data->inliers = fgr_result.inlier_rmse_ * fpfh_data->src_fpfh_pcd_PointNT_ptr->size();
            //std::cout << std::endl << "Fitness Level: " << fgr_result.fitness_ << ", RSME inliers: " << fgr_result.inlier_rmse_ << std::endl;
        }

        if (fpfh_data->fitnessScore >= fpfh_data->pcr_preset->inlier_fraction)
            retval = EXIT_SUCCESS;
        else
            retval = EXIT_FAILURE;
        
        return retval;
    }


    inline void convert_point_FPFH33(
        vector33f_host_ptr fpfh_data_host,
        CloudTypeFPFH::Ptr pcl_fpfh_cloud,
        size_t i)
    {
        for (size_t j = 0; j < 33; j++)
            {
                (*fpfh_data_host)[i][j] = pcl_fpfh_cloud->points.at(i).histogram[j];
            }
    }

    inline void convert_point_pointNT(
        std::shared_ptr<cupoch::io::HostPointCloud> pc_host,
        CloudTypePointNT::Ptr input,
        size_t i)
    {
        pc_host->points_[i](0) = input->points.at(i).x;
        pc_host->points_[i](1) = input->points.at(i).y;
        pc_host->points_[i](2) = input->points.at(i).z;

        pc_host->normals_[i](0) = input->points.at(i).normal_x;
        pc_host->normals_[i](1) = input->points.at(i).normal_y;
        pc_host->normals_[i](2) = input->points.at(i).normal_z;
    }

    inline void get_GPU_memory_free(uint64_t *free_mem)
    {
        cudaMemGetInfo(free_mem, NULL);
    }
} // namespace ilpcr
#endif //CUPOCH_ENABLED