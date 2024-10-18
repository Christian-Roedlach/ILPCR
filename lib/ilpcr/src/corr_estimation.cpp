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
 * File:            corr_estimation.cpp
 * Description:     pose estimation functions for PCR
 */

#if CUPOCH_ENABLED
#include "ilpcrCupoch.h"
#endif // CUPOCH_ENABLED
#include "corr_estimation.h"
#include "file_io.h"
#include <iostream>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_validation_euclidean.h>

namespace ilpcr {
    static inline bool matchPointClouds_cpu(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd);
#if CUPOCH_ENABLED
    static inline bool matchPointClouds_gpu(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd);
#endif // CUPOCH_ENABLED

    bool matchPointClouds(capture_data_t *capture_data, narf_data_t *narf_pcd)
    {
        alignType_Narf align;
        return matchPointClouds(capture_data, narf_pcd, &align);
    }

    bool matchPointClouds(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd)
    {
        bool retval = false;
        assertm(fpfh_pcd->pcr_preset->preset != pcr_preset_undefined, "RANSAC sample consensus prerejective preset was not initialized!");

        switch (fpfh_pcd->pcr_preset->processing_unit)
        {
            case ilpcr_proc_type_cpu:
                retval = matchPointClouds_cpu(capture_data, fpfh_pcd);
                break;
#if CUPOCH_ENABLED
            case ilpcr_proc_type_fgr_gpu:
            case ilpcr_proc_type_fpfh_fgr_gpu:
                retval = matchPointClouds_gpu(capture_data, fpfh_pcd);
                break;
#endif // CUPOCH_ENABLED
            default:
                retval = false;
        }

        return retval;        
    }

    static inline bool matchPointClouds_cpu(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd)
    {
        alignType_Fpfh align;
        return matchPointClouds(capture_data, fpfh_pcd, &align);
    }

    #if CUPOCH_ENABLED
    static inline bool matchPointClouds_gpu(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd)
    {       
        int retval = EXIT_FAILURE;
        retval = matchPointClouds_cupoch(capture_data, fpfh_pcd);

        /* PERFORMANCE: src point cloud is transformed here - this may not be required */
        if (EXIT_SUCCESS == retval)
        {
            CloudTypePointNT::Ptr align_pointNT (new CloudTypePointNT);
            pcl::transformPointCloud(*fpfh_pcd->src_fpfh_pcd_PointNT_ptr, *align_pointNT, capture_data->transformation);
            fpfh_pcd->point_cloud_src_aligned_ptr = align_pointNT;
        }
        
        if (EXIT_SUCCESS == retval)
            return true;
        else
            return false;
    }
    #endif // CUPOCH_ENABLED

/*
    // TransformationEstimationSVD
    bool matchPointCloudsTest(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd)
    {
        pcl::Correspondences correspondences;
        pcl::registration::CorrespondenceEstimation<FeatureTypeFPFH, FeatureTypeFPFH > corrEst;
        corrEst.setInputSource(fpfh_pcd->src_fpfh_descriptors_ptr);
        corrEst.setInputTarget(fpfh_pcd->dst_fpfh_descriptors_ptr);
        corrEst.determineCorrespondences(correspondences, 
                fpfh_pcd->pcr_preset->max_corr_distance);

        std::cout << "NR of Correspondences: " << correspondences.size() << std::endl;

        pcl::registration::TransformationEstimationSVD<PointTypePointNT, PointTypePointNT> transf_est_svd;

        transf_est_svd.estimateRigidTransformation(*fpfh_pcd->src_fpfh_pcd_PointNT_ptr, *fpfh_pcd->dst_fpfh_pcd_PointNT_ptr, correspondences, capture_data->transformation.matrix());
        std::cout << "Transformation Matrix svd:" << std::endl << capture_data->transformation.matrix() << std::endl;

        std::cout << "Number of source keypoints:      " << fpfh_pcd->src_fpfh_pcd_PointNT_ptr->size() << std::endl;
        std::cout << "Number of destination keypoints: " << fpfh_pcd->dst_fpfh_pcd_PointNT_ptr->size() << std::endl;
        
        fpfh_pcd->inliers = correspondences.size();
        fpfh_pcd->fitnessScore = NAN;

        CloudTypePointNT::Ptr point_cloud_src_aligned_ptr (new CloudTypePointNT);
        fpfh_pcd->point_cloud_src_aligned_ptr = point_cloud_src_aligned_ptr;

        pcl::transformPointCloud(*fpfh_pcd->src_fpfh_pcd_PointNT_ptr, *fpfh_pcd->point_cloud_src_aligned_ptr, capture_data->transformation);

        return true;
    }
*/

    // GeneralizedIterativeClosestPoint GICP - set >#define TEST_MATCHING (1) in 06_normal_fpfh.cpp<
    bool matchPointCloudsTest(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd)
    {
        //pcl::GeneralizedIterativeClosestPoint<FeatureTypeFPFH, FeatureTypeFPFH> gicp_fpfh;
        pcl::GeneralizedIterativeClosestPoint<PointTypeXYZ, PointTypeXYZ> gicp_pointNT;
        //Eigen::Matrix4f guess_fpfh;
        //CloudTypeFPFH::Ptr align_fpfh (new CloudTypeFPFH);
        Eigen::Matrix4f guess_pointNT;
        CloudTypeXYZ::Ptr align_pointXYZ (new CloudTypeXYZ);
        CloudTypePointNT::Ptr align_pointNT (new CloudTypePointNT);
        CloudTypeXYZ::Ptr srcXYZ (new CloudTypeXYZ);
        CloudTypeXYZ::Ptr dstXYZ (new CloudTypeXYZ);

        pcl::copyPointCloud(*fpfh_pcd->src_fpfh_pcd_PointNT_ptr, *srcXYZ);
        pcl::copyPointCloud(*fpfh_pcd->dst_fpfh_pcd_PointNT_ptr, *dstXYZ);

        gicp_pointNT.setMaxCorrespondenceDistance(1);
        //gicp_pointNT.setTranslationGradientTolerance(0.25);

        //gicp_fpfh.setInputSource(fpfh_pcd->src_fpfh_descriptors_ptr);
        //gicp_fpfh.setInputTarget(fpfh_pcd->dst_fpfh_descriptors_ptr);

        gicp_pointNT.setInputSource(srcXYZ);
        gicp_pointNT.setInputTarget(dstXYZ);

        //gicp_pointNT.setMaximumOptimizerIterations(100000);
        gicp_pointNT.min_number_correspondences_ = 300;
        gicp_pointNT.setRANSACIterations(100000);

        //gicp_fpfh.align(*align_fpfh, guess_fpfh);
        gicp_pointNT.align(*align_pointXYZ);

        //std::cout << "FPFH Fitness Score: " << gicp_fpfh.getFitnessScore() << std::endl;
        //print_transformation(guess_fpfh);
        std::cout << "PointNT Fitness Score: " << gicp_pointNT.getFitnessScore() << std::endl;
        std::cout << "gicp_pointNT.getMaxCorrespondenceDistance(): " << gicp_pointNT.getMaxCorrespondenceDistance() << std::endl;
        std::cout << "align_pointXYZ->size(): " << align_pointXYZ->size() << std::endl;
        std::cout << "gicp_pointNT.getRotationEpsilon(): " << gicp_pointNT.getRotationEpsilon() << std::endl;
        std::cout << "gicp_pointNT.getRotationGradientTolerance(): " << gicp_pointNT.getRotationGradientTolerance() << std::endl;
        std::cout << "gicp_pointNT.getTranslationGradientTolerance(): " << gicp_pointNT.getTranslationGradientTolerance() << std::endl; 
        std::cout << "gicp_pointNT.getRANSACIterations(): " << gicp_pointNT.getRANSACIterations() << std::endl;
        guess_pointNT = gicp_pointNT.getFinalTransformation();
        print_transformation(guess_pointNT);

        capture_data->transformation = guess_pointNT;
        pcl::transformPointCloud(*fpfh_pcd->src_fpfh_pcd_PointNT_ptr, *align_pointNT, capture_data->transformation);

        //pcl::copyPointCloud(*align_pointXYZ, *align_pointNT);
        fpfh_pcd->point_cloud_src_aligned_ptr = align_pointNT;
        
        return true;
    }
    
    bool matchPointClouds(capture_data_t *capture_data, narf_data_t *narf_pcd, alignType_Narf *align)
    {
        assertm(narf_pcd->pcr_preset->preset != pcr_preset_undefined, "RANSAC sample consensus prerejective preset was not initialized!");

        bool retval = false;        
        CloudTypeXYZ::Ptr point_cloud_src_aligned_ptr (new CloudTypeXYZ);
        narf_pcd->point_cloud_src_aligned_ptr = point_cloud_src_aligned_ptr;

        align->setInputSource (narf_pcd->src_narf_keypoints_XYZ_ptr);
        align->setSourceFeatures (narf_pcd->src_narf_descriptors_ptr);
        align->setInputTarget (narf_pcd->dst_narf_keypoints_XYZ_ptr);
        align->setTargetFeatures(narf_pcd->dst_narf_descriptors_ptr);
        align->setMaximumIterations (narf_pcd->pcr_preset->max_iterations); // Number of RANSAC iterations
        align->setNumberOfSamples (narf_pcd->pcr_preset->nr_of_samples); // Number of points to sample for generating/prerejecting a pose
        align->setCorrespondenceRandomness (narf_pcd->pcr_preset->corr_rand); // Number of nearest features to use
        align->setSimilarityThreshold (narf_pcd->pcr_preset->sim_threshold); // Polygonal edge length similarity threshold
        align->setMaxCorrespondenceDistance (narf_pcd->pcr_preset->max_corr_distance); // Inlier threshold
        align->setInlierFraction (narf_pcd->pcr_preset->inlier_fraction); // Required inlier fraction for accepting a pose hypothesis

        // If number of points is smaller then sample size, align causes a segmentation fault
        if (narf_pcd->src_narf_keypoints_XYZ_ptr->size() >= (size_t) align->getNumberOfSamples())
        {
            align->align (*point_cloud_src_aligned_ptr);
        
            std::cout << "Alignment fitness score:   " << align->getFitnessScore() << std::endl;

            if (align->hasConverged ())
            {
                retval = true;
                capture_data->transformation.matrix() = align->getFinalTransformation();
                printAlignmentInfo(capture_data, align);
            }
            else
                retval = false;
        }
        else
            retval = false;
        
        if (false == retval)
        {
            pcl::console::print_error ("Alignment failed!\n");
        }

        return retval;
    }

    bool matchPointClouds(capture_data_t *capture_data, fpfh_data_t *fpfh_pcd, alignType_Fpfh *align)
    {
        bool retval = false;
        CloudTypePointNT::Ptr point_cloud_src_aligned_ptr (new CloudTypePointNT);
        fpfh_pcd->point_cloud_src_aligned_ptr = point_cloud_src_aligned_ptr;

        align->setInputSource (fpfh_pcd->src_fpfh_pcd_PointNT_ptr);
        align->setSourceFeatures (fpfh_pcd->src_fpfh_descriptors_ptr);
        align->setInputTarget (fpfh_pcd->dst_fpfh_pcd_PointNT_ptr);
        align->setTargetFeatures(fpfh_pcd->dst_fpfh_descriptors_ptr);
        
        align->setMaximumIterations (fpfh_pcd->pcr_preset->max_iterations); // Number of RANSAC iterations
        align->setNumberOfSamples (fpfh_pcd->pcr_preset->nr_of_samples); // Number of points to sample for generating/prerejecting a pose
        align->setCorrespondenceRandomness (fpfh_pcd->pcr_preset->corr_rand); // Number of nearest features to use
        align->setSimilarityThreshold (fpfh_pcd->pcr_preset->sim_threshold); // Polygonal edge length similarity threshold
        align->setMaxCorrespondenceDistance (fpfh_pcd->pcr_preset->max_corr_distance); // Inlier threshold
        align->setInlierFraction (fpfh_pcd->pcr_preset->inlier_fraction); // Required inlier fraction for accepting a pose hypothesis

        // If number of points is smaller then sample size, align causes a segmentation fault
        if (fpfh_pcd->src_fpfh_descriptors_ptr->size() >= (size_t) align->getNumberOfSamples())
        {
            align->align (*point_cloud_src_aligned_ptr);
         
            std::cout << "Alignment fitness score:   " << align->getFitnessScore() << std::endl;
            fpfh_pcd->fitnessScore = align->getFitnessScore();
            fpfh_pcd->inliers = align->getInliers().size();

            if (align->hasConverged ())
            {
                retval = true;
                capture_data->transformation.matrix() = align->getFinalTransformation();
                printAlignmentInfo(capture_data, align);
            }
            else
                retval = false;
        }
        else
            retval = false;
        
        if (false == retval)
        {
            pcl::console::print_error ("Alignment failed!\n");
        }

        return retval;
    }

    void set_sample_cons_prerej_preset(narf_data_t *narf_pcd, pcr_names_t preset)
	{
		narf_pcd->pcr_preset = &pcr_presets[preset];
        assertm(narf_pcd->pcr_preset->preset == preset, "RANSAC sample consensus prerejective preset integrity check failed!");
	}
    
	void set_sample_cons_prerej_preset(fpfh_data_t *fpfh_pcd, pcr_names_t preset)
	{
		fpfh_pcd->pcr_preset = &pcr_presets[preset];
        
        if (fpfh_pcd->pcr_preset->preset != preset) 
        {
            std::string message = "Preset array number entry is not correct: " 
                + std::to_string(fpfh_pcd->pcr_preset->preset) + " != " + std::to_string(preset);
            std::cerr << message << std::endl;
        }

        assertm(fpfh_pcd->pcr_preset->preset == preset, "RANSAC sample consensus prerejective preset integrity check failed!");
	}

    double validateTransformationEuclidean(
            CloudTypeXYZ::Ptr source, 
            CloudTypeXYZ::Ptr target, 
            Eigen::Affine3f *tf_matrix)
    {
        double result = 0;
        pcl::registration::TransformationValidationEuclidean<
                PointTypeXYZ, PointTypeXYZ> val_trans_euc;
            
        val_trans_euc.setMaxRange(0.5);

        result = val_trans_euc.validateTransformation(
                source, target, tf_matrix->matrix());

        return result;
    }

    double validateTransformationEuclidean(
            CloudTypePointNT::Ptr source, 
            CloudTypePointNT::Ptr target, 
            Eigen::Affine3f *tf_matrix)
    {
        double result = 0;
        pcl::registration::TransformationValidationEuclidean<
                PointTypePointNT, PointTypePointNT> val_trans_euc;
            
        val_trans_euc.setMaxRange(0.5);

        result = val_trans_euc.validateTransformation(
                source, target, tf_matrix->matrix());

        return result;
    }


}