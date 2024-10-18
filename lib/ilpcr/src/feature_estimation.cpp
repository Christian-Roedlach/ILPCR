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
 * File:            faature_estimation.cpp
 * Description:     functions for key point and feature destcriptors estimation
 */

#include "feature_estimation.h"
#include "corr_estimation.h"

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/narf.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <omp.h>

#if (PCL_GPU_ENABLED)
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/features/features.hpp>

#define GPU_NEST_MAX_NEIGHBORS 4096
#define GPU_FPFH_MAX_NEIGHBORS GPU_NEST_MAX_NEIGHBORS
#endif // PCL_GPU_ENABLED

namespace ilpcr {
	void feature_estimation (ilpcr::CloudTypeXYZ::Ptr cloud_filtered, fpfh_data_t *fpfh_pcd, bool is_dst_cloud);
#if (PCL_GPU_ENABLED)
    void fpfh_estimation_gpu(ilpcr::CloudTypeXYZ::Ptr cloud_filtered, fpfh_data_t *fpfh_pcd, bool is_dst_cloud);
#endif // PCL_GPU_ENABLED

	CloudTypeFPFH::Ptr fpfh_estimation(CloudTypePointNT::Ptr input_cloud, double search_radius) 
	{
		CloudTypeFPFH::Ptr features (new CloudTypeFPFH);

		// Estimate features
		// pcl::console::print_highlight ("Estimating features...\n");
		featEstTypeFPFH fest;
		fest.setRadiusSearch (search_radius);
		fest.setInputCloud (input_cloud);
		fest.setInputNormals (input_cloud);
		fest.compute (*features);
		
		return features;
	}

	#if (PCL_GPU_ENABLED)
    void fpfh_estimation_gpu(ilpcr::CloudTypeXYZ::Ptr cloud_filtered, fpfh_data_t *fpfh_pcd, bool is_dst_cloud)
	{
		CloudTypeXYZ::Ptr cloud_XYZ_downsampled;
		pcl::gpu::NormalEstimation::PointCloud cloud_device;
		pcl::gpu::NormalEstimation::PointCloud cloud_surface_device;
		pcl::gpu::NormalEstimation::Normals normals_device;
		pcl::gpu::NormalEstimation normalEstimation;
		/* cloud_normals_host_xyz is necessary, as normals are stored to PointXYZ (cuvature is data[3]) */
		CloudTypeXYZ::Ptr cloud_normals_host_xyz (new CloudTypeXYZ);
		ilpcr::CloudTypePointNT::Ptr cloud_PointNormal;
		ilpcr::CloudTypeFPFH::Ptr cloud_FPFH;
		pcl::gpu::FPFHEstimation fpfhEstimation;
		pcl::gpu::DeviceArray2D<pcl::FPFHSignature33> fpfhs_gpu;
		std::vector<pcl::FPFHSignature33> fpfhs_host;

		// Downsampling (CPU)
		cloud_XYZ_downsampled = downsample(cloud_filtered, fpfh_pcd->raster_size);

		// Upoad cloud to GPU
		cloud_device.upload(cloud_XYZ_downsampled->points);
		// NEST settings
		normalEstimation.setInputCloud(cloud_device);
		normalEstimation.setRadiusSearch(
				fpfh_pcd->pcr_preset->nest_search_radius, 
				GPU_NEST_MAX_NEIGHBORS);
		if (fpfh_pcd->pcr_preset->use_normal_surface_search)
		{
			cloud_surface_device.upload(cloud_filtered->points);
			normalEstimation.setSearchSurface(cloud_surface_device);
		}

		// Estimate Normals
		normalEstimation.compute(normals_device);

		// FPFH settings
		fpfhEstimation.setInputCloud(cloud_device);
    	fpfhEstimation.setInputNormals(normals_device);
    	fpfhEstimation.setRadiusSearch(fpfh_pcd->pcr_preset->feature_search_radius, GPU_FPFH_MAX_NEIGHBORS);

		// Estimate FPFHs
		fpfhEstimation.compute(fpfhs_gpu);

		// Download normals
		normals_device.download(cloud_normals_host_xyz->points);
		
		// Set pointer according to cloud type (lidar model == dst_cloud or capture)
		if (is_dst_cloud) {
			cloud_PointNormal = fpfh_pcd->dst_fpfh_pcd_PointNT_ptr;
			cloud_FPFH = fpfh_pcd->dst_fpfh_descriptors_ptr;
		} else {
			cloud_PointNormal = fpfh_pcd->src_fpfh_pcd_PointNT_ptr;
			cloud_FPFH = fpfh_pcd->src_fpfh_descriptors_ptr;
		}
		
		// Copy downsampled XYZ points to PointNormals type
		pcl::copyPointCloud(*cloud_XYZ_downsampled, *cloud_PointNormal);

		// Copy Normals (required, as pcl::gpu type is wrong format)
		assertm(cloud_normals_host_xyz->size() == cloud_PointNormal->size(), "ERROR: Point Cloud sizes do not match!");

		// Store Normals
		#pragma omp parallel for
		for (size_t i = 0; i < cloud_PointNormal->size(); i++)
		{
			cloud_PointNormal->points.at(i).normal_x = cloud_normals_host_xyz->points.at(i).x;
			cloud_PointNormal->points.at(i).normal_y = cloud_normals_host_xyz->points.at(i).y;
			cloud_PointNormal->points.at(i).normal_z = cloud_normals_host_xyz->points.at(i).z;
			// cuvature is stored in .data[3] element --> https://github.com/PointCloudLibrary/pcl/issues/2419
			cloud_PointNormal->points.at(i).curvature = cloud_normals_host_xyz->points.at(i).data[3];
		}	

		int cols;
		// Download and store FPFHs
		fpfhs_gpu.download(cloud_FPFH->points, cols);
				
		// Write FPFH point numbers
		cloud_FPFH->width = cloud_FPFH->size();
		cloud_FPFH->height = 1;

		/* Don't think that this is necessary (shared pointers)
		// free gpu data
		cloud_device.release();
		cloud_surface_device.release();
		normals_device.release();
		fpfhs_gpu.release();
		*/
	}
	#endif // PCL_GPU_ENABLED

	CloudTypeXYZ::Ptr downsample(CloudTypeXYZ::Ptr input_cloud, double raster_size) 
	{
		CloudTypeXYZ::Ptr cloud_XYZ_downsampled (new CloudTypeXYZ);

		// Downsample
		pcl::VoxelGrid<PointTypeXYZ> grid;
		const float leaf = raster_size;
		grid.setLeafSize (leaf, leaf, leaf);
		grid.setInputCloud (input_cloud);
		grid.filter (*cloud_XYZ_downsampled);

		return cloud_XYZ_downsampled;
	}

	CloudTypePointNT::Ptr downsample_normal_estimation(
			CloudTypeXYZ::Ptr input_cloud, 
			double raster_size, 
			double search_radius, 
			bool use_surface_search) 
	{
		CloudTypeXYZ::Ptr cloud_XYZ_downsampled;
		CloudTypePointNT::Ptr cloud_PointNormal (new CloudTypePointNT);

		// Downsample
		cloud_XYZ_downsampled = downsample(input_cloud, raster_size);

		// Copy point cloud to PointNormal datastructure, as NormalEstimationOMP only stores Normal data
		pcl::copyPointCloud(*cloud_XYZ_downsampled, *cloud_PointNormal);

		// Estimate normals for scene
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> nest;

		// Create the normal estimation class, and pass the input dataset to it
		nest.setInputCloud (cloud_XYZ_downsampled);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		nest.setSearchMethod (tree);

		// Use all neighbors in a sphere of raster_size * search_radius_multip
		if (search_radius > 0)
			nest.setRadiusSearch (search_radius);
		else
		{
			std::cerr << "WARNING: search_radius_multip is set to 0 - using 1" << std::endl;
			nest.setRadiusSearch (raster_size);
		}

		/* ToDo: Check if SearchSurface is an enhancement */
		if (use_surface_search)
			nest.setSearchSurface (input_cloud);

		// Compute the features
		nest.compute (*cloud_PointNormal);

		return cloud_PointNormal;
	}

	void feature_estimation (capture_data_t *capture_data, fpfh_data_t *fpfh_pcd) 
	{
		feature_estimation(capture_data->realsense_pcl_filtered, fpfh_pcd, false);
	}

	void feature_estimation (lidar_data_t *lidar_data, fpfh_data_t *fpfh_pcd) 
	{
		feature_estimation(lidar_data->cloud_filtered, fpfh_pcd, true);
	}

	void feature_estimation (ilpcr::CloudTypeXYZ::Ptr cloud_filtered, fpfh_data_t *fpfh_pcd, bool is_dst_cloud)
	{
		pcl::Indices unused;

		fpfh_pcd->raster_size = fpfh_pcd->pcr_preset->raster_size;
		pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, unused);
		
		switch (fpfh_pcd->pcr_preset->processing_unit)
		{
			#if (PCL_GPU_ENABLED)
			case ilpcr_proc_type_fpfh_fgr_gpu:
				fpfh_estimation_gpu(cloud_filtered, fpfh_pcd, is_dst_cloud);
				break;
			#endif // PCL_GPU_ENABLED
				
			case ilpcr_proc_type_cpu:
			case ilpcr_proc_type_fgr_gpu:
				if (is_dst_cloud)
				{
					fpfh_pcd->dst_fpfh_pcd_PointNT_ptr = downsample_normal_estimation(
						cloud_filtered, 
						fpfh_pcd->pcr_preset->raster_size,
						fpfh_pcd->pcr_preset->nest_search_radius,
						fpfh_pcd->pcr_preset->use_normal_surface_search);
					fpfh_pcd->dst_fpfh_descriptors_ptr = fpfh_estimation(fpfh_pcd->dst_fpfh_pcd_PointNT_ptr, 
						fpfh_pcd->pcr_preset->feature_search_radius);
				}
				else
				{
					fpfh_pcd->src_fpfh_pcd_PointNT_ptr = downsample_normal_estimation(
						cloud_filtered, 
						fpfh_pcd->pcr_preset->raster_size,
						fpfh_pcd->pcr_preset->nest_search_radius,
						fpfh_pcd->pcr_preset->use_normal_surface_search);
					fpfh_pcd->src_fpfh_descriptors_ptr = fpfh_estimation(fpfh_pcd->src_fpfh_pcd_PointNT_ptr, 
						fpfh_pcd->pcr_preset->feature_search_radius);
				}
				break;
			
			default:
				std::cerr << "ERROR: fpfh_pcd->pcr_preset->processing_unit is not handled: " << 
						fpfh_pcd->pcr_preset->processing_unit << std::endl;
		}
		
	}

	void feature_estimation (capture_data_t *capture_data, narf_data_t *narf_pcd) 
	{
		
		// -----------------------------------------------
		// -----Create RangeImage from the PointCloud-----
		// -----------------------------------------------
		float angular_resolution = 0.5f;
		angular_resolution = pcl::deg2rad (angular_resolution);
		float support_size = 0.3f;
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		bool setUnseenToMaxRange = false;
		bool rotation_invariant = true;
		Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

		float noise_level = 0.0;
		float min_range = 0.0f;
		int border_size = 1;
		pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;   
		pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
		
		range_image.createFromPointCloud (*capture_data->realsense_pcl_filtered.get(), angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
										scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
		range_image.integrateFarRanges (far_ranges);
		
		if (setUnseenToMaxRange)
			range_image.setUnseenToMaxRange ();
		
		// --------------------------------
		// -----Extract NARF keypoints-----
		// --------------------------------
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector;
		narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage (&range_image);
		narf_keypoint_detector.getParameters ().support_size = support_size;

		pcl::PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute (keypoint_indices);

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
		narf_descriptor.compute (*narf_pcd->src_narf_descriptors_ptr);

		std::size_t descr_size = narf_pcd->src_narf_descriptors_ptr->size();

		std::cout << "Extracted "<< descr_size << " descriptors for "
				<< keypoint_indices.size ()<< " keypoints.\n";
		
		narf_pcd->src_narf_keypoints_XYZ_ptr->resize(descr_size);
		CloudTypeXYZ* narf_keypoints_XYZ = narf_pcd->src_narf_keypoints_XYZ_ptr.get();

		for (std::size_t i=0; i<descr_size; ++i)
		{
			narf_keypoints_XYZ->points[i].x = narf_pcd->src_narf_descriptors_ptr->points[i].x;
			narf_keypoints_XYZ->points[i].y = narf_pcd->src_narf_descriptors_ptr->points[i].y;
			narf_keypoints_XYZ->points[i].z = narf_pcd->src_narf_descriptors_ptr->points[i].z;
		}
	}

	void feature_estimation (lidar_data_t *lidar_data, narf_data_t *narf_pcd) 
	{
		// -----------------------------------------------
		// -----Create RangeImage from the PointCloud-----
		// -----------------------------------------------
		float angular_resolution = 0.5f;
		angular_resolution = pcl::deg2rad (angular_resolution);
		float support_size = 0.3f;
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		bool setUnseenToMaxRange = false;
		bool rotation_invariant = true;
		Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

		float noise_level = 0.0;
		float min_range = 0.0f;
		int border_size = 1;
		pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
		pcl::RangeImage& range_image = *range_image_ptr;   
		pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
		
		range_image.createFromPointCloud (*lidar_data->cloud_filtered.get(), angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
				scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
		range_image.integrateFarRanges (far_ranges);
		
		if (setUnseenToMaxRange)
			range_image.setUnseenToMaxRange ();
		
		// --------------------------------
		// -----Extract NARF keypoints-----
		// --------------------------------
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector;
		narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage (&range_image);
		narf_keypoint_detector.getParameters ().support_size = support_size;

		pcl::PointCloud<int> keypoint_indices;
		narf_keypoint_detector.compute (keypoint_indices);

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
		narf_descriptor.compute (*narf_pcd->dst_narf_descriptors_ptr);

		std::size_t descr_size = narf_pcd->dst_narf_descriptors_ptr->size();

		std::cout << "Extracted "<< descr_size << " descriptors for "
				<< keypoint_indices.size ()<< " keypoints.\n";
		
		narf_pcd->dst_narf_keypoints_XYZ_ptr->resize(descr_size);
		CloudTypeXYZ* narf_keypoints_XYZ = narf_pcd->dst_narf_keypoints_XYZ_ptr.get();

		for (std::size_t i=0; i<descr_size; ++i)
		{
			narf_keypoints_XYZ->points[i].x = narf_pcd->dst_narf_descriptors_ptr->points[i].x;
			narf_keypoints_XYZ->points[i].y = narf_pcd->dst_narf_descriptors_ptr->points[i].y;
			narf_keypoints_XYZ->points[i].z = narf_pcd->dst_narf_descriptors_ptr->points[i].z;
		}
	}

	void init_pcd (narf_data_t *narf_pcd, pcr_names_t preset) {
		CloudTypeXYZ::Ptr src_narf_keypoints_XYZ_temp (new CloudTypeXYZ);
		narf_pcd->src_narf_keypoints_XYZ_ptr = src_narf_keypoints_XYZ_temp;
		CloudTypeXYZ::Ptr dst_narf_keypoints_XYZ_temp (new CloudTypeXYZ);
		narf_pcd->dst_narf_keypoints_XYZ_ptr = dst_narf_keypoints_XYZ_temp;
		CloudTypeNarf36::Ptr src_narf_descriptors_temp (new CloudTypeNarf36);
		narf_pcd->src_narf_descriptors_ptr = src_narf_descriptors_temp;
		CloudTypeNarf36::Ptr dst_narf_descriptors_temp (new CloudTypeNarf36);
		narf_pcd->dst_narf_descriptors_ptr = dst_narf_descriptors_temp;
		if (preset != pcr_names_custom)
			set_sample_cons_prerej_preset(narf_pcd, preset);
	}

	void init_pcd (fpfh_data_t *fpfh_pcd, pcr_names_t preset) {
		CloudTypeFPFH::Ptr src_fpfh_descriptors_ptr_temp (new CloudTypeFPFH);
		fpfh_pcd->src_fpfh_descriptors_ptr = src_fpfh_descriptors_ptr_temp;
		CloudTypePointNT::Ptr src_fpfh_keypoints_PointNT_ptr_temp (new CloudTypePointNT);
		fpfh_pcd->src_fpfh_pcd_PointNT_ptr = src_fpfh_keypoints_PointNT_ptr_temp;
		CloudTypeFPFH::Ptr dst_fpfh_descriptors_ptr_temp (new CloudTypeFPFH);
		fpfh_pcd->dst_fpfh_descriptors_ptr = dst_fpfh_descriptors_ptr_temp;
		CloudTypePointNT::Ptr dst_fpfh_keypoints_PointNT_ptr_temp (new CloudTypePointNT);
		fpfh_pcd->dst_fpfh_pcd_PointNT_ptr = dst_fpfh_keypoints_PointNT_ptr_temp;
		if (preset != pcr_names_custom)
			set_sample_cons_prerej_preset(fpfh_pcd, preset);
	}
}