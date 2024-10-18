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
 * File:            15_fpfh_registration.cpp
 * Description:     static research scenario: point cloud registration
 * 					with FPFH algorithm using ILPCR library 
 * 					(CPU and GPU, P-RANSAC and FGR)
 */

#include "cupoch/cupoch.h"
#include <ilpcr/feature_estimation.h>
#include <ilpcr/file_io.h>
#include <ilpcr/log.h>
#include <ilpcr/corr_estimation.h>

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <filesystem>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/transforms.h>

using namespace ilpcr;

#define TEST_MATCHING (0)

typedef pcl::visualization::PointCloudColorHandlerCustom<PointTypePointNT> ColorHandlerT;

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
	int retval = EXIT_FAILURE;
	std::string filename_src = "";
    std::string filename_tgt = "";
	std::string filename_tgt_desc = "";
	capture_data_t settings;
	fpfh_data_t fpfh_data;
	bool viewer_enabled = true;
	bool logging_enabled = false;
	const std::string LOG_HEADER = "hostname,executable,timestamp,cmd_args,srcCloud,destCloudPtNT,destCloudDesc,RANSAC_preset,NE&FEST src [ms],align [ms],fitScore,allignment,L2SQR,info,tf_martrix_filename,maxIter,simThrH,maxCorrDist,inlierFract";
	const std::string LOG_FILENAME = "../log/15_fpfh_registration.csv";
	std::ofstream logfile;
	CloudTypePointNT::Ptr align_pointNT (new CloudTypePointNT);
	CloudTypeXYZ::Ptr input_cloud (new CloudTypeXYZ);
	settings.realsense_pcl_filtered = input_cloud;
	
	init_pcd(&fpfh_data, pcr_preset_fpfh_1);

	cout << "Usage: ./15_fpfh_registration <dest cloud [.pcd]> <src cloud [.pcd]> [-v] [-log] [-p presetName | -pcfg presetFile]\n";

	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
    if (!pcd_filename_indices.empty ())
    {
		settings.filename = pcl::getFilenameWithoutExtension(argv[pcd_filename_indices[0]]);
				
        if (pcd_filename_indices.size() >= 2) {
			filename_src = argv[pcd_filename_indices[1]];
			pcl::io::loadPCDFile (filename_src, *settings.realsense_pcl_filtered);
		} else {
			std::cout << "2nd parameter (source cloud) required" << std::endl;
			return EXIT_FAILURE;
		}
    }
	else {
		std::cout << "1st parameter (target cloud) required" << std::endl;
		return EXIT_FAILURE;
	}

	filename_tgt = settings.filename + "_PointNormal.pcd";
    filename_tgt_desc = settings.filename + "_fpfh33.pcd";

    cout << "Source:    " << filename_src << 
          "\nTarget:    " << filename_tgt << "and \n" << 
		  	"           " << filename_tgt_desc << endl;

	if (pcl::console::find_argument (argc, argv, "-v") >= 0) {
		viewer_enabled = true;
		std::cout << "Viewer enabled.\n";
    } else {
		viewer_enabled = false;
		std::cout << "Viewer disabled.\n";
    }

	if (pcl::console::find_argument (argc, argv, "-log") >= 0) {
		logging_enabled = true;
		std::cout << "Logging enabled.\n";
    } else {
		logging_enabled = false;
		std::cout << "Logging disabled.\n";
    }

	std::string text;

	if (pcl::console::parse (argc, argv, "-p", text) >= 0) {
		retval = parse_preset(text, &fpfh_data);
		if (EXIT_SUCCESS != retval)
			return retval;
	} else if (pcl::console::parse (argc, argv, "-pcfg", text) >= 0) {
		retval = parse_preset_config_file(text, &fpfh_data);
		if (EXIT_SUCCESS == retval)
		{
			std::cout << "Using custom preset " << fpfh_data.pcr_preset->preset_name << " from file " << text << std::endl;
		} else {
			std::cerr << "ERROR: parsing custom preset from file " << text << " failed! " << std::endl;
			return retval;
		}
	}
	
	retval = ilpcr::loadDestinationCloud(&settings, &fpfh_data);
	if (EXIT_SUCCESS != retval)
		return retval;

	// LOGGING
	set_log_enabled(logging_enabled);

	if (EXIT_SUCCESS != log_init(LOG_FILENAME, LOG_HEADER, &logfile))
		exit (EXIT_FAILURE);

	log_write_process_info(&logfile, argc, argv);
	log_write(filename_src, false, &logfile); /* srcCloud */
	log_write(filename_tgt + "," + filename_tgt_desc, false, &logfile); /* destCloud */

	log_write(fpfh_data.pcr_preset->preset_name, false, &logfile); /* RANSAC_preset */
			
	{
		pcl::ScopeTime t("Downsampling, Normals and Feature estimation source cloud");
		feature_estimation(&settings, &fpfh_data);
		log_write(std::to_string(t.getTime()), false, &logfile); /* NE&FEST src [ms] */
	}

	// Perform alignment
	pcl::console::print_highlight ("Starting alignment...\n");

	bool converged = false;

	#if (TEST_MATCHING)
	{
		pcl::ScopeTime t("Alignment_TestMatching");
		converged = matchPointCloudsTest(&settings, &fpfh_data);
		//log_write(std::to_string(t.getTime()), false, &logfile); /* align [ms] */
	}
	
	#else // (TEST_MATCHING)
	{
		//alignType_Fpfh align;
		pcl::ScopeTime t("Alignment");
		converged = matchPointClouds(&settings, &fpfh_data);
		log_write(std::to_string(t.getTime()), false, &logfile); /* align [ms] */
	}
	
	#endif // TEST_MATCHING

	std::cout << "RANSAC fitness score " << fpfh_data.fitnessScore << std::endl;
	log_write(std::to_string(fpfh_data.fitnessScore), false, &logfile); /* fitScore */
	
	if (converged)
	{
		log_write("CONVERGED", false, &logfile); /* allignment */
		
		/* calculate PCR score */
		double pcr_score = validateTransformationEuclidean(fpfh_data.src_fpfh_pcd_PointNT_ptr,
				fpfh_data.dst_fpfh_pcd_PointNT_ptr, &settings.transformation);

		log_write(std::to_string(pcr_score), 
				false, &logfile); /* L2SQR validation */
		
		std::cout << "L2SQR score = " << pcr_score << std::endl;

		std::string allignment_info = "Inliers: " + 
				std::to_string(fpfh_data.inliers) + 
				" / " +
				std::to_string(fpfh_data.src_fpfh_pcd_PointNT_ptr->size()) + 
				" (" + 
				std::to_string((float) fpfh_data.inliers * 100 / fpfh_data.src_fpfh_pcd_PointNT_ptr->size ())
				+ " %)";

		pcl::console::print_info ((allignment_info + "\n").c_str());
		log_write(allignment_info, false, &logfile); /* info */

		Eigen::Affine3f tf_matrix_export;
		tf_matrix_export = settings.transformation;
		std::filesystem::path matrix_path = pcl::getFilenameWithoutExtension(filename_src);
		std::string tf_matrix_filename_base = 
				matrix_path.parent_path().string() + 
				"/feature_desc/" +
				matrix_path.filename().string() +
				"_PointNormal_" + 
				fpfh_data.pcr_preset->preset_name + 
				".pcd";		

		ilpcr::store_tf_matrix_to_file(tf_matrix_export, tf_matrix_filename_base);

		log_write(pcl::getFilenameWithoutExtension(filename_src) + "_tfmatrix.Affine3f", 
				false, &logfile); /* tf_martrix_filename */
				
		if (viewer_enabled) 
		{

			log_close(&logfile);
			// Show alignment
			pcl::visualization::PCLVisualizer viewer("Alignment");
			viewer.getRenderWindow()->GlobalWarningDisplayOff(); 
			viewer.setBackgroundColor (255, 255, 255);

			// targed cloud
			viewer.addCoordinateSystem (1.0);
			viewer.addPointCloud (fpfh_data.dst_fpfh_pcd_PointNT_ptr, ColorHandlerT (fpfh_data.dst_fpfh_pcd_PointNT_ptr, 0.0, 0.0, 0.0), "target");
			
			// aligned source cloud
			viewer.addCoordinateSystem(2, settings.transformation, "source_coord_system");
			viewer.addPointCloud (fpfh_data.point_cloud_src_aligned_ptr, ColorHandlerT (fpfh_data.point_cloud_src_aligned_ptr, 0.0, 127.0, 0.0), "source_aligned");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_aligned");

			viewer.spin ();
			log_reopen(LOG_FILENAME, &logfile);
		}
		retval = EXIT_SUCCESS;
	}
	else
	{
		pcl::console::print_error ("Alignment failed!\n");

		log_write("FAILED", false, &logfile); /* allignment */
		log_write("none", false, &logfile); /* L2SQR validation */
		log_write("none", false, &logfile); /* info */
		log_write("none", false, &logfile); /* tf_martrix_filename */

		retval = EXIT_FAILURE;
	}
	
	
	log_write(std::to_string(fpfh_data.pcr_preset->max_iterations), false, &logfile); /* maxIter */
	log_write(std::to_string(fpfh_data.pcr_preset->sim_threshold), false, &logfile); /* simThrH */
	log_write(std::to_string(fpfh_data.pcr_preset->max_corr_distance), false, &logfile); /* maxCorrDist */
	log_write(std::to_string(fpfh_data.pcr_preset->inlier_fraction), true, &logfile); /* inlierFract */

	log_close(&logfile);

	if (pcr_names_custom == fpfh_data.pcr_preset->preset) {
		delete fpfh_data.pcr_preset;
		fpfh_data.pcr_preset = nullptr;
	}
		
	return retval;
}
