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
 * File:            02_convert_log.cpp
 * Description:     real-time scenario: convert binary log-file data
 *                  acquired with the real-time application (pipeline) 
 *                  to an ASCII CSV file
 */

#include <ilpcr/types.h>
#include <ilpcr/file_io.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <unistd.h>

#include <pcl/console/parse.h>

#include <date/date.h>

#define USAGE   "usage: ./02_convert_log <log filename> [-conv]" \
                "   <log filename>  filename of binary logfile" \
                "   [-conv]         only convert successfully alligned data (optional)" \
                "output file is inputfile.csv (without original file extension)"

#define ILPCR_DEBUG 0

#define CSV_SEPERATOR ", "

#define CSV_HEADER \
        "id" CSV_SEPERATOR \
        "timestamp" CSV_SEPERATOR \
        "converged" CSV_SEPERATOR \
        "ram_VmSize" CSV_SEPERATOR \
        "ram_VmRSS" CSV_SEPERATOR \
        "gpu_total" CSV_SEPERATOR \
        "gpu_fest" CSV_SEPERATOR \
        "gpu_pose_est" CSV_SEPERATOR \
        "tf_row1" CSV_SEPERATOR \
        "tf_row2" CSV_SEPERATOR \
        "tf_row3" CSV_SEPERATOR \
        "tf_row4" CSV_SEPERATOR \
        "timestamp_post_capture" CSV_SEPERATOR \
        "timestamp_post_fest" CSV_SEPERATOR \
        "timestamp_post_pose_est" CSV_SEPERATOR \
        "successful_count"

using namespace ilpcr;
using namespace std;

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
    int retval = EXIT_FAILURE;
    string filename = "";
    string line = "";
    ifstream input_file;
    ofstream output_file;
    string output_filename;
    bool converged_only = false;
    
    if (1 < argc)
    {
        filename = argv[1];
        output_filename = pcl::getFilenameWithoutExtension(filename) + ".csv";
        retval = EXIT_SUCCESS;
    }
    else
    {
        cerr << "ERROR: no argument found - name of logfile is required!" << endl;
        cout << USAGE << endl;
        retval = EXIT_FAILURE;
    }

    if (2 < argc)
    {
        if (pcl::console::find_argument (argc, argv, "-conv") >= 0) {
            converged_only = true;
            std::cout << "--> converting successfully alligned data only" << endl;
        } else {
            converged_only = false;
            std::cout << "--> converting all data" << endl;
        }
    }

    if (EXIT_SUCCESS == retval)
    {
        retval = pose_log_check_header(&input_file, &filename);
    }

    if (EXIT_SUCCESS == retval)
    {
        output_file.open(output_filename, std::ofstream::out);

        if (output_file.is_open())
        {    
            cout << "Writing data to file: " << output_filename << endl;
            retval = EXIT_SUCCESS;
        }
        else 
        {
            cerr << "ERROR: unable to open output file " << output_filename << endl;
            retval = EXIT_FAILURE;
        }
    }

    if (EXIT_SUCCESS == retval)
    {
        pose_data_t data_buffer;
        const static Eigen::IOFormat csv_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", CSV_SEPERATOR);

        output_file << CSV_HEADER << endl;

        while (true)
        {
            input_file.read((char *)(&data_buffer), sizeof(pose_data_t));

            if (!(input_file.good() && output_file.good()))
                break;

            if (converged_only)
            {
                if (!data_buffer.converged)
                    continue;
            }

            output_file << data_buffer.id << CSV_SEPERATOR;
            output_file << date::format("%F %T UTC", time_point_cast<std::chrono::milliseconds>(data_buffer.timestamp)) << CSV_SEPERATOR;
            output_file << data_buffer.converged << CSV_SEPERATOR;
            output_file << data_buffer.memory.ram_VmSize << CSV_SEPERATOR;
            output_file << data_buffer.memory.ram_VmRSS << CSV_SEPERATOR;
            output_file << data_buffer.memory.gpu_total << CSV_SEPERATOR;
            output_file << data_buffer.memory.gpu_feat_est << CSV_SEPERATOR;
            output_file << data_buffer.memory.gpu_pose_est << CSV_SEPERATOR;
            output_file << data_buffer.tf_matrix.matrix().format(csv_format) << CSV_SEPERATOR;
            output_file << date::format("%F %T UTC", time_point_cast<std::chrono::milliseconds>(data_buffer.timestamp_post_capture)) << CSV_SEPERATOR;
            output_file << date::format("%F %T UTC", time_point_cast<std::chrono::milliseconds>(data_buffer.timestamp_post_fest)) << CSV_SEPERATOR;
            output_file << date::format("%F %T UTC", time_point_cast<std::chrono::milliseconds>(data_buffer.timestamp_post_pose_est)) << CSV_SEPERATOR;
            output_file << data_buffer.successful_count << endl; 

            #if (ILPCR_DEBUG)
            cout << "id: " << data_buffer.id << endl;
            cout << "timestamp: " << date::format("%F %T UTC", time_point_cast<std::chrono::milliseconds>(data_buffer.timestamp)) << endl;
            cout << "converged: " << data_buffer.converged << endl;
            cout << "tf_matrix: " << data_buffer.tf_matrix.matrix().format(csv_format) << endl;   
            cout << "successful_count: " << data_buffer.successful_count << endl;        
            #endif // ILPCR_DEBUG
        }
    }

    input_file.close();
    output_file.close();

    return retval;
}
