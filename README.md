# Indoor Localization by Cross-Source Point Cloud Registration (ILPCR) library

This library and applications are part of the master thesis "Indoor Localization by Cross-Source Point Cloud Registration of LiDAR and RGB-D Datasets". Author: Christian Roedlach

ToDo: Link to Thesis

# 1 Prerequisites

## The code depends on: 

- RS-SDK2 2.54.2 [link](https://github.com/IntelRealSense/librealsense)
- PCL 1.14.0 [link](https://github.com/PointCloudLibrary/pcl)
- Cupoch 0.2.11.0 [link](https://github.com/neka-nat/cupoch), supposed build directory: ~/cupoch/build
- CUDA 11.4 (Laptop: 11.8) [link Xavier](https://developer.nvidia.com/embedded/jetpack-sdk-502), [link CUDA toolkit archive](https://developer.nvidia.com/cuda-toolkit-archive)

The program is built with cmake and tested on NVIDIA Jetson Xavier NX (NVIDIA’s JetPack SDK 5.0.2) and Laptop Development device (Ubuntu 22.04 on WSL2 x64). Please see the thesis for deatils. 

Please find the example CMake output files of the PCL and Cupoch libraries generated on the Xavier NX in ./CMakeLogs for the required build settings.

# 2 Build Executables

The software bundle is separated into four parts containing their own CMakeList.txt due to implementation progress reasons. The parts can be found in the following directories:

```console
./01_feature_ext
./02_correspondence_estimation
./03_data_aquisition
./05_pipeline
```

To build the programs, execute the following steps:

```console
cd <directory_of_preferred_part>
mkdir build && cd build
cmake ..
make
```

The programs are built into the build directory and should be executed from there. 

# 3 Datasets

The Manually Pre-Processed 3D model Datasets of Scene A and Scene B are located in ./files. Additionally, sample files of file lists and feature descritors are located in the corresponding sub-directories. 

# 4 Program Descriptions

The following sections describe the programs required to reproduce the thesis' results. 

## 4.1 Pre-Processing of the LiDAR Point Cloud Dataset

First, the LiDAR dataset has to be scaled, as one unit of the LiDAR dataset is 1 cm, but 1 m is required.

### 4.1.1 Program 01_feature_ext/14_lidar_preprocessing.cpp

The program is used to preprocess the 3D model \ac{PCD}. Please refere to the library or the thesis for the available presets.

```console
Usage: ./14_lidar_preprocessing <input cloud [.pcd]> [-t <feature type>] [-s scale_factor] \n \
     <-p <samp_cons_preset> | -pcfg <filename>> \n \
     [-x rot_x_degrees] [-y rot_y_degrees] [-z rot_z_degrees] [-v ... show 3Dviewer]\n \
     [bf <cube length>] [bfw <width X >,<width Y>,<width Z> -bfc <center X>,<center Y>,<center Z>] \n\n \
        -t      <feature type>\n \
        -p      <sample consensus preset>\n \
        -pcfg   <sample consensus custom file preset name> (DEBUG only)\n \
        -s      <scale> scale size\n \
        -x      <rotation x-axis [deg]>\n \
        -y      <rotation y-axis [deg]>\n \
        -z      <rotation z-axis [deg]>\n \
        -v      show viewer\n \
        -bf     <cube length> cube remove filter (origin: 0,0,0) - after transformation\n \
        -bfw    <width X >,<width Y>,<width Z> cuboid remove filter width (requires -bfc) - after transformation\n \
        -bfc    <center X>,<center Y>,<center Z> cuboid remove filter center (requires -bfw) - after transformation\n";
```

#### Example for pre-processing of the LiDAR point cloud dataset
```console
./14_lidar_preprocessing ../../files/20231121_fhtw_hallway_groundplane_tf.pcd -t fpfh -p fpfh_5_fpfh_fgr_gpu
```

## 4.2 Static Research Scenario

The following programs are executed at the static research scenario.

### 4.2.1 Data Acquisition from RealSense Camera: Program 03_data_aquisition/01_RealSense_Capture_PCD_RGB_BAG.cpp

The data of the RealSense cameras can be acquired for later usage and research. The camera models D415 and D455 are supported. The program captures an image frame, displays it and waits for user interaction to store the current frame (keystroke `s`) or capture the next frame (any other key). Keystroke `x` exits the program. The `-p [period in s]` option starts a timer-triggered automatic data capturing.

The following steps are performed within a loop:
- capture data from the camera
- save .png file from RGB camera
- save .depth file containing depth frame 
- save .bag file containing the raw data from the sensor
- apply post-processing filters of RS2 library
- convert image to point cloud dataset
- (optional) save the point cloud data of the original depth image to a file
- (optional) save the point cloud data of the filtered depth image to a file
- (optional) when exiting, the last frame is displayed in a 3D point cloud viewer

The program is assumed to be executed from the build directory for file storage in `../../files/`.

```console
Usage: 
        ./01_RealSense_Capture_PCD_RGB_BAG <filename.pcd> [-v] [-s] [-hl] [-p <period in s>] [-rsp <rs2 preset name>]

        filename:         filename to store data to - files are stored to ../../files/ 
        -s:               save .pcd files (optional) 
        -hl:              headless mode: do not display capture window (optional) 
        -rsp:             use RS2 filter preset (optional)
        -p <period in s>: automatic mode: capture every given time interval (optional) 
                          CAUTION: if capture window is shown, timing might be incorrect at small periods. 
                                   use headless mode for better timing [-hl] 
        -v:               open Point Cloud Viewer on last captured data (optional) 

example:  ./01_RealSense_Capture_PCD_RGB_BAG -s -hl -p 0.3 -rsp p5a 20241020_fhtw_stairs_p5a_.pcd
```

### 4.2.2 Visualization: Program 02_correspondence_estimation/08_draw_position_history.cpp 

To visualize the correspondence estimations, the localized poses and orientations can be displayed in a 3D viewer:
- The pose is displayed as dot and the number of the dataset
- The orientation is displayed as a small coordinate frame
- The path between the datasets is displayed as a green line between the estimated positions.
- The LiDAR 3D model is displayed as a black point cloud. 
- (optional) The corresponding source point clouds are displayed in different colors matching the pose numbers. 

The program requires a file list containing the names of the data. This list is described in detail in the usage advice. The example dataset contains a file list to display the positive correspondences.

#### Usage
```console
Display position history using following data: 
        - Destination pointcloud (3D-Model) 
        - Transformation matrix of every matching point to display 
        - Pointcloud corresponding to the transformation matrix (optional) 

Usage: ./08_draw_position_history <fileList> [-p] [-fpfh|-narf] 
        <fileList>    ... ASCII file containing source files to display - file extension must be .flist (details below) 
        [-p]          ... Display corresponding transformed pointcloud (optional; if activated, .pcd data must be present) 
        [-fpfh|-narf] ... Set feature type (NARF or FPFH) 

<fileList> data format (file extension: .flist): 
        1st line: 
                fileName.flist of 3D-Modell without path and including file extension 
                path must be ../ relative to <fileList>; file extention must be .pcd 
        2nd to nth line: 
                file name of matching point transformation matrix without path, without _tfmatrix appendix and without file extension: 
                    path must be ../feature_desc/ relative to <fileList>; 
                    NARF features: file extention must be _tfmatrix.Affine3f 
                    FPFH features: file extention must be _PointNormal_tfmatrix.Affine3f 
                corresponding PCD file (in case of [-p]): path must be ../ relative to <fileList>; file extention must be .pcd 

        EXAMPLE: 
                3dmodel.pcd 
                point1 
                point2 
                point3 
```

#### Example usage
```console
.08_draw_position_history ../../files/file_lists/file_list.flist -fpfh
```


### 4.2.3 Manual Computation NARF: Key Points and Correspondence Estimation

The next section describes how to manually execute the correspondence estimation steps. After successful estimation, a path of the localizations can be drawn in a 3D viewer as described in Section 4.5. This section describes the usage of NARF key points. The required dataset is generated as described in section 4.2. Alternatively, the example dataset might be used. As the research has shown, NARF key points are not very reliable. Therefore, the datasets might not match. 


### 4.2.3.1 NARF Correspondence Estimation: Program 02_correspondence_estimation/01_narf_corr.cpp

The program 01_narf_corr executes the correspondence estimation by matching the NARF key points and descriptors. It requires the pre-computed LiDAR 3D model computed in Section 4.1.1 as the target/destination dataset and the captured point normal PCD as described in 4.2.1 as the source dataset. The pose and orientation are estimated using a RANSAC algorithm. If successful, the program prints the transformation data to the console and writes a raw file containing the transformation matrix to `feature_desc/<source_filename_without_extension>_tfmatrix.Affine3f` relative to the source file path. The program requires the `_XYZ.pcd` and `_narf36.pcd` in the folder `feature_desc/` relative to the source and target files.

```console
Usage: ./01_narf_corr <src cloud [.pcd]> <dest cloud [.pcd]> [-v]

Options: 
      -v display 3D viewer  
```

The resulting `_tfmatrix.Affine3f` file can be used to display a path with the program described in Section 4.2.2.

### 4.2.3.2 Combination of RealSense image capture and NARF feature extraction

`01_feature_ext/04_realsense_and_narf_feature_extraction.cpp`


### 4.2.4 Manual Computation: ISS Key Points: Program 01_feature_ext/07_iss_keypoints_from_file.cpp

This program was used to calculate and display the ISS key points.

```console
Usage: ./07_iss_keypoints_from_file.cpp [filename] [extraction resolution]

        - [filename] (default: ../../files/coor_cap_rgb.pcd)"
        - [extraction resolution] <double> (default: 0.11)
```

### 4.2.5 Manual Computation FPFH: Correspondence Estimation: Program 01_feature_ext/15_fpfh_registration.cpp

This program was used for the FPFH feature descriptor and correspondence estimation using the PCR presets and C-Presets of the ILPCR library

```console
Usage: ./15_fpfh_registration <dest cloud [.pcd]> <src cloud [.pcd]> [-v] [-log] [-p presetName | -pcfg presetFile]

        - <dest cloud [.pcd]>: 3d model point cloud file (see 4.1)
        - <src cloud [.pcd]>:  RGB-D capture PCD (see 4.2.1)
        - [-v]:                enable viewer (optional)
        - [-log]:              write log data to file ../log/15_fpfh_registration.csv
        - [-p presetName | -pcfg presetFile]: ILPCR preset to be executed
```

#### Example Usage
```console
./15_fpfh_registration <target_file> <source_file_tmp> -p fpfh_5_fpfh_fgr_gpu -log
```

#### Bash Scripts for Multiple Multi-Threaded Estimations

The scripts in `01_feature_ext/scripts/` compute the FPFH correspondence estimations as selected. The script is designed to be executed from the build directory. A link to the executable of 02_correspondence_estimation/08_draw_position_history.cpp (Section 4.2.2) is required in the `01_feature_ext/build/` directory.


## 4.3 Real-Time Scenario

The following programs are executed at the real-time scenario.

### 4.3.1 Pose Estimation Loop: Program 05_pipeline/01_pipeline.cpp

This program is the implementation of the real-time scenario as described in the thesis.

```console
usage: ./01_pipeline <3D model [.pcd]> <-t <fpfh|narf>> <-p <preset name>> [-v] [-s] [-rsp <rs2 preset name>]

        -t <fpfh|narf>: feature type
        -p <preset name>: \ac{PCR} preset name
        -v: open viewer (optional)
        -s: save files (optional) - files are stored to ../../files/
        -rsp: use RS2 filter preset (optional)
```

#### Example command for executing the real-time scenario capturing
```console
./01_pipeline ../../files/feature_desc/20231121_fhtw_hallway_groundplane_tf_fpfh_5_fpfh_fgr_gpu.pcd -t fpfh -p fpfh_5_fpfh_fgr_gpu -rsp p5a
```

### 4.3.2 Live Visualization: Program 03_data_aquisition/07_draw_position_stream.cpp

Implementation of the live visualization application as described in the thesis. 

```console
Usage: ./07_draw_position_stream <3D model PCD [.pcd]> [server ip address]

        - [server ip adderss]:  IP address of the server (e.g. pipeline)
                                default: localhost (127.0.0.1)
```

### 4.3.3 Live Logfile Playback: Program 05_pipeline/09_stream_logged_data.cpp

The application opens a logfile stored with 4.3.1 for replaying. It opens a TCP/IP server to stream the data to the 4.3.2 live visualization application.

```console
usage: ./09_stream_logged_data <log filename> [-nowait]"
                <log filename>  filename of binary logfile"
                -nowait         don't replay temporal data"
```

### 4.3.4 Live Logfile Playback: Program 05_pipeline/02_convert_log.cpp

This program converts a binary log-file data acquired with the real-time application pose estimation loop (pipeline) to an ASCII CSV file.

```console
usage: ./02_convert_log <log filename> [-conv]" \
                <log filename>  filename of binary logfile" \
                [-conv]         only convert successfully alligned data (optional)" \
       output file is inputfile.csv (without original file extension)"
```
