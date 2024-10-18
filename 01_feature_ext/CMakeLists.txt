cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(01_feature_ext)

set(USE_ILPRC_CUPOCH_FEATURES "yes")
set(USE_IPLCR_PCL_GPU_FEATURES "yes")

find_package(PCL 1.14 REQUIRED)
find_package(realsense2 2.54 REQUIRED)

# CUPOCH
if(USE_ILPRC_CUPOCH_FEATURES STREQUAL "yes")
    # root dirs
    set(CUPOCH_BUILD_ROOT "~/cupoch/build/")
    set(CUPOCH_SRC_ROOT "~/cupoch/")

    set (CMAKE_CUDA_STANDARD 20)
    enable_language(CUDA)
    set(USE_RMM ON)

    include(CheckLanguage)
    check_language(CUDA)

    find_package(CUDA REQUIRED)
    find_package(OpenMP REQUIRED)

    # include settings required for cupoch library
    include(lib/ilpcr/Cmakelist_CUPOCH_include.txt)

    include_directories(SYSTEM ${CUPOCH_3RDPARTY_INCLUDE_DIRS})
    include_directories(SYSTEM ${CUPOCH_SRC_ROOT}/src)
    include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
    add_definitions(${CUPOCH_DEFINITIONS})
    link_directories(${CUPOCH_LIBRARY_DIRS})
    link_libraries(${CUPOCH_LIBRARIES})
    link_libraries(${CUPOCH_3RDPARTY_LIBRARIES})
    # CUPOCH END
endif()


# Custom library functions
add_subdirectory(lib/ilpcr)
set(ILPCR_INCLUDE_DIRS "lib/ilpcr/include")

include_directories(${ILPCR_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_compile_definitions(${ILPCR_CUPOCH_ENABLE_DEFINE})
add_compile_definitions(${ILPCR_CUPOCH_GPU_FEATURES_DEFINE})

add_executable (02_narf_feature_extraction 02_narf_feature_extraction.cpp)
target_link_libraries (02_narf_feature_extraction ${PCL_LIBRARIES})

add_executable (04_realsense_and_narf_feature_extraction 04_realsense_and_narf_feature_extraction.cpp)
target_link_libraries (04_realsense_and_narf_feature_extraction ${realsense2_LIBRARY} ${PCL_LIBRARIES})

add_executable (07_iss_keypoints_from_file 07_iss_keypoints_from_file.cpp)
target_link_libraries (07_iss_keypoints_from_file ${PCL_LIBRARIES})
target_link_libraries (07_iss_keypoints_from_file ilpcr)

add_executable (14_lidar_preprocessing 14_lidar_preprocessing.cpp)
target_link_libraries (14_lidar_preprocessing ${PCL_LIBRARIES} ilpcr)
target_compile_definitions(14_lidar_preprocessing PRIVATE ${ILPCR_CUPOCH_ENABLE_DEFINE})

add_executable (15_fpfh_registration 15_fpfh_registration.cpp)
target_link_libraries (15_fpfh_registration ${PCL_LIBRARIES} ${realsense2_LIBRARY} ilpcr)
target_compile_options(15_fpfh_registration PRIVATE -Wall -Wextra -Wpedantic) #-Werror
