cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(02_correspondence_estimation)

find_package(PCL 1.14 REQUIRED)

set(USE_ILPRC_CUPOCH_FEATURES "no")
set(USE_IPLCR_PCL_GPU_FEATURES "no")

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
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_compile_definitions(${ILPCR_CUPOCH_ENABLE_DEFINE})
add_compile_definitions(${ILPCR_CUPOCH_GPU_FEATURES_DEFINE})

add_executable (01_narf_corr 01_narf_corr.cpp)
target_link_libraries (01_narf_corr ${PCL_LIBRARIES})

add_executable (08_draw_position_history 08_draw_position_history.cpp)
target_link_libraries (08_draw_position_history ${PCL_LIBRARIES})
target_link_libraries (08_draw_position_history ilpcr)
