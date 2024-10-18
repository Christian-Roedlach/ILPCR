cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(OpenGL_GL_PREFERENCE GLVND)

project(03_data_aquisition)

find_package(PCL 1.14 REQUIRED)
find_package(realsense2 2.54 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(realsense2-gl REQUIRED)
find_package(OpenGL REQUIRED)

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

    # Include settings required for the cupoch library
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
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_compile_definitions(${ILPCR_CUPOCH_ENABLE_DEFINE})
add_compile_definitions(${ILPCR_CUPOCH_GPU_FEATURES_DEFINE})

add_executable (01_RealSense_Capture_PCD_RGB_BAG 01_RealSense_Capture_PCD_RGB_BAG.cpp)
target_link_libraries (01_RealSense_Capture_PCD_RGB_BAG ${PCL_LIBRARIES} ${realsense2_LIBRARY} glfw GL GLU) #realsense2-gl
target_link_libraries (01_RealSense_Capture_PCD_RGB_BAG ilpcr)
target_include_directories(01_RealSense_Capture_PCD_RGB_BAG SYSTEM PUBLIC "include_ext/") # prevent warnings from external headers
target_compile_options(01_RealSense_Capture_PCD_RGB_BAG PRIVATE -Wall -Wextra) #-Werror
target_include_directories(01_RealSense_Capture_PCD_RGB_BAG PRIVATE "lib/external/")

add_executable (05_tcp_server_testdata 05_tcp_server_testdata.cpp)
target_link_libraries (05_tcp_server_testdata ${PCL_LIBRARIES})
target_link_libraries (05_tcp_server_testdata ilpcr)
target_compile_options(05_tcp_server_testdata PRIVATE -Wall -Wextra) #-Werror

add_executable (07_draw_position_stream 07_draw_position_stream.cpp)
target_link_libraries (07_draw_position_stream ${PCL_LIBRARIES})
target_link_libraries (07_draw_position_stream ilpcr)
target_compile_options(07_draw_position_stream PRIVATE -Wall -Wextra) #-Werror
target_include_directories(07_draw_position_stream PRIVATE "lib/external/date/include")

add_executable (09_stream_logged_data 09_stream_logged_data.cpp)
target_link_libraries (09_stream_logged_data ${PCL_LIBRARIES})
target_link_libraries (09_stream_logged_data ilpcr)
target_compile_options(09_stream_logged_data PRIVATE -Wall -Wextra) #-Werror
target_include_directories(09_stream_logged_data PRIVATE "lib/external/")
