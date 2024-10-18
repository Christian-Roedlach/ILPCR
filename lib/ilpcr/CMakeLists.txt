cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(OpenGL_GL_PREFERENCE GLVND)

project(ilpcr_library)

if(NOT USE_ILPRC_CUPOCH_FEATURES STREQUAL "yes")
    message("USE_ILPRC_CUPOCH_FEATURES disabled")
else()
    message("USE_ILPRC_CUPOCH_FEATURES enabled")
endif()

if(NOT USE_IPLCR_PCL_GPU_FEATURES STREQUAL "yes")
    message("USE_IPLCR_PCL_GPU_FEATURES disabled")
else()
    message("USE_IPLCR_PCL_GPU_FEATURES enabled")
endif()

#message("USE_ILPRC_CUPOCH_FEATURES = ${USE_ILPRC_CUPOCH_FEATURES}")

find_package(PCL 1.14 REQUIRED)
find_package(realsense2 2.54 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(realsense2-gl REQUIRED)
find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)


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
    include(Cmakelist_CUPOCH_include.txt)

    include_directories(SYSTEM ${CUPOCH_3RDPARTY_INCLUDE_DIRS})
    include_directories(SYSTEM ${CUPOCH_SRC_ROOT}/src)
    include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
    add_definitions(${CUPOCH_DEFINITIONS})
    link_directories(${CUPOCH_LIBRARY_DIRS})
    link_libraries(${CUPOCH_LIBRARIES})
    link_libraries(${CUPOCH_3RDPARTY_LIBRARIES})
    # CUPOCH END
endif()

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${OPENGL_INCLUDE_DIRS})
#include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library (ilpcr STATIC)

target_sources(ilpcr
    PRIVATE 
        src/feature_estimation.cpp
        src/file_io.cpp
        src/camera.cpp
        src/corr_estimation.cpp
        src/viewer.cpp
        src/network.cpp
        src/testdata.cpp
        src/filters.cpp
        src/log.cpp
        src/presets.cpp
        src/ilpcrCupoch.cpp
)

target_link_libraries (ilpcr 
    ${PCL_LIBRARIES}
    ${realsense2_LIBRARY}
    ${CUPOCH_LIBRARIES}
    OpenMP::OpenMP_CXX
)

target_compile_options(ilpcr PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(ilpcr
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/ilpcr
)

#enable PCL_GPU (normals and fpfh)
if(USE_IPLCR_PCL_GPU_FEATURES STREQUAL  "yes")
    message("USING PCL GPU FEATURES (NEST and FPFH)")
    target_compile_definitions(ilpcr PRIVATE PCL_GPU_ENABLED=1)
    set(ILPCR_CUPOCH_GPU_FEATURES_DEFINE "PCL_GPU_ENABLED=1" PARENT_SCOPE)
else()
    message("NOT USING PCL GPU FEATURES (NEST and FPFH)")
endif()


#enable CUPOCH
if(USE_ILPRC_CUPOCH_FEATURES STREQUAL  "yes")
    message("USING ILPCR CUPOCH FEATURES")
    target_compile_definitions(ilpcr PRIVATE CUPOCH_ENABLED=1)
    set(ILPCR_CUPOCH_ENABLE_DEFINE "CUPOCH_ENABLED=1" PARENT_SCOPE)
else()
    message("NOT USING ILPCR CUPOCH FEATURES")
endif()

#set_target_properties(ilpcr PROPERTIES
#                      CUDA_SEPARABLE_COMPILATION ON)

#add_subdirectory(cupoch)