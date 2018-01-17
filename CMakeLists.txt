cmake_minimum_required(VERSION 3.6)
project(Jigsaw)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp filter/Point_filter.cpp filter/Point_filter.h configuration/config.h keypoint/keypoint.cpp keypoint/keypoint.h featrue_align/FeatrueAlign.cpp featrue_align/FeatrueAlign.h io/LoadData.cpp io/LoadData.h ICP_Refine/ICPRefine.cpp ICP_Refine/ICPRefine.h utils/RotatePointCloud.cpp utils/RotatePointCloud.h)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(Jigsaw ${SOURCE_FILES})
target_link_libraries(Jigsaw ${PCL_LIBRARIES})