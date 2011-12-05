cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd2txt)

find_package(PCL 1.3 REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd2txt src/pcd2txt.cpp)
add_executable (statistical_removal src/statistical_removal.cpp)
add_executable (ply2pcd src/ply2pcd.cpp)
add_executable (visualize src/visualize.cpp)
add_executable (resampling src/resampling.cpp)
add_executable (test_surface src/test_surface.cpp)

target_link_libraries (pcd2txt ${PCL_LIBRARIES})
target_link_libraries (statistical_removal ${PCL_LIBRARIES})
target_link_libraries (ply2pcd ${PCL_LIBRARIES})
target_link_libraries (visualize ${PCL_LIBRARIES})
target_link_libraries (resampling ${PCL_LIBRARIES})
target_link_libraries (test_surface ${PCL_LIBRARIES})
