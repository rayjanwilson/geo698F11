cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd2txt)

find_package(PCL 1.2 REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd2txt src/pcd2txt.cpp)
target_link_libraries (pcd2txt ${PCL_LIBRARIES})
