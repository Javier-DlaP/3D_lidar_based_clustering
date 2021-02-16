cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_add_executable(subscriber src/subscriber.cpp)
target_link_libraries (subscriber ${PCL_LIBRARIES})

rosbuild_add_executable(basic_shapes src/basic_shapes.cpp)
#target_link_libraries(basic_shapes ${catkin_LIBRARIES})