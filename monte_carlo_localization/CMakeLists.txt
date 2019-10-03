cmake_minimum_required(VERSION 2.8.3)
project(monte_carlo_localization)

set(OpenCV_DIR /usr/share/OpenCV)
set(geometry_msgs_DIR /opt/ros/melodic/share/geometry_msgs/cmake)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  tf
  tf2
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_lib include/${PROJECT_NAME}/particle_filter.h src/particle_filter.cpp)
add_executable(particle_filter_node src/particle_filter_node.cpp)
target_link_libraries(particle_filter_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
)

add_executable(test_csv_reader src/test_CSV_main.cpp)
target_link_libraries(test_csv_reader
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)

