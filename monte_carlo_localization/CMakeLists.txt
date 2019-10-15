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

# add_library(${PROJECT_NAME}_lib)
add_executable(particle_filter_node src/particle_filter_node.cpp src/particle_filter.cpp src/particles.cpp)
target_link_libraries(particle_filter_node
  yaml-cpp
  ${catkin_LIBRARIES}
  # ${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
)

add_executable(test_particles_publisher src/test_particles_pub_main.cpp src/particles.cpp)
target_link_libraries(test_particles_publisher
  ${catkin_LIBRARIES}
)

add_executable(test_move_particles src/test_move_particles.cpp src/particles.cpp)
target_link_libraries(test_move_particles
  ${catkin_LIBRARIES}
)

add_executable(test_scan_wall src/test_laser_wall_pub_main.cpp src/particles.cpp src/particle_filter.cpp)
target_link_libraries(test_scan_wall
  yaml-cpp
  ${catkin_LIBRARIES}
)

# add_executable(test_csv_reader src/test_CSV_main.cpp)
# target_link_libraries(test_csv_reader
#   ${catkin_LIBRARIES}
#   # ${PROJECT_NAME}_lib
# )

# add_executable(test_magPublisher src/test_magPub_main.cpp)
# target_link_libraries(test_magPublisher
#   ${catkin_LIBRARIES}
#   # ${PROJECT_NAME}_lib
# )

# add_executable(test_LaserPublisher src/test_laser_map_main.cpp)
# target_link_libraries(test_LaserPublisher
#   yaml-cpp
#   ${catkin_LIBRARIES}
#   # ${PROJECT_NAME}_lib
# )

# add_executable(test_LaserLikelihoodMap src/test_laser_likelihood_map_main.cpp)
# target_link_libraries(test_LaserLikelihoodMap
#   yaml-cpp
#   ${catkin_LIBRARIES}
#   # ${PROJECT_NAME}_lib
# )

# add_executable(test_Publisher src/test_laser_likelihood_map_pub_main.cpp)
# target_link_libraries(test_Publisher
#   yaml-cpp
#   ${catkin_LIBRARIES}
#   # ${PROJECT_NAME}_lib
# )

