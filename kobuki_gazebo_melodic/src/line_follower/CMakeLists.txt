cmake_minimum_required(VERSION 2.8.3)
project(line_follower)
set(line_follower_ROS_DEPS 
  nav_core 
  base_local_planner 
  costmap_2d 
  roscpp
  rospy
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_msgs 
  move_base_msgs
  pluginlib
)

find_package(catkin REQUIRED COMPONENTS ${line_follower_ROS_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line_follower
  CATKIN_DEPENDS ${line_follower_ROS_DEPS}
)

add_library(line_follower src/line_follower.cpp)
target_link_libraries(line_follower ${catkin_LIBRARIES})

install(TARGETS line_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
