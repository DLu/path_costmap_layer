cmake_minimum_required(VERSION 2.8.3)
project(path_costmap_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  roscpp
  dynamic_reconfigure
  pluginlib
)

#   geometry_msgs

catkin_package(
CATKIN_DEPENDS costmap_2d dynamic_reconfigure pluginlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

