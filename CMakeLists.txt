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
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(path_costmap_layer src/path_costmap_layer.cpp src/inflater.cpp)
