cmake_minimum_required(VERSION 3.0.2)
project(oa)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
  visualization_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES oa
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs tf pcl_ros pcl_conversions visualization_msgs
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(oanode src/oanode.cpp)

target_link_libraries(oanode
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(oanode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
