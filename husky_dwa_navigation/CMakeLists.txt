cmake_minimum_required(VERSION 3.0.2)
project(husky_dwa_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base
  dwa_local_planner
  pcl_ros
  nodelet
  pointcloud_to_laserscan
  costmap_2d
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)