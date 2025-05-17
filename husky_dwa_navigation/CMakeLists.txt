cmake_minimum_required(VERSION 3.0.2)
project(husky_dwa_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2_ros
  tf2_geometry_msgs
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
  dynamic_reconfigure
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 스크립트 설치 설정 추가
catkin_install_python(
  PROGRAMS 
    scripts/planner_switcher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 기존 설치 설정
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)