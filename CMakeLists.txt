cmake_minimum_required(VERSION 2.8.3)
project(husky_pid)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros
  husky_gazebo
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)

catkin_package()
 
install(PROGRAMS
  nodes/pid_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch rviz worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)