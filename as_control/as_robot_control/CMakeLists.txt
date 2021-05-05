cmake_minimum_required(VERSION 2.8.3)
project(rr_locomotion)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joint_state_aggregator
  vel_mux
  ros_control_boilerplate
  joint_state_publisher
  robot_state_publisher
  joint_state_controller
  diff_drive_controller
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)