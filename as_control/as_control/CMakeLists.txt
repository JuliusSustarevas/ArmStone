cmake_minimum_required(VERSION 3.0.2)
project(as_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  hardware_interface    
  pluginlib    
  combined_robot_hw
  controller_manager
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    #include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp    
    hardware_interface    
    pluginlib        
    combined_robot_hw
    controller_manager
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(armstone_hw_interface_node
  src/armstone_hw_interface_node.cpp
)

target_link_libraries(armstone_hw_interface_node ${catkin_LIBRARIES})