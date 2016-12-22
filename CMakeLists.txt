cmake_minimum_required(VERSION 2.8.3)
project(pacmod)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  can_interface
  globe_epas
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pacmod_cmd.msg
  global_rpt.msg
  system_rpt_int.msg
  system_rpt_float.msg
  motor_rpt_1.msg
  motor_rpt_2.msg
  motor_rpt_3.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
#  LIBRARIES globe_epas
  CATKIN_DEPENDS roscpp std_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_definitions(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/${PROJECT_NAME}
    core_pacmod/include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pacmod_node src/pacmod_node.cpp core_pacmod/src/pacmod_core.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pacmod_node pacmod_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pacmod_node
  ${catkin_LIBRARIES}
)
