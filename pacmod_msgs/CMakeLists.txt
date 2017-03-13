cmake_minimum_required(VERSION 2.8.3)
project(pacmod_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PacmodCmd.msg
  GlobalRpt.msg
  SystemRptInt.msg
  SystemRptFloat.msg
  MotorRpt1.msg
  MotorRpt2.msg
  MotorRpt3.msg
  PositionWithSpeed.msg
  VehicleSpeedRpt.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
)
