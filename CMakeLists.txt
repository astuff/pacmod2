cmake_minimum_required(VERSION 2.8.3)
project(pacmod)

## Add support for C++11, supported in ROS Kinetic and newer
# For PACMod, this must be enabled to use mutexes.
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pacmod_msgs
  can_msgs
  roslint
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs 
  pacmod_msgs
  can_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}
  src/pacmod_ros_msg_handler.cpp
  src/pacmod_node.cpp
  src/pacmod_core.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

set(ROSLINT_CPP_OPTS "--filter=-build/namespaces,-build/c++11")
roslint_cpp()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
