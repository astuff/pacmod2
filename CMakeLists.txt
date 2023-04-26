cmake_minimum_required(VERSION 3.5)
project(pacmod2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_msgs REQUIRED)
find_package(pacmod2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include/
)

add_library(
  ${PROJECT_NAME}_node SHARED
  src/pacmod2_ros_msg_handler.cpp
  src/pacmod2_core.cpp
  src/pacmod2_node.cpp
)
target_compile_definitions(
  ${PROJECT_NAME}_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  ${PROJECT_NAME}_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "can_msgs"
  "pacmod2_msgs"
  "std_msgs"
)
rclcpp_components_register_nodes(
  ${PROJECT_NAME}_node
  "pacmod2::PACMod2Node"
)

add_executable(
  ${PROJECT_NAME}_driver
  src/pacmod2_driver.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}_driver
  "rclcpp"
  "lifecycle_msgs"
)
target_link_libraries(
  ${PROJECT_NAME}_driver
  ${PROJECT_NAME}_node
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME}_node HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
