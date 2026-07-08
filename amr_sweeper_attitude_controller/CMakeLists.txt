cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_attitude_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_sweeper_safety_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(amr_sweeper_attitude_controller_node
  src/attitude_controller_node.cpp
)
set_target_properties(amr_sweeper_attitude_controller_node PROPERTIES OUTPUT_NAME attitude_controller_node)
target_include_directories(amr_sweeper_attitude_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(amr_sweeper_attitude_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(amr_sweeper_attitude_controller_node
  amr_sweeper_safety_msgs
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  amr_sweeper_attitude_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  amr_sweeper_safety_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
)

ament_package()
