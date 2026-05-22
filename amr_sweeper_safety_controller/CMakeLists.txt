cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_safety_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_sweeper_safety_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(amr_sweeper_safety_controller_node
  src/amr_sweeper_safety_controller_node.cpp
)
target_include_directories(amr_sweeper_safety_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(amr_sweeper_safety_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(amr_sweeper_safety_controller_node
  amr_sweeper_safety_msgs
  diagnostic_msgs
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS
  amr_sweeper_safety_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  amr_sweeper_safety_msgs
  diagnostic_msgs
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
)

ament_package()
