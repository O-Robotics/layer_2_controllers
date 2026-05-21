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

add_library(${PROJECT_NAME}_core
  src/attitude_estimator.cpp
  src/stop_supervisor.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  rclcpp
  sensor_msgs
)

add_executable(amr_sweeper_attitude_controller_node
  src/attitude_controller_node.cpp
)
target_include_directories(amr_sweeper_attitude_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(amr_sweeper_attitude_controller_node PUBLIC cxx_std_17)
target_link_libraries(amr_sweeper_attitude_controller_node ${PROJECT_NAME}_core)
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
  ${PROJECT_NAME}_core
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_attitude_estimator test/test_attitude_estimator.cpp)
  target_link_libraries(test_attitude_estimator ${PROJECT_NAME}_core)
  target_include_directories(test_attitude_estimator PRIVATE include)
  ament_target_dependencies(test_attitude_estimator geometry_msgs rclcpp sensor_msgs)

  ament_add_gtest(test_imu_health test/test_imu_health.cpp)
  target_include_directories(test_imu_health PRIVATE include)
  ament_target_dependencies(test_imu_health rclcpp sensor_msgs)

  ament_add_gtest(test_stop_supervisor test/test_stop_supervisor.cpp)
  target_link_libraries(test_stop_supervisor ${PROJECT_NAME}_core)
  target_include_directories(test_stop_supervisor PRIVATE include)
  ament_target_dependencies(test_stop_supervisor geometry_msgs rclcpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  amr_sweeper_safety_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
)

ament_package()
