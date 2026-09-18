cmake_minimum_required(VERSION 3.8)
project(banyu_vehicle_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(banyu_msgs REQUIRED)
find_package(banyu_guidance REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/command_encoder.cpp
  src/motion_intent.cpp
  src/goal_arbiter.cpp
  src/action_servers/acquire_pipeline_server.cpp
  src/action_servers/align_heading_server.cpp
  src/action_servers/center_on_ring_server.cpp
  src/action_servers/dive_to_depth_server.cpp
  src/action_servers/follow_pipeline_server.cpp
  src/action_servers/hold_probe_on_ring_server.cpp
  src/action_servers/navigate_surface_gps_server.cpp
  src/action_servers/search_pipeline_server.cpp
  src/action_servers/surface_server.cpp
  src/vehicle_execution_node.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  banyu_msgs
  banyu_guidance
  mavros_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  unique_identifier_msgs)

add_executable(vehicle_execution_node src/main.cpp)
target_compile_features(vehicle_execution_node PUBLIC cxx_std_17)
target_link_libraries(vehicle_execution_node ${PROJECT_NAME})
ament_target_dependencies(vehicle_execution_node
  rclcpp
  rclcpp_action
  banyu_msgs
  banyu_guidance
  mavros_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  unique_identifier_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS vehicle_execution_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  banyu_msgs
  banyu_guidance
  mavros_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  unique_identifier_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vehicle_execution test/test_vehicle_execution.cpp)
  target_link_libraries(test_vehicle_execution ${PROJECT_NAME})
  target_compile_features(test_vehicle_execution PUBLIC cxx_std_17)
  target_include_directories(test_vehicle_execution PRIVATE include)
  ament_target_dependencies(test_vehicle_execution
    rclcpp
    rclcpp_action
    banyu_msgs
    banyu_guidance
    mavros_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    unique_identifier_msgs)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
