cmake_minimum_required(VERSION 3.8)
project(banyu_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(banyu_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(perception_state_machines
    src/pipeline_end_confirmer.cpp
    src/pipeline_end_adapter.cpp
    src/pipeline_end_node.cpp
    src/ring_tracker.cpp
    src/ring_tracker_adapter.cpp
    src/ring_tracker_node.cpp
)
target_include_directories(perception_state_machines PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_state_machines rclcpp banyu_msgs geometry_msgs std_msgs)

add_executable(por_detection src/por_detection.cpp)
ament_target_dependencies(por_detection rclcpp cv_bridge OpenCV sensor_msgs banyu_msgs)

target_include_directories(por_detection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(task2_pipeline_detector src/task2_pipeline_detector.cpp)
ament_target_dependencies(task2_pipeline_detector rclcpp cv_bridge OpenCV sensor_msgs banyu_msgs)

add_executable(ring_tracker_node src/ring_tracker_main.cpp)
ament_target_dependencies(ring_tracker_node rclcpp sensor_msgs geometry_msgs banyu_msgs)
target_link_libraries(ring_tracker_node perception_state_machines)

add_executable(pipeline_end_node src/pipeline_end_main.cpp)
ament_target_dependencies(pipeline_end_node rclcpp std_msgs banyu_msgs)
target_link_libraries(pipeline_end_node perception_state_machines)

install(TARGETS por_detection task2_pipeline_detector ring_tracker_node pipeline_end_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS perception_state_machines
    EXPORT perception_state_machinesTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(EXPORT perception_state_machinesTargets
    DESTINATION share/${PROJECT_NAME}/cmake
    NAMESPACE banyu_detection::
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp banyu_msgs geometry_msgs std_msgs)
ament_export_targets(perception_state_machinesTargets HAS_LIBRARY_TARGET)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pipeline_end_confirmer
      test/test_pipeline_end_confirmer.cpp)
  target_link_libraries(test_pipeline_end_confirmer perception_state_machines)
  ament_target_dependencies(test_pipeline_end_confirmer banyu_msgs)

  ament_add_gtest(test_ring_tracker
      test/test_ring_tracker.cpp)
  target_link_libraries(test_ring_tracker perception_state_machines)
  ament_target_dependencies(test_ring_tracker banyu_msgs geometry_msgs)

  ament_add_gtest(test_ring_tracker_adapter
      test/test_ring_tracker_adapter.cpp)
  target_link_libraries(test_ring_tracker_adapter perception_state_machines)
  ament_target_dependencies(test_ring_tracker_adapter banyu_msgs geometry_msgs)

  ament_add_gtest(test_ring_tracker_node_qos
      test/test_ring_tracker_node_qos.cpp)
  target_link_libraries(test_ring_tracker_node_qos perception_state_machines)
  ament_target_dependencies(test_ring_tracker_node_qos rclcpp geometry_msgs banyu_msgs)

  ament_add_gtest(test_run_context_gate test/test_run_context_gate.cpp)
  target_include_directories(test_run_context_gate PRIVATE include)

  ament_add_gtest(test_ring_tracker_reset test/test_ring_tracker_reset.cpp)
  target_link_libraries(test_ring_tracker_reset perception_state_machines)
  ament_target_dependencies(test_ring_tracker_reset rclcpp geometry_msgs banyu_msgs)

  ament_add_gtest(test_pipeline_end_adapter
      test/test_pipeline_end_adapter.cpp)
  target_link_libraries(test_pipeline_end_adapter perception_state_machines)
  ament_target_dependencies(test_pipeline_end_adapter banyu_msgs)

  ament_add_gtest(test_pipeline_end_node
      test/test_pipeline_end_node.cpp)
  target_link_libraries(test_pipeline_end_node perception_state_machines)
  ament_target_dependencies(test_pipeline_end_node rclcpp std_msgs banyu_msgs)
endif()

ament_package()
