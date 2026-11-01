cmake_minimum_required(VERSION 3.8)
project(banyu_detection)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

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
find_package(rx_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)

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
ament_target_dependencies(perception_state_machines rclcpp banyu_msgs geometry_msgs std_msgs sensor_msgs mavros_msgs)

# Task 2 vision algorithms and acoustics ported from main_mission_sim
add_library(banyu_task2_perception STATIC
    src/pipeline_vision.cpp
    src/acoustics.cpp
    src/resource_code_decoder.cpp
)
target_include_directories(banyu_task2_perception PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(banyu_task2_perception ${OpenCV_LIBS})
set_target_properties(banyu_task2_perception PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(por_detection src/por_detection.cpp)
ament_target_dependencies(por_detection rclcpp cv_bridge OpenCV sensor_msgs banyu_msgs image_transport std_srvs)
target_include_directories(por_detection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(task2_pipeline_detector src/task2_pipeline_detector.cpp)
target_link_libraries(task2_pipeline_detector banyu_task2_perception)
ament_target_dependencies(task2_pipeline_detector rclcpp cv_bridge OpenCV sensor_msgs banyu_msgs)

add_executable(pinger_localization src/pinger_localization_node.cpp)
target_link_libraries(pinger_localization banyu_task2_perception)
ament_target_dependencies(pinger_localization rclcpp sensor_msgs ros_gz_interfaces banyu_msgs tf2_ros)

add_executable(ring_tracker_node src/ring_tracker_main.cpp)
ament_target_dependencies(ring_tracker_node rclcpp sensor_msgs geometry_msgs banyu_msgs mavros_msgs)
target_link_libraries(ring_tracker_node perception_state_machines)

add_executable(pipeline_end_node src/pipeline_end_main.cpp)
ament_target_dependencies(pipeline_end_node rclcpp std_msgs banyu_msgs)
target_link_libraries(pipeline_end_node perception_state_machines)

add_executable(survey_evidence_node src/survey_evidence_node.cpp)
ament_target_dependencies(survey_evidence_node rclcpp sensor_msgs banyu_msgs rx_msgs)
target_include_directories(survey_evidence_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(resource_code_decoder_node src/resource_code_decoder_node.cpp)
target_link_libraries(resource_code_decoder_node banyu_task2_perception)
ament_target_dependencies(resource_code_decoder_node rclcpp std_srvs banyu_msgs)
target_include_directories(resource_code_decoder_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
    por_detection
    task2_pipeline_detector
    pinger_localization
    ring_tracker_node
    pipeline_end_node
    survey_evidence_node
    resource_code_decoder_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS perception_state_machines banyu_task2_perception
    EXPORT banyu_detectionTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(EXPORT banyu_detectionTargets
    DESTINATION share/${PROJECT_NAME}/cmake
    NAMESPACE banyu_detection::
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(perception_state_machines banyu_task2_perception)
ament_export_dependencies(rclcpp banyu_msgs geometry_msgs std_msgs)
ament_export_targets(banyu_detectionTargets HAS_LIBRARY_TARGET)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pipeline_vision test/test_pipeline_vision.cpp)
  target_link_libraries(test_pipeline_vision banyu_task2_perception)

  ament_add_gtest(test_resource_code_decoder test/test_resource_code_decoder.cpp)
  target_link_libraries(test_resource_code_decoder banyu_task2_perception)

  ament_add_gtest(test_acoustics test/test_acoustics.cpp)
  target_link_libraries(test_acoustics banyu_task2_perception)

  ament_add_gtest(test_pipeline_end_confirmer
      test/test_pipeline_end_confirmer.cpp)
  target_link_libraries(test_pipeline_end_confirmer perception_state_machines)
  ament_target_dependencies(test_pipeline_end_confirmer banyu_msgs)

  ament_add_gtest(test_ring_tracker
      test/test_ring_tracker.cpp)
  target_link_libraries(test_ring_tracker perception_state_machines)
  ament_target_dependencies(test_ring_tracker banyu_msgs geometry_msgs)

  ament_add_gtest(test_dead_reckoner test/test_dead_reckoner.cpp)
  target_include_directories(test_dead_reckoner PRIVATE include)

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

  ament_add_gtest(test_survey_evidence_node
      test/test_survey_evidence_node.cpp src/survey_evidence_node.cpp)
  target_compile_definitions(test_survey_evidence_node PRIVATE BANYU_DETECTION_NO_MAIN)
  target_include_directories(test_survey_evidence_node PRIVATE include)
  ament_target_dependencies(test_survey_evidence_node rclcpp sensor_msgs banyu_msgs rx_msgs)

  ament_add_gtest(test_por_detection
      test/test_por_detection.cpp src/por_detection.cpp)
  target_compile_definitions(test_por_detection PRIVATE BANYU_DETECTION_NO_MAIN)
  target_include_directories(test_por_detection PRIVATE include)
  ament_target_dependencies(test_por_detection rclcpp cv_bridge OpenCV sensor_msgs banyu_msgs image_transport std_srvs)
endif()

ament_package()
