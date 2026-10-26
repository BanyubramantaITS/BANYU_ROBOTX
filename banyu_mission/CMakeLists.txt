cmake_minimum_required(VERSION 3.8)
project(banyu_mission)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(banyu_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rx_msgs REQUIRED)

add_library(banyu_mission_core
  src/run_coordinator.cpp
  src/mission_state.cpp
  src/por_mission_config.cpp
  src/action/acknowledge_red_ring_node.cpp
  src/action/acquire_pipeline_node.cpp
  src/action/align_heading_node.cpp
  src/action/arm_and_set_guided.cpp
  src/action/center_on_ring_node.cpp
  src/action/decode_resource_code_node.cpp
  src/action/dive_to_depth_node.cpp
  src/action/follow_pipeline_node.cpp
  src/action/handle_abort_or_teleop_node.cpp
  src/action/hold_probe_on_ring_node.cpp
  src/action/navigate_surface_gps_node.cpp
  src/action/por_action_nodes.cpp
  src/action/publish_pipeline_survey_node.cpp
  src/action/register_actions.cpp
  src/action/search_pipeline_node.cpp
  src/action/set_fcu_mode_node.cpp
  src/action/surface_node.cpp
  src/condition/autonomy_interrupted_condition.cpp
  src/condition/pipeline_active_condition.cpp
  src/condition/pipeline_ended_condition.cpp
  src/condition/pipeline_lost_condition.cpp
  src/condition/red_ring_pending_condition.cpp
  src/condition/register_conditions.cpp
  src/condition/require_gps_available_condition.cpp
  src/condition/require_mission_started_condition.cpp
  src/condition/require_vehicle_ready_condition.cpp
  src/banyu_mission.cpp)
target_compile_features(banyu_mission_core PUBLIC cxx_std_17)
target_include_directories(banyu_mission_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(banyu_mission_core
  ament_index_cpp banyu_msgs behaviortree_cpp builtin_interfaces
  rclcpp rclcpp_action std_msgs std_srvs mavros_msgs rx_msgs)

add_executable(banyu_mission src/main.cpp)
target_compile_features(banyu_mission PUBLIC cxx_std_17)
target_link_libraries(banyu_mission banyu_mission_core)
ament_target_dependencies(banyu_mission ament_index_cpp rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS banyu_mission_core banyu_mission
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY bt DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(banyu_mission_core)
ament_export_dependencies(
  ament_index_cpp banyu_msgs behaviortree_cpp builtin_interfaces
  rclcpp rclcpp_action std_msgs std_srvs mavros_msgs rx_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_run_coordinator test/test_run_coordinator.cpp)
  target_link_libraries(test_run_coordinator banyu_mission_core)
  ament_add_gtest(test_run_lifecycle test/test_run_lifecycle.cpp ENV ROS_DOMAIN_ID=42)
  target_link_libraries(test_run_lifecycle banyu_mission_core)
  ament_target_dependencies(test_run_lifecycle rclcpp rclcpp_action banyu_msgs mavros_msgs rx_msgs)
  ament_add_gtest(test_mission_tree test/test_mission_tree.cpp ENV ROS_DOMAIN_ID=42)
  target_compile_features(test_mission_tree PRIVATE cxx_std_17)
  target_compile_definitions(test_mission_tree PRIVATE
    BANYU_MISSION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_mission_tree banyu_mission_core)
  ament_target_dependencies(test_mission_tree ament_index_cpp rclcpp)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
