cmake_minimum_required(VERSION 3.8)
project(banyu_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(banyu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_executable(banyu_mission
  src/banyu_mission.cpp
  src/bt_nodes/common.cpp
  src/bt_nodes/aborted_or_teleop.cpp
  src/bt_nodes/gate_align.cpp
  src/bt_nodes/gate_pass.cpp
  src/bt_nodes/marker_align.cpp
  src/bt_nodes/marker_orbit.cpp
  src/bt_nodes/mission_active.cpp
  src/bt_nodes/nav_to_gate.cpp
  src/bt_nodes/return_gate_pass.cpp
  src/bt_nodes/search_gate.cpp
  src/bt_nodes/search_marker.cpp
  src/bt_nodes/surface.cpp
  src/bt_nodes/task2_common.cpp
  src/bt_nodes/task2_preflight.cpp
  src/bt_nodes/task2_source.cpp
  src/bt_nodes/task2_camera_preflight.cpp
  src/bt_nodes/acquire_task2_pipeline.cpp
  src/bt_nodes/follow_task2_pipeline.cpp
  src/bt_nodes/follow_and_record_task2_pipeline.cpp
  src/bt_nodes/reverse_to_red_ring.cpp
  src/bt_nodes/repair_red_ring.cpp
  src/bt_nodes/surface_at_red_target.cpp
  src/bt_nodes/task2_report.cpp
  src/bt_nodes/surface_acquire_gps_anchor.cpp
  src/bt_nodes/home_to_start_buoy.cpp
  src/bt_nodes/turn_around.cpp
  src/bt_nodes/visual_align.cpp
)
target_include_directories(banyu_mission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(banyu_mission
  rclcpp
  behaviortree_cpp
  std_srvs
  mavros_msgs
  geometry_msgs
  ament_index_cpp
  banyu_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  unique_identifier_msgs
)

install(TARGETS banyu_mission
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY bt
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
