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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Task 2 pipeline survey: one BT node per file (src/task2, include/banyu_mission/task2).
add_library(banyu_task2 STATIC
  src/task2/led_tracker.cpp
  src/task2/flash_decoder.cpp
  src/task2/vehicle_io.cpp
  src/task2/arm_vehicle.cpp
  src/task2/set_flight_mode.cpp
  src/task2/surface_vehicle.cpp
  src/task2/transit_to_pinger.cpp
  src/task2/record_buoy_fix.cpp
  src/task2/find_pipe.cpp
  src/task2/set_survey_depth.cpp
  src/task2/turn_on_pipe.cpp
  src/task2/pinger_forward_at_least.cpp
  src/task2/follow_pipe.cpp
  src/task2/send_report.cpp
  src/task2/probe_servo.cpp
  src/task2/repair_light.cpp
  src/task2/read_flash.cpp
  src/task2/send_resource_request.cpp
)
target_include_directories(banyu_task2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(banyu_task2 rclcpp behaviortree_cpp mavros_msgs sensor_msgs geometry_msgs banyu_msgs)

add_executable(banyu_mission
  src/banyu_mission.cpp
  src/bt_nodes.cpp
)
target_link_libraries(banyu_mission banyu_task2)
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
  std_msgs
  sensor_msgs
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_led_tracker test/test_led_tracker.cpp)
  target_link_libraries(test_led_tracker banyu_task2)
  ament_add_gtest(test_flash_decoder test/test_flash_decoder.cpp)
  target_link_libraries(test_flash_decoder banyu_task2)
endif()

ament_package()
