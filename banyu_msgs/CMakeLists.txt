cmake_minimum_required(VERSION 3.8)
project(banyu_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/HydrophoneHeading.msg"
  "msg/PipelineRingState.msg"
  "msg/PipelineObservation.msg"
  "msg/PipelineRingObservation.msg"
  "msg/Task2RingRecord.msg"
  "msg/Task2Report.msg"
  "msg/VehicleStatus.msg"
  "msg/ActionLease.msg"
  "action/NavigateSurfaceGps.action"
  "action/AlignHeading.action"
  "action/DiveToDepth.action"
  "action/AcquirePipeline.action"
  "action/SearchPipeline.action"
  "action/AcquireGate.action"
  "action/SearchMarker.action"
  "action/SearchGate.action"
  "action/AlignGate.action"
  "action/AlignMarker.action"
  "action/PassGate.action"
  "action/OrbitMarker.action"
  "action/TurnAround.action"
  "action/FollowPipeline.action"
  "action/CenterOnRing.action"
  "action/HoldProbeOnRing.action"
  "action/Surface.action"
  "srv/AcknowledgeRingRepair.srv"
  "srv/TriggerPipelineSurvey.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs unique_identifier_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_lease test/test_action_lease.cpp)
  add_dependencies(test_action_lease ${PROJECT_NAME}__cpp)
  target_include_directories(test_action_lease PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")
  ament_target_dependencies(test_action_lease
    builtin_interfaces
    std_msgs
    unique_identifier_msgs)

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
