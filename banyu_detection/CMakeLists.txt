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
find_package(banyu_msgs REQUIRED)

add_executable(por_detection src/por_detection.cpp)
ament_target_dependencies(por_detection rclcpp cv_bridge OpenCV sensor_msgs banyu_msgs)

target_include_directories(por_detection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(ros_gz_interfaces REQUIRED)

# Task 2: pipeline + light nodes from the bottom camera, pinger from the hydrophones.
add_library(banyu_task2_perception STATIC src/pipeline_vision.cpp src/acoustics.cpp)
target_include_directories(banyu_task2_perception PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(banyu_task2_perception ${OpenCV_LIBS})
set_target_properties(banyu_task2_perception PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(pipeline_detection src/pipeline_detection_node.cpp)
target_link_libraries(pipeline_detection banyu_task2_perception)
ament_target_dependencies(pipeline_detection rclcpp cv_bridge OpenCV sensor_msgs banyu_msgs)

add_executable(pinger_localization src/pinger_localization_node.cpp)
target_link_libraries(pinger_localization banyu_task2_perception)
ament_target_dependencies(pinger_localization rclcpp sensor_msgs ros_gz_interfaces banyu_msgs)

install(TARGETS por_detection pipeline_detection pinger_localization
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
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
  ament_add_gtest(test_pipeline_vision test/test_pipeline_vision.cpp)
  target_link_libraries(test_pipeline_vision banyu_task2_perception)
  ament_add_gtest(test_acoustics test/test_acoustics.cpp)
  target_link_libraries(test_acoustics banyu_task2_perception)
endif()

ament_package()
