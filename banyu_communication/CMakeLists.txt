cmake_minimum_required(VERSION 3.8)
project(banyu_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(banyu_msgs REQUIRED)

add_library(report_state_machine src/report_state_machine.cpp)
target_compile_features(report_state_machine PUBLIC cxx_std_17)
target_include_directories(report_state_machine PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(report_state_machine banyu_msgs)

add_executable(uav_report_node src/uav_report_node.cpp)
target_compile_features(uav_report_node PUBLIC cxx_std_17)
target_link_libraries(uav_report_node report_state_machine)
ament_target_dependencies(uav_report_node rclcpp rclcpp_action banyu_msgs)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS report_state_machine uav_report_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action banyu_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_report_state_machine test/test_report_state_machine.cpp)
  target_link_libraries(test_report_state_machine report_state_machine)
  target_compile_features(test_report_state_machine PUBLIC cxx_std_17)
  target_include_directories(test_report_state_machine PRIVATE include)
  ament_target_dependencies(test_report_state_machine banyu_msgs)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
