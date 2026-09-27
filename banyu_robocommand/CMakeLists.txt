cmake_minimum_required(VERSION 3.8)
project(banyu_robocommand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(banyu_msgs REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_library(rx_report STATIC src/rx_report.cpp)
target_include_directories(rx_report PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(rx_report banyu_msgs)

add_executable(report_bridge src/report_bridge_node.cpp)
target_link_libraries(report_bridge rx_report PahoMqttCpp::paho-mqttpp3)
ament_target_dependencies(report_bridge rclcpp banyu_msgs)

install(TARGETS report_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rx_report test/test_rx_report.cpp)
  target_link_libraries(test_rx_report rx_report)
endif()

ament_package()
