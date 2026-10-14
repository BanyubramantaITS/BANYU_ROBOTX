cmake_minimum_required(VERSION 3.8)
project(banyu_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(banyu_msgs REQUIRED)
find_package(rx_msgs REQUIRED)

add_library(banyu_survey_core src/survey_reporting.cpp)
target_compile_features(banyu_survey_core PUBLIC cxx_std_17)
target_include_directories(banyu_survey_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(banyu_survey_core banyu_msgs rx_msgs)

add_executable(uuv_comm_node src/uuv_comm_node.cpp)
target_link_libraries(uuv_comm_node banyu_survey_core)
target_compile_features(uuv_comm_node PUBLIC cxx_std_17)
target_include_directories(uuv_comm_node PRIVATE include)
ament_target_dependencies(uuv_comm_node
  rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs)

install(DIRECTORY include/
  DESTINATION include)
install(TARGETS uuv_comm_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS banyu_survey_core EXPORT export_banyu_survey_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
ament_export_targets(export_banyu_survey_core HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(banyu_mission REQUIRED)
  ament_add_gtest(test_uuv_comm_node
    test/test_uuv_comm_node.cpp src/uuv_comm_node.cpp
    ENV ROS_DOMAIN_ID=42)
  target_compile_definitions(test_uuv_comm_node PRIVATE BANYU_COMMUNICATION_NO_MAIN)
  target_compile_features(test_uuv_comm_node PUBLIC cxx_std_17)
  target_include_directories(test_uuv_comm_node PRIVATE include)
  target_link_libraries(test_uuv_comm_node banyu_survey_core)
  ament_target_dependencies(test_uuv_comm_node
    rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs)

  ament_add_gtest(test_survey_reporting_core test/test_survey_reporting_core.cpp)
  target_link_libraries(test_survey_reporting_core banyu_survey_core)

  ament_add_gtest(test_survey_reporting
    test/test_survey_reporting.cpp src/uuv_comm_node.cpp ENV ROS_DOMAIN_ID=42)
  target_compile_definitions(test_survey_reporting PRIVATE BANYU_COMMUNICATION_NO_MAIN)
  target_compile_features(test_survey_reporting PUBLIC cxx_std_17)
  target_include_directories(test_survey_reporting PRIVATE include)
  target_link_libraries(test_survey_reporting banyu_survey_core ${banyu_mission_LIBRARIES})
  ament_target_dependencies(test_survey_reporting
    rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs banyu_mission)

  ament_add_gtest(test_mission_report_integration
    test/test_mission_report_integration.cpp src/uuv_comm_node.cpp
    TIMEOUT 60
    ENV ROS_DOMAIN_ID=42)
  target_compile_definitions(test_mission_report_integration PRIVATE BANYU_COMMUNICATION_NO_MAIN)
  target_compile_features(test_mission_report_integration PUBLIC cxx_std_17)
  target_include_directories(test_mission_report_integration PRIVATE include)
  target_link_libraries(test_mission_report_integration banyu_survey_core ${banyu_mission_LIBRARIES})
  ament_target_dependencies(test_mission_report_integration
    rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs banyu_mission)


  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
