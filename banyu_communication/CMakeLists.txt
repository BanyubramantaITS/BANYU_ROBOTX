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

add_executable(uuv_comm_node src/uuv_comm_node.cpp)
target_compile_features(uuv_comm_node PUBLIC cxx_std_17)
target_include_directories(uuv_comm_node PRIVATE include)
ament_target_dependencies(uuv_comm_node
  rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs)

install(DIRECTORY include/
  DESTINATION include)
install(TARGETS uuv_comm_node DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uuv_comm_node
    test/test_uuv_comm_node.cpp src/uuv_comm_node.cpp
    ENV ROS_DOMAIN_ID=42)
  target_compile_definitions(test_uuv_comm_node PRIVATE BANYU_COMMUNICATION_NO_MAIN)
  target_compile_features(test_uuv_comm_node PUBLIC cxx_std_17)
  target_include_directories(test_uuv_comm_node PRIVATE include)
  ament_target_dependencies(test_uuv_comm_node
    rclcpp std_msgs sensor_msgs geometry_msgs std_srvs mavros_msgs banyu_msgs rx_msgs)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
