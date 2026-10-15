cmake_minimum_required(VERSION 3.8)
project(banyu_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(banyu_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(vehicle_status_tracker src/vehicle_status_tracker.cpp)
target_compile_features(vehicle_status_tracker PUBLIC cxx_std_17)
target_include_directories(vehicle_status_tracker PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(vehicle_status_tracker
  rclcpp
  banyu_msgs
  mavros_msgs
  sensor_msgs)

add_executable(vehicle_status_node src/vehicle_status_node.cpp)
target_compile_features(vehicle_status_node PUBLIC cxx_std_17)
target_link_libraries(vehicle_status_node vehicle_status_tracker)
ament_target_dependencies(vehicle_status_node rclcpp)

add_executable(esp32_bridge_node src/esp32_bridge_node.cpp)
target_compile_features(esp32_bridge_node PUBLIC cxx_std_17)
target_include_directories(esp32_bridge_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(esp32_bridge_node
  rclcpp
  std_msgs)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS vehicle_status_tracker vehicle_status_node esp32_bridge_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs banyu_msgs mavros_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vehicle_status_tracker test/test_vehicle_status_tracker.cpp)
  target_link_libraries(test_vehicle_status_tracker vehicle_status_tracker)
  target_compile_features(test_vehicle_status_tracker PUBLIC cxx_std_17)
  target_include_directories(test_vehicle_status_tracker PRIVATE include)
  ament_target_dependencies(test_vehicle_status_tracker
    rclcpp
    banyu_msgs
    mavros_msgs
    sensor_msgs)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
