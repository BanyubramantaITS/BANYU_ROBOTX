cmake_minimum_required(VERSION 3.8)
project(banyu_vehicle_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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

add_library(esp32_bridge_lib src/esp32_bridge_node.cpp)
target_compile_features(esp32_bridge_lib PUBLIC cxx_std_17)
target_include_directories(esp32_bridge_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(esp32_bridge_lib
  rclcpp
  std_msgs)

add_executable(esp32_bridge_node src/esp32_bridge_main.cpp)
target_compile_features(esp32_bridge_node PUBLIC cxx_std_17)
target_link_libraries(esp32_bridge_node esp32_bridge_lib)
ament_target_dependencies(esp32_bridge_node
  rclcpp
  std_msgs)

add_library(esp32_trigger_node src/esp32_trigger_node.cpp)
target_compile_features(esp32_trigger_node PUBLIC cxx_std_17)
target_include_directories(esp32_trigger_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(esp32_trigger_node
  rclcpp
  std_msgs
  std_srvs
  banyu_msgs)

add_executable(esp32_trigger_node_bin src/esp32_trigger_main.cpp)
set_target_properties(esp32_trigger_node_bin PROPERTIES OUTPUT_NAME "esp32_trigger_node")
target_compile_features(esp32_trigger_node_bin PUBLIC cxx_std_17)
target_link_libraries(esp32_trigger_node_bin esp32_trigger_node)
ament_target_dependencies(esp32_trigger_node_bin rclcpp)

add_executable(esp32_serial_node_bin src/esp32_trigger_main.cpp)
set_target_properties(esp32_serial_node_bin PROPERTIES OUTPUT_NAME "esp32_serial_node")
target_compile_features(esp32_serial_node_bin PUBLIC cxx_std_17)
target_link_libraries(esp32_serial_node_bin esp32_trigger_node)
ament_target_dependencies(esp32_serial_node_bin rclcpp)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS
  vehicle_status_tracker
  vehicle_status_node
  esp32_bridge_lib
  esp32_bridge_node
  esp32_trigger_node
  esp32_trigger_node_bin
  esp32_serial_node_bin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs std_srvs banyu_msgs mavros_msgs sensor_msgs)

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

  ament_add_gtest(test_esp32_bridge_node test/test_esp32_bridge_node.cpp)
  target_link_libraries(test_esp32_bridge_node esp32_bridge_lib)
  target_compile_features(test_esp32_bridge_node PUBLIC cxx_std_17)
  target_include_directories(test_esp32_bridge_node PRIVATE include)
  ament_target_dependencies(test_esp32_bridge_node
    rclcpp
    std_msgs)

  ament_add_gtest(test_esp32_trigger_node test/test_esp32_trigger_node.cpp)
  target_link_libraries(test_esp32_trigger_node esp32_trigger_node)
  target_compile_features(test_esp32_trigger_node PUBLIC cxx_std_17)
  target_include_directories(test_esp32_trigger_node PRIVATE include)
  ament_target_dependencies(test_esp32_trigger_node
    rclcpp
    std_msgs
    std_srvs
    banyu_msgs)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
