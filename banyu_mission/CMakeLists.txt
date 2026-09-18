cmake_minimum_required(VERSION 3.8)
project(banyu_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(banyu_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(banyu_mission_core src/banyu_mission.cpp)
target_compile_features(banyu_mission_core PUBLIC cxx_std_17)
target_include_directories(banyu_mission_core PUBLIC include)
ament_target_dependencies(banyu_mission_core
  ament_index_cpp banyu_msgs behaviortree_cpp builtin_interfaces
  rclcpp rclcpp_action std_msgs std_srvs)

add_executable(banyu_mission src/main.cpp)
target_compile_features(banyu_mission PUBLIC cxx_std_17)
target_link_libraries(banyu_mission banyu_mission_core)
ament_target_dependencies(banyu_mission ament_index_cpp rclcpp)

install(TARGETS banyu_mission_core banyu_mission
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY bt DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_tree test/test_mission_tree.cpp)
  target_compile_features(test_mission_tree PRIVATE cxx_std_17)
  target_compile_definitions(test_mission_tree PRIVATE
    BANYU_MISSION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_mission_tree banyu_mission_core)
  ament_target_dependencies(test_mission_tree ament_index_cpp rclcpp)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
