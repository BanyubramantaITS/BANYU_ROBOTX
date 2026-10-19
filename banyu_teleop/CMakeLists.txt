cmake_minimum_required(VERSION 3.8)
project(banyu_teleop)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)

add_library(banyu_teleop_lib
  src/banyu_teleop.cpp
)
target_include_directories(banyu_teleop_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(banyu_teleop_lib
  rclcpp
  mavros_msgs
  std_msgs
)
target_link_libraries(banyu_teleop_lib Boost::boost)

add_executable(banyu_teleop src/banyu_teleop_main.cpp)
target_link_libraries(banyu_teleop banyu_teleop_lib)

install(TARGETS banyu_teleop banyu_teleop_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_banyu_teleop test/test_banyu_teleop.cpp)
  target_link_libraries(test_banyu_teleop banyu_teleop_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(banyu_teleop_lib)
ament_export_dependencies(rclcpp mavros_msgs std_msgs)
ament_package()
