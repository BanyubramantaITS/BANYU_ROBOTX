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

add_executable(banyu_teleop src/banyu_teleop.cpp)
target_include_directories(banyu_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(banyu_teleop
  rclcpp
  mavros_msgs
  std_msgs
)
target_link_libraries(banyu_teleop Boost::boost)

install(TARGETS banyu_teleop
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
endif()

ament_package()
