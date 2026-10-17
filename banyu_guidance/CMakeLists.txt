cmake_minimum_required(VERSION 3.8)
project(banyu_guidance)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(banyu_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VelocityIntent.msg"
  DEPENDENCIES std_msgs
)

add_library(pipeline_follower src/pipeline_follower.cpp)
target_compile_features(pipeline_follower PUBLIC cxx_std_17)
target_include_directories(pipeline_follower PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

add_executable(pipeline_guidance_node src/pipeline_guidance_node.cpp)
target_compile_features(pipeline_guidance_node PUBLIC cxx_std_17)
target_link_libraries(pipeline_guidance_node pipeline_follower)
ament_target_dependencies(pipeline_guidance_node rclcpp banyu_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pipeline_guidance_node "${cpp_typesupport_target}")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pipeline_follower pipeline_guidance_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pipeline_follower test/test_pipeline_follower.cpp)
  target_link_libraries(test_pipeline_follower pipeline_follower)
  target_compile_features(test_pipeline_follower PUBLIC cxx_std_17)
  target_include_directories(test_pipeline_follower PRIVATE include)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Humble's xmllint wrapper hangs when asked to emit its xunit report here;
  # package.xml is checked separately in the package validation command.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
