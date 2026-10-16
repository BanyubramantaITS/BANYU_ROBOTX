cmake_minimum_required(VERSION 3.8)
project(banyu_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/dvl_mavlink_bridge.py
        scripts/dvl_health.py
        scripts/ekf_source_switch.py
        scripts/task2_sim_provider.py
        scripts/launch_sim.sh
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_flake8
    ament_cmake_pep257
  )
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_task2_sim_provider test/test_task2_sim_provider.py)
endif()

ament_package()
