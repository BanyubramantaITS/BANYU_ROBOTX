cmake_minimum_required(VERSION 3.8)
project(banyu_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Build against ROS' vendored Gazebo, the one `gz sim` and ros_gz run. Without
# these, CMake can pick up a system gz-harmonic install instead, and the plugins
# then load a second Ogre-Next next to the running one (mesh errors, NaN asserts).
find_package(gz_sim_vendor REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_ogre_next_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-rendering8 REQUIRED COMPONENTS ogre2)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(mavlink REQUIRED)
# gz-rendering's ogre2 target does not export the OGRE-Next headers its own headers include.
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE2 REQUIRED IMPORTED_TARGET OGRE-Next)

# World systems for the RobotX course, loaded by filename from the world SDF.
add_library(BanyuHydrophoneArray SHARED src/HydrophoneArray.cc)
target_link_libraries(BanyuHydrophoneArray gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
add_library(BanyuPipelineLights SHARED src/PipelineLights.cc)
target_link_libraries(BanyuPipelineLights gz-sim8::gz-sim8 gz-plugin2::register)

# DVL -> ArduSub SITL. Header-only MAVLink C library from the ROS mavlink package.
add_library(BanyuDvlMavlink SHARED src/DvlMavlink.cc)
target_link_libraries(BanyuDvlMavlink
  gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
target_include_directories(BanyuDvlMavlink PRIVATE ${mavlink_INCLUDE_DIRS}/mavlink/v2.0)
# Generated MAVLink headers trip these; they are not our code.
target_compile_options(BanyuDvlMavlink PRIVATE -Wno-address-of-packed-member -Wno-pedantic)

add_library(BanyuUnderwaterTurbidity SHARED src/UnderwaterTurbidity.cc)
target_link_libraries(BanyuUnderwaterTurbidity
  gz-sim8::gz-sim8 gz-plugin2::register gz-rendering8::gz-rendering8-ogre2 PkgConfig::OGRE2)

# gz-rendering dlopens ogre2 from its engine-plugins dir, a second file with the
# same soname as the one in libdir. Resolve that copy, or the pass would live in
# a different Ogre2RenderEngine singleton than the one actually rendering.
get_target_property(_ogre2_lib gz-rendering8::gz-rendering8-ogre2 LOCATION)
get_filename_component(_gz_libdir "${_ogre2_lib}" DIRECTORY)
set(_engine_dir "${_gz_libdir}/gz-rendering-8/engine-plugins")
set_target_properties(BanyuUnderwaterTurbidity PROPERTIES
  BUILD_RPATH "${_engine_dir}" INSTALL_RPATH "${_engine_dir}")
target_compile_definitions(BanyuUnderwaterTurbidity PRIVATE OGRE_IGNORE_UNKNOWN_DEBUG)
target_include_directories(BanyuUnderwaterTurbidity PRIVATE
  ${OGRE2_INCLUDEDIR}/OGRE-Next/Hlms/Common ${OGRE2_INCLUDEDIR}/OGRE-Next/Hlms/Pbs
  ${OGRE2_INCLUDEDIR}/OGRE-Next/Hlms/Unlit)

install(TARGETS BanyuHydrophoneArray BanyuPipelineLights BanyuDvlMavlink BanyuUnderwaterTurbidity DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gazebo_env.dsv")

install(
    DIRECTORY models worlds
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_acoustic_shadow test/test_acoustic_shadow.cpp)
  target_link_libraries(test_acoustic_shadow gz-math7::gz-math7)
endif()

ament_package()
