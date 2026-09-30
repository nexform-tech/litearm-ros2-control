cmake_minimum_required(VERSION 3.16)
project(litearm_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# litearm C++ SDK
#
# Preferred: an installed litearm (find_package). If there is none, build the litearm-cpp
# source tree that sits beside this repository in the workspace in-tree. The fallback is
# what makes a plain `colcon build` in the workspace root work with no manual install step
# and no flags.
#
# Why the fallback is needed at all: litearm-cpp has no package.xml, so colcon treats it
# as a plain CMake package named after its directory ("litearm-cpp"), builds it in
# parallel, and never puts its install prefix on this package's CMAKE_PREFIX_PATH. On top
# of that, colcon builds it as a static, non-PIC archive, which a plugin shared object
# cannot link. Building the tree here lets us choose a PIC build.
# ─────────────────────────────────────────────────────────────────────────────
# This package is one directory below the repository root, and the repository is checked
# out beside litearm-cpp in the workspace source directory, so the SDK is two levels up
# (`<ws>/src/litearm-ros2-control/litearm_ros2_control` -> `<ws>/src/litearm-cpp`).
set(LITEARM_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../litearm-cpp"
  CACHE PATH "litearm-cpp source tree, used when no installed litearm package is found")

find_package(litearm CONFIG QUIET)

if(litearm_FOUND)
  # A ros2_control plugin is a shared object, so the SDK it embeds must be position
  # independent. A static litearm is fine only if it was built with PIC.
  get_target_property(_litearm_type litearm::litearm TYPE)
  if(_litearm_type STREQUAL "STATIC_LIBRARY")
    message(FATAL_ERROR
      "The installed litearm::litearm is a static library (${_litearm_type}). A shared "
      "plugin cannot link a non-PIC static library. Reinstall litearm-cpp with "
      "-DBUILD_SHARED_LIBS=ON, or delete that install and let this package build the "
      "sibling tree instead.")
  endif()
  message(STATUS "Using installed litearm (${litearm_DIR})")
else()
  if(NOT EXISTS "${LITEARM_CPP_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "No installed litearm package was found, and the litearm-cpp source tree is not at "
      "'${LITEARM_CPP_DIR}'. Either install litearm-cpp (cmake with -DBUILD_SHARED_LIBS=ON, "
      "then its install step) or pass -DLITEARM_CPP_DIR=<path to litearm-cpp>.")
  endif()
  # Build the SDK as a static *PIC* library and link it into the plugin, so the plugin
  # ships self-contained: no extra liblitearm.so on LD_LIBRARY_PATH, and nothing from the
  # SDK leaks into this package's install prefix. PIC is what makes that legal for a
  # shared object.
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(LITEARM_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(LITEARM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  add_subdirectory("${LITEARM_CPP_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/litearm-cpp"
    EXCLUDE_FROM_ALL)
  message(STATUS "Using in-tree litearm from ${LITEARM_CPP_DIR} (static PIC, linked in)")
endif()

# ─────────────────────────────────────────────────────────────────────────────
# ros2_control hardware component
# ─────────────────────────────────────────────────────────────────────────────
add_library(litearm_ros2_control SHARED src/litearm_system.cpp)
target_include_directories(litearm_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ament_target_dependencies uses the plain target_link_libraries signature, so every
# target_link_libraries call for this target must stay plain too (no PUBLIC keyword).
ament_target_dependencies(litearm_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(litearm_ros2_control litearm::litearm)

pluginlib_export_plugin_description_file(hardware_interface litearm_ros2_control.xml)

# ─────────────────────────────────────────────────────────────────────────────
# install
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS litearm_ros2_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

# ─────────────────────────────────────────────────────────────────────────────
# tests
# ─────────────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_litearm_system test/test_litearm_system.cpp)
  ament_target_dependencies(test_litearm_system
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(test_litearm_system litearm_ros2_control litearm::litearm)
endif()

ament_package()
