cmake_minimum_required(VERSION 3.16)
project(bagwiz_base CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Scope to C++ only. When BAGWIZ_WITH_SLAM_CUDA enables the CUDA language,
  # these flags would otherwise reach nvcc, which forwards -Wpedantic to the
  # host compiler on its generated intermediate (.cudafe1.stub.c). That file is
  # full of GCC-style linemarkers, so -Wpedantic floods the build with
  # "style of line directive is a GCC extension" — one per line of the .cu.
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Wshadow>")
endif()

# SLAM / map-viewer build options. scripts/bagwiz-build.sh passes them
# colcon-wide (--cmake-args reaches every package in the workspace), so every
# bagwiz_* package declares them: an undeclared cache variable makes CMake
# warn "Manually-specified variables were not used" per package. They are
# consumed only by bagwiz_slam (the GLIM wrapper library) and bagwiz (the
# CLI's `map` command group); this package builds the same either way.
option(BAGWIZ_WITH_SLAM "Build the in-process GLIM `bagwiz map slam` command" OFF)
option(BAGWIZ_WITH_SLAM_CUDA "GPU-accelerate `bagwiz map slam --backend cuda` (implies BAGWIZ_WITH_SLAM)" OFF)
option(BAGWIZ_WITH_MAP_VIEWER "Build the `map slam --viewer` browser map viewer (fetches cpp-httplib)" OFF)
if(BAGWIZ_WITH_SLAM_CUDA)
  set(BAGWIZ_WITH_SLAM ON CACHE BOOL "Implied by BAGWIZ_WITH_SLAM_CUDA" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)

add_library(bagwiz_base
  src/core/base/atomic_write.cpp
  src/core/base/duration_parse.cpp
  src/core/base/logging.cpp
  src/core/base/output_path.cpp
  src/core/base/signal_handler.cpp
  src/core/base/str_utils.cpp
  src/core/base/terminal_input.cpp
  src/core/base/topic_match.cpp
)
target_include_directories(bagwiz_base
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# ROS package dependencies of bagwiz_base. ament_target_dependencies links each
# package's include dirs and libraries, but newer (rolling) ROS distros removed
# it in favour of the per-package modern CMake targets. Humble/Jazzy still
# provide the macro and keep using it; where it is absent, link the exported
# ${pkg}_TARGETS instead (each was found via find_package above).
set(bagwiz_base_ros_deps
  rcutils
)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(bagwiz_base PUBLIC ${bagwiz_base_ros_deps})
else()
  foreach(bagwiz_dep IN LISTS bagwiz_base_ros_deps)
    target_link_libraries(bagwiz_base PUBLIC ${${bagwiz_dep}_TARGETS})
  endforeach()
endif()

install(TARGETS bagwiz_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# rcutils appears in the public logging.hpp, so downstream packages need it.
ament_export_dependencies(rcutils)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(bagwiz_atomic_write_test test/core/base/atomic_write_test.cpp)
  target_link_libraries(bagwiz_atomic_write_test bagwiz_base)

  ament_add_gtest(bagwiz_duration_parse_test test/core/base/duration_parse_test.cpp)
  target_link_libraries(bagwiz_duration_parse_test bagwiz_base)

  ament_add_gtest(bagwiz_output_path_test test/core/base/output_path_test.cpp)
  target_link_libraries(bagwiz_output_path_test bagwiz_base)

  ament_add_gtest(bagwiz_str_utils_test test/core/base/str_utils_test.cpp)
  target_link_libraries(bagwiz_str_utils_test bagwiz_base)

  ament_add_gtest(bagwiz_terminal_input_test test/core/base/terminal_input_test.cpp)
  target_link_libraries(bagwiz_terminal_input_test bagwiz_base)

  ament_add_gtest(bagwiz_tolerance_test test/core/base/tolerance_test.cpp)
  target_link_libraries(bagwiz_tolerance_test bagwiz_base)

  ament_add_gtest(bagwiz_topic_match_test test/core/base/topic_match_test.cpp)
  target_link_libraries(bagwiz_topic_match_test bagwiz_base)
endif()

ament_package()
