cmake_minimum_required(VERSION 3.16)
project(bagwiz_tf CXX)

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
find_package(bagwiz_io REQUIRED)
find_package(bagwiz_msg REQUIRED)
# tf2 supplies BufferCore + math helpers used by the TF trajectory code.
# geometry_msgs is needed for the header-only TransformStamped struct handed to
# BufferCore::setTransform from tf_value_extract. tf2_msgs is pulled in for
# tf_message_wire, which constructs a tf2_msgs::msg::TFMessage in-memory and
# serialises it via the introspection typesupport — only the header-only struct
# is needed at build time (the typesupport `.so` is dlopen'd at runtime).
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rmw REQUIRED)
# tf_message_wire.cpp calls rmw_serialize, which is declared in <rmw/rmw.h>
# but DEFINED in librmw_implementation.so, so both packages are needed at link
# time.
find_package(rmw_implementation REQUIRED)
# nlohmann_json (header-only) backs the TF static transform JSON export. Used
# only inside tf_transform_format.cpp, so it is linked PRIVATE and does not
# leak into the export set.
find_package(nlohmann_json REQUIRED)
# yaml-cpp backs the static-TF-tree YAML reader (tf_static_tree_yaml.cpp). Only
# the reader needs it: the writer emits its YAML directly, so nothing in the
# public header exposes a yaml-cpp type and it is linked PRIVATE.
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(bagwiz_tf
  src/core/tf/tf_buffer_loader.cpp
  src/core/tf/tf_chain.cpp
  src/core/tf/tf_forest_check.cpp
  src/core/tf/tf_merge_check.cpp
  src/core/tf/tf_message_wire.cpp
  src/core/tf/tf_static_collect.cpp
  src/core/tf/tf_static_tree_yaml.cpp
  src/core/tf/tf_topics.cpp
  src/core/tf/tf_trajectory_sample.cpp
  src/core/tf/tf_transform_format.cpp
  src/core/tf/tf_tree_check.cpp
  src/core/tf/tf_value_extract.cpp
  src/core/tf/trajectory.cpp
)
target_include_directories(bagwiz_tf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bagwiz_tf PUBLIC bagwiz_io::bagwiz_io bagwiz_msg::bagwiz_msg)
# ROS package dependencies of bagwiz_tf. ament_target_dependencies links each
# package's include dirs and libraries, but newer (rolling) ROS distros removed
# it in favour of the per-package modern CMake targets. Humble/Jazzy still
# provide the macro and keep using it; where it is absent, link the exported
# ${pkg}_TARGETS instead (each was found via find_package above).
# tf2, geometry_msgs, and nav_msgs appear in public headers (tf_chain.hpp,
# tf_value_extract.hpp, trajectory.hpp, ...) and are PUBLIC; the rest are used
# only inside .cpp files and stay PRIVATE. Jazzy's ament_target_dependencies
# accepts only the SYSTEM/INTERFACE/PUBLIC keywords, so the PRIVATE deps are
# linked through the per-package modern CMake targets (${pkg}_TARGETS) while
# the PUBLIC ones keep the ament idiom with its rolling fallback.
set(bagwiz_tf_ros_deps_public
  geometry_msgs
  nav_msgs
  tf2
)
set(bagwiz_tf_ros_deps_private
  rmw
  rmw_implementation
  std_msgs
  tf2_msgs
)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(bagwiz_tf PUBLIC ${bagwiz_tf_ros_deps_public})
else()
  foreach(bagwiz_dep IN LISTS bagwiz_tf_ros_deps_public)
    target_link_libraries(bagwiz_tf PUBLIC ${${bagwiz_dep}_TARGETS})
  endforeach()
endif()
foreach(bagwiz_dep IN LISTS bagwiz_tf_ros_deps_private)
  target_link_libraries(bagwiz_tf PRIVATE ${${bagwiz_dep}_TARGETS})
endforeach()
# nlohmann_json is an implementation detail of tf_transform_format.cpp (the
# public header returns std::string). It is header-only, so pull its include
# dir from the target instead of linking it: nothing leaks into the static
# archive's link interface and downstream packages need no target.
target_include_directories(bagwiz_tf SYSTEM PRIVATE
  $<TARGET_PROPERTY:nlohmann_json::nlohmann_json,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(bagwiz_tf PRIVATE yaml-cpp)

install(TARGETS bagwiz_tf
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# tf2, geometry_msgs, and nav_msgs appear in public headers. rmw, std_msgs,
# and tf2_msgs are private link dependencies, but bagwiz_tf is a static
# archive: their imported targets still show up in its link interface as
# $<LINK_ONLY:...>, so downstream packages must be able to re-create those
# targets when they find_package bagwiz_tf.
ament_export_dependencies(
  bagwiz_io
  bagwiz_msg
  geometry_msgs
  nav_msgs
  tf2
  rmw
  rmw_implementation
  std_msgs
  tf2_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages used by the introspection-path tests
  # (tf_message_wire_test). These are not runtime deps of bagwiz_tf — the
  # introspection backend dlopens the matching `.so` files at runtime — so
  # they only need to be on AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  # yaml-cpp is a PRIVATE dependency of bagwiz_tf (see above), so tests that use
  # it directly — parsing the emitted YAML back, which is what proves the output
  # is well-formed rather than merely well-indented — must link it themselves.

  ament_add_gtest(bagwiz_tf_buffer_loader_test test/core/tf/tf_buffer_loader_test.cpp)
  target_link_libraries(bagwiz_tf_buffer_loader_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_chain_test test/core/tf/tf_chain_test.cpp)
  target_link_libraries(bagwiz_tf_chain_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_merge_check_test test/core/tf/tf_merge_check_test.cpp)
  target_link_libraries(bagwiz_tf_merge_check_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_replay_loader_test test/core/tf/tf_replay_loader_test.cpp)
  target_link_libraries(bagwiz_tf_replay_loader_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_topics_test test/core/tf/tf_topics_test.cpp)
  target_link_libraries(bagwiz_tf_topics_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_trajectory_sample_test test/core/tf/tf_trajectory_sample_test.cpp)
  target_link_libraries(bagwiz_tf_trajectory_sample_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_message_wire_test test/core/tf/tf_message_wire_test.cpp)
  target_link_libraries(bagwiz_tf_message_wire_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_static_tree_yaml_test test/core/tf/tf_static_tree_yaml_test.cpp)
  target_link_libraries(bagwiz_tf_static_tree_yaml_test bagwiz_tf yaml-cpp)

  ament_add_gtest(bagwiz_tf_tree_check_test test/core/tf/tf_tree_check_test.cpp)
  target_link_libraries(bagwiz_tf_tree_check_test bagwiz_tf)

  ament_add_gtest(bagwiz_tf_transform_format_test test/core/tf/tf_transform_format_test.cpp)
  target_link_libraries(
    bagwiz_tf_transform_format_test bagwiz_tf nlohmann_json::nlohmann_json)

  ament_add_gtest(bagwiz_trajectory_test test/core/tf/trajectory_test.cpp)
  target_link_libraries(bagwiz_trajectory_test bagwiz_tf)
endif()

ament_package()
