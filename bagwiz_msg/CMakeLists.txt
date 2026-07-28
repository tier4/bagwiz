cmake_minimum_required(VERSION 3.16)
project(bagwiz_msg CXX)

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
option(BAGWIZ_WITH_SLAM_CUDA "GPU-accelerate `bagwiz map slam --gpu` (implies BAGWIZ_WITH_SLAM)" OFF)
option(BAGWIZ_WITH_MAP_VIEWER "Build the `map slam --viewer` browser map viewer (fetches cpp-httplib)" OFF)
if(BAGWIZ_WITH_SLAM_CUDA)
  set(BAGWIZ_WITH_SLAM ON CACHE BOOL "Implied by BAGWIZ_WITH_SLAM_CUDA" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(bagwiz_base REQUIRED)
find_package(bagwiz_io REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcutils REQUIRED)
# Introspection fallback path: rmw_deserialize plus the per-package
# typesupport `.so`s loaded at runtime via dlopen. Still required
# alongside the schema-driven decoder because SQLite3 inputs and
# legacy MCAPs without embedded schemas have no schema text for the
# decoder factory to use.
find_package(rmw REQUIRED)
# rmw_serialize/rmw_deserialize are declared in <rmw/rmw.h> but DEFINED in
# librmw_implementation.so, so both packages are needed at link time.
find_package(rmw_implementation REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(bagwiz_msg
  src/core/cdr_walker/cdr_reader.cpp
  src/core/cdr_walker/cdr_writer.cpp
  src/core/cdr_walker/walker.cpp
  src/core/decoder/decoder_factory.cpp
  src/core/decoder/introspection_decoder.cpp
  src/core/decoder/message_view.cpp
  src/core/decoder/schema_decoder.cpp
  src/core/introspection/introspection_loader.cpp
  src/core/introspection/message_deserializer.cpp
  src/core/msg_schema/parser.cpp
  src/core/msg_schema/schema_model.cpp
  src/core/msg_yaml/message_formatter.cpp
  src/core/msg_yaml/msg_definition_resolver.cpp
  src/core/msg_yaml/ros2_yaml_to_cdr.cpp
  src/core/msg_yaml/yaml_msg_stamp_sync.cpp
  src/core/msg_yaml/yaml_msg_validate.cpp
)
target_include_directories(bagwiz_msg
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# decoder.hpp (public) includes io/bag_io.hpp, so bagwiz_io is PUBLIC; the
# remaining ROS packages are used only inside .cpp files and stay PRIVATE so
# they do not enter bagwiz_msg's export set.
target_link_libraries(bagwiz_msg PUBLIC bagwiz_base::bagwiz_base bagwiz_io::bagwiz_io)
# ROS package dependencies of bagwiz_msg. Jazzy's ament_target_dependencies
# accepts only the SYSTEM/INTERFACE/PUBLIC keywords, so these PRIVATE deps are
# linked through the per-package modern CMake targets (${pkg}_TARGETS, set by
# each find_package above) instead.
set(bagwiz_msg_ros_deps
  ament_index_cpp
  rcutils
  rmw
  rmw_implementation
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
  geometry_msgs
)
foreach(bagwiz_dep IN LISTS bagwiz_msg_ros_deps)
  target_link_libraries(bagwiz_msg PRIVATE ${${bagwiz_dep}_TARGETS})
endforeach()
target_link_libraries(bagwiz_msg PRIVATE yaml-cpp ${CMAKE_DL_LIBS})

install(TARGETS bagwiz_msg
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# bagwiz_io appears in the public decoder.hpp; the rest are private link
# dependencies, but bagwiz_msg is a static archive: their imported targets
# still show up in its link interface as $<LINK_ONLY:...>, so downstream
# packages must be able to re-create those targets when they find_package
# bagwiz_msg. (yaml-cpp leaks only as a plain `-lyaml-cpp` link item, which
# the conda toolchain resolves without a target, so it needs no export.)
ament_export_dependencies(
  bagwiz_base
  bagwiz_io
  ament_index_cpp
  rcutils
  rmw
  rmw_implementation
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages used by the introspection-path tests
  # (decoder_test, message_formatter regression, msg_definition_resolver
  # tests). These are not runtime deps of bagwiz_msg — the introspection
  # backend dlopens the matching `.so` files at runtime — so they only
  # need to be on AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  ament_add_gtest(bagwiz_cdr_walker_test test/core/cdr_walker/walker_test.cpp)
  target_link_libraries(bagwiz_cdr_walker_test bagwiz_msg)

  ament_add_gtest(bagwiz_decoder_test test/core/decoder/decoder_test.cpp)
  target_link_libraries(bagwiz_decoder_test bagwiz_msg)

  ament_add_gtest(bagwiz_msg_schema_parser_test test/core/msg_schema/parser_test.cpp)
  target_link_libraries(bagwiz_msg_schema_parser_test bagwiz_msg)

  # yaml-cpp is a private dep of bagwiz_msg, so the msg_yaml tests that parse
  # the produced YAML directly re-link it explicitly.
  ament_add_gtest(bagwiz_message_formatter_test test/core/msg_yaml/message_formatter_test.cpp)
  target_link_libraries(bagwiz_message_formatter_test bagwiz_msg yaml-cpp)

  ament_add_gtest(
    bagwiz_msg_definition_resolver_test test/core/msg_yaml/msg_definition_resolver_test.cpp)
  target_link_libraries(bagwiz_msg_definition_resolver_test bagwiz_msg)

  ament_add_gtest(bagwiz_yaml_msg_validate_test test/core/msg_yaml/yaml_msg_validate_test.cpp)
  target_link_libraries(bagwiz_yaml_msg_validate_test bagwiz_msg yaml-cpp)

  ament_add_gtest(bagwiz_yaml_msg_stamp_sync_test test/core/msg_yaml/yaml_msg_stamp_sync_test.cpp)
  target_link_libraries(bagwiz_yaml_msg_stamp_sync_test bagwiz_msg yaml-cpp)

  ament_add_gtest(bagwiz_mcap_schema_decode_test test/io/mcap_schema_decode_test.cpp)
  target_link_libraries(bagwiz_mcap_schema_decode_test bagwiz_msg)
endif()

ament_package()
