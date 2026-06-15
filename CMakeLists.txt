cmake_minimum_required(VERSION 3.16)
project(bagwiz CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(mcap_vendor REQUIRED)
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(SQLite3 REQUIRED)
# zstd is used by message_decompressor.cpp to handle rosbag2 MESSAGE-mode
# (per-message zstd) bags transparently. The vendor/module package below wires
# up CMAKE_MODULE_PATH so the inner find_package(zstd) can locate the imported
# `zstd::zstd` target. Humble and Jazzy ship zstd_vendor; Lyrical and
# newer replaced it with zstd_cmake_module (a thin module over the system
# libzstd). Both expose the same zstd::zstd target, so try zstd_vendor first and
# fall back.
find_package(zstd_vendor QUIET)
if(NOT zstd_vendor_FOUND)
  find_package(zstd_cmake_module REQUIRED)
endif()
find_package(zstd REQUIRED)
# Introspection fallback path: rmw_deserialize plus the per-package
# typesupport `.so`s loaded at runtime via dlopen. Still required
# alongside the schema-driven decoder because SQLite3 inputs and
# legacy MCAPs without embedded schemas have no schema text for the
# decoder factory to use.
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
# tf2 supplies BufferCore + math helpers used by `tf tree` and
# `traj dump --from/--to`. geometry_msgs is needed for the
# header-only TransformStamped struct we hand to BufferCore::setTransform
# from tf_value_extract. tf2_msgs is pulled in for `traj join`, which
# constructs a tf2_msgs::msg::TFMessage in-memory and serialises it via
# the introspection typesupport (see tf_message_wire) — only the
# header-only struct is needed at build time (the typesupport `.so` is
# dlopen'd at runtime).
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
# nlohmann_json (header-only) backs `tf static --json`. Used only inside
# bagwiz_core's tf_transform_format.cpp, so it is linked PRIVATE and does
# not leak into the export set.
find_package(nlohmann_json REQUIRED)

# GeographicLib provides the WGS84 -> ENU / UTM projections used by
# `convert msgtype geo` (geo_pose_convert.cpp). It is consumed PRIVATE — an
# implementation detail behind the GeoPoseConverter pimpl that must not leak
# into bagwiz_core's export set. Two packagings exist and both are supported:
#   - a CMake CONFIG package (e.g. conda-forge `geographiclib-cpp`, used by the
#     pixi/RoboStack build) that exports the GeographicLib::GeographicLib target;
#   - the classic Debian/Ubuntu find-module that ships no imported target, only
#     sets GeographicLib_LIBRARIES / _INCLUDE_DIRS, and lives under
#     /usr/share/cmake/geographiclib.
# Try CONFIG mode first; fall back to MODULE mode, appending the Debian module
# directory to CMAKE_MODULE_PATH only when it exists so the path is harmless
# inside a conda prefix (which has no /usr/share/cmake/geographiclib).
find_package(GeographicLib CONFIG QUIET)
if(NOT GeographicLib_FOUND)
  if(EXISTS "/usr/share/cmake/geographiclib")
    list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
  endif()
  find_package(GeographicLib MODULE REQUIRED)
endif()
# Normalise both packagings onto the variables the consumers below use: when the
# CONFIG package exported only the imported target, point GeographicLib_LIBRARIES
# at it (a target is a valid link item, and linking it propagates its include
# dirs — same approach as the PRIVATE nlohmann_json::nlohmann_json link). The
# classic find-module already populates these variables itself.
if(TARGET GeographicLib::GeographicLib AND NOT GeographicLib_LIBRARIES)
  set(GeographicLib_LIBRARIES GeographicLib::GeographicLib)
endif()

# CLI11 and fmt are not available as rosdep keys on all supported distros, so
# fetch them at configure time. Both are small, header-light libraries.
include(FetchContent)

FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.4.2
  GIT_SHALLOW TRUE
)
set(CLI11_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 11.0.2
  GIT_SHALLOW TRUE
)
set(FMT_TEST OFF CACHE BOOL "" FORCE)
set(FMT_DOC OFF CACHE BOOL "" FORCE)
set(FMT_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  rang
  GIT_REPOSITORY https://github.com/agauniyal/rang.git
  GIT_TAG v3.2
  GIT_SHALLOW TRUE
)
# rang (and older CLI11) declare cmake_minimum_required below the floor CMake
# 4.0 still accepts, which is a hard error on newer toolchains such as the ROS 2
# Lyrical image. Give the fetched subprojects a compatibility floor. Older CMake
# (Humble/Jazzy) does not know this variable and simply ignores it.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
FetchContent_MakeAvailable(cli11 fmt rang)
unset(CMAKE_POLICY_VERSION_MINIMUM)

# ---------------------------------------------------------------------------
# Embedded joke data: read the single source-of-truth JSON at configure time
# and bake it into a generated header. This keeps `bagwiz joke` self-contained
# (no runtime file / sourced overlay needed) while jokes stay managed in one
# JSON file. CMAKE_CONFIGURE_DEPENDS re-runs configure when the JSON changes.
# ---------------------------------------------------------------------------
set(BAGWIZ_JOKES_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/core/joke/jokes.json")
file(READ "${BAGWIZ_JOKES_SOURCE}" BAGWIZ_JOKES_JSON)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${BAGWIZ_JOKES_SOURCE}")
set(BAGWIZ_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/src/core/joke/jokes_data.hpp.in"
  "${BAGWIZ_GENERATED_INCLUDE_DIR}/bagwiz/core/joke/jokes_data.hpp"
  @ONLY)

# ---------------------------------------------------------------------------
# bagwiz_core: shared library with logging, command registry, and IO
# interfaces. Command implementations live in the executable target so their
# static registrars pull into the final binary.
# ---------------------------------------------------------------------------
add_library(bagwiz_core
  src/core/cdr_walker/cdr_reader.cpp
  src/core/cdr_walker/walker.cpp
  src/core/decoder/decoder_factory.cpp
  src/core/decoder/introspection_decoder.cpp
  src/core/decoder/message_view.cpp
  src/core/decoder/schema_decoder.cpp
  src/core/introspection_loader.cpp
  src/core/joke/jokes.cpp
  src/core/joke/joke_layout.cpp
  src/core/logging.cpp
  src/core/message_deserializer.cpp
  src/core/output_path.cpp
  src/core/message_formatter.cpp
  src/core/msg_definition_resolver.cpp
  src/core/msg_schema/parser.cpp
  src/core/msg_schema/schema_model.cpp
  src/core/ros2_yaml_to_cdr.cpp
  src/core/yaml_msg_stamp_sync.cpp
  src/core/yaml_msg_validate.cpp
  src/core/terminal_input.cpp
  src/core/bag_copy.cpp
  src/core/bag_inplace.cpp
  src/core/bag_topic_plan.cpp
  src/core/topic_match.cpp
  src/core/broken_bag.cpp
  src/core/tf_chain.cpp
  src/core/tf_merge_check.cpp
  src/core/tf_message_wire.cpp
  src/core/tf_transform_format.cpp
  src/core/tf_value_extract.cpp
  src/core/tf_walk_timeline.cpp
  src/core/trajectory.cpp
  src/core/msgtype_convert/geo_pose_convert.cpp
  src/core/tui/layout.cpp
  src/core/tui/pager.cpp
  src/core/tui/renderer.cpp
  src/core/tui/screen_scope.cpp
  src/core/tui/signal_handler.cpp
  src/core/tui/width.cpp
  src/commands/registry.cpp
  src/io/bag_factory.cpp
  src/io/file_decompressor.cpp
  src/io/mcap_reader.cpp
  src/io/mcap_writer.cpp
  src/io/message_decompressor.cpp
  src/io/metadata_computer.cpp
  src/io/metadata_yaml.cpp
  src/io/sqlite3_reader.cpp
  src/io/sqlite3_writer.cpp
)
target_include_directories(bagwiz_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    # Generated, build-only: holds the embedded joke data (jokes_data.hpp).
    "${BAGWIZ_GENERATED_INCLUDE_DIR}"
)
# ROS package dependencies of bagwiz_core. ament_target_dependencies links each
# package's include dirs and libraries, but it was removed in Lyrical (rolling)
# in favour of the per-package modern CMake targets. Humble/Jazzy still provide
# the macro and keep using it; on Lyrical and newer, link the exported
# ${pkg}_TARGETS instead (each was found via find_package above).
set(bagwiz_core_ros_deps
  ament_index_cpp
  rcutils
  mcap_vendor
  yaml_cpp_vendor
  rmw
  rmw_implementation
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_msgs
)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(bagwiz_core PUBLIC ${bagwiz_core_ros_deps})
else()
  foreach(bagwiz_dep IN LISTS bagwiz_core_ros_deps)
    target_link_libraries(bagwiz_core PUBLIC ${${bagwiz_dep}_TARGETS})
  endforeach()
endif()
target_link_libraries(bagwiz_core PUBLIC yaml-cpp SQLite::SQLite3 zstd::zstd ${CMAKE_DL_LIBS})
# GeographicLib (WGS84 -> ENU/UTM projection) is private to geo_pose_convert.cpp.
target_include_directories(bagwiz_core PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(bagwiz_core PRIVATE ${GeographicLib_LIBRARIES})
# nlohmann_json is an implementation detail of tf_transform_format.cpp (the
# public header returns std::string), so keep it PRIVATE and out of the
# export set.
target_link_libraries(bagwiz_core PRIVATE nlohmann_json::nlohmann_json)
# fmt is used header-only inside bagwiz_core (renderer.cpp). Linking fmt::fmt
# would force it into the bagwiz_core export set, which fails because fmt is
# fetched via FetchContent and not installable.
target_compile_definitions(bagwiz_core PRIVATE FMT_HEADER_ONLY)
target_include_directories(bagwiz_core PRIVATE $<TARGET_PROPERTY:fmt::fmt,INTERFACE_INCLUDE_DIRECTORIES>)

# ---------------------------------------------------------------------------
# bagwiz: CLI executable. Installs to bin/ so users run it directly after
# sourcing the ROS setup.
# ---------------------------------------------------------------------------
add_executable(bagwiz
  src/main.cpp
  src/commands/check.cpp
  src/commands/completion.cpp
  src/commands/convert.cpp
  src/commands/convert_msgtype_geo.cpp
  src/commands/joke.cpp
  src/commands/ls.cpp
  src/commands/walk.cpp
  src/commands/traj.cpp
  src/commands/tf.cpp
  src/commands/tf_walk.cpp
  src/commands/tf_static_cp.cpp
  src/commands/topic.cpp
  src/commands/topic_omit.cpp
  src/commands/topic_keep.cpp
)
target_link_libraries(bagwiz PRIVATE bagwiz_core CLI11::CLI11 fmt::fmt rang)

install(TARGETS bagwiz RUNTIME DESTINATION bin)
install(TARGETS bagwiz_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rcutils
  mcap_vendor
  yaml_cpp_vendor
  SQLite3
  rmw
  rmw_implementation
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages used by the introspection-path tests
  # (decoder_test, message_formatter regression, msg_definition_resolver
  # tests). These are not runtime deps of bagwiz_core — the introspection
  # backend dlopens the matching `.so` files at runtime — so they only
  # need to be on AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  ament_add_gtest(bagwiz_completion_test
    test/commands/completion_test.cpp
    src/commands/completion.cpp)
  target_link_libraries(bagwiz_completion_test bagwiz_core CLI11::CLI11 fmt::fmt)

  ament_add_gtest(bagwiz_tf_static_cp_test
    test/commands/tf_static_cp_test.cpp
    src/commands/tf_static_cp.cpp)
  target_link_libraries(bagwiz_tf_static_cp_test bagwiz_core fmt::fmt)

  ament_add_gtest(bagwiz_smoke_test test/smoke_test.cpp)
  ament_add_gtest(bagwiz_yaml_msg_validate_test test/core/yaml_msg_validate_test.cpp)
  target_link_libraries(bagwiz_yaml_msg_validate_test bagwiz_core)

  ament_add_gtest(bagwiz_yaml_msg_stamp_sync_test test/core/yaml_msg_stamp_sync_test.cpp)
  target_link_libraries(bagwiz_yaml_msg_stamp_sync_test bagwiz_core)

  target_link_libraries(bagwiz_smoke_test bagwiz_core)

  ament_add_gtest(bagwiz_broken_bag_test test/core/broken_bag_test.cpp)
  target_link_libraries(bagwiz_broken_bag_test bagwiz_core)

  ament_add_gtest(bagwiz_message_formatter_test test/core/message_formatter_test.cpp)
  target_link_libraries(bagwiz_message_formatter_test bagwiz_core)

  ament_add_gtest(bagwiz_msg_schema_parser_test test/core/msg_schema/parser_test.cpp)
  target_link_libraries(bagwiz_msg_schema_parser_test bagwiz_core)

  ament_add_gtest(bagwiz_cdr_walker_test test/core/cdr_walker/walker_test.cpp)
  target_link_libraries(bagwiz_cdr_walker_test bagwiz_core)

  ament_add_gtest(bagwiz_decoder_test test/core/decoder/decoder_test.cpp)
  target_link_libraries(bagwiz_decoder_test bagwiz_core)

  ament_add_gtest(bagwiz_mcap_schema_decode_test test/io/mcap_schema_decode_test.cpp)
  target_link_libraries(bagwiz_mcap_schema_decode_test bagwiz_core)

  ament_add_gtest(bagwiz_terminal_input_test test/core/terminal_input_test.cpp)
  target_link_libraries(bagwiz_terminal_input_test bagwiz_core)

  ament_add_gtest(bagwiz_tui_width_test test/core/tui/width_test.cpp)
  target_link_libraries(bagwiz_tui_width_test bagwiz_core)

  ament_add_gtest(bagwiz_jokes_test test/core/joke/jokes_test.cpp)
  target_link_libraries(bagwiz_jokes_test bagwiz_core)

  ament_add_gtest(bagwiz_joke_layout_test test/core/joke/joke_layout_test.cpp)
  target_link_libraries(bagwiz_joke_layout_test bagwiz_core)

  ament_add_gtest(bagwiz_tui_layout_test test/core/tui/layout_test.cpp)
  target_link_libraries(bagwiz_tui_layout_test bagwiz_core)

  ament_add_gtest(bagwiz_tui_renderer_test test/core/tui/renderer_test.cpp)
  target_link_libraries(bagwiz_tui_renderer_test bagwiz_core)

  ament_add_gtest(bagwiz_tui_pager_test test/core/tui/pager_test.cpp)
  target_link_libraries(bagwiz_tui_pager_test bagwiz_core)

  ament_add_gtest(bagwiz_trajectory_test test/core/trajectory_test.cpp)
  target_link_libraries(bagwiz_trajectory_test bagwiz_core)

  ament_add_gtest(bagwiz_tf_chain_test test/core/tf_chain_test.cpp)
  target_link_libraries(bagwiz_tf_chain_test bagwiz_core)

  ament_add_gtest(bagwiz_tf_merge_check_test test/core/tf_merge_check_test.cpp)
  target_link_libraries(bagwiz_tf_merge_check_test bagwiz_core)

  ament_add_gtest(bagwiz_tf_message_wire_test test/core/tf_message_wire_test.cpp)
  target_link_libraries(bagwiz_tf_message_wire_test bagwiz_core)

  ament_add_gtest(bagwiz_tf_transform_format_test test/core/tf_transform_format_test.cpp)
  target_link_libraries(
    bagwiz_tf_transform_format_test bagwiz_core nlohmann_json::nlohmann_json)

  ament_add_gtest(bagwiz_tf_walk_timeline_test test/core/tf_walk_timeline_test.cpp)
  target_link_libraries(bagwiz_tf_walk_timeline_test bagwiz_core)

  ament_add_gtest(bagwiz_bag_topic_plan_test test/core/bag_topic_plan_test.cpp)
  target_link_libraries(bagwiz_bag_topic_plan_test bagwiz_core)

  ament_add_gtest(bagwiz_bag_copy_test test/core/bag_copy_test.cpp)
  target_link_libraries(bagwiz_bag_copy_test bagwiz_core)

  ament_add_gtest(bagwiz_bag_inplace_test test/core/bag_inplace_test.cpp)
  target_link_libraries(bagwiz_bag_inplace_test bagwiz_core)

  ament_add_gtest(bagwiz_topic_match_test test/core/topic_match_test.cpp)
  target_link_libraries(bagwiz_topic_match_test bagwiz_core)

  ament_add_gtest(bagwiz_topic_omit_test
    test/commands/topic_omit_test.cpp
    src/commands/topic_omit.cpp)
  target_link_libraries(bagwiz_topic_omit_test bagwiz_core fmt::fmt)

  ament_add_gtest(bagwiz_topic_keep_test
    test/commands/topic_keep_test.cpp
    src/commands/topic_keep.cpp)
  target_link_libraries(bagwiz_topic_keep_test bagwiz_core fmt::fmt)

  ament_add_gtest(bagwiz_output_path_test test/core/output_path_test.cpp)
  target_link_libraries(bagwiz_output_path_test bagwiz_core)

  ament_add_gtest(bagwiz_mcap_reader_test test/io/mcap_reader_test.cpp)
  target_link_libraries(bagwiz_mcap_reader_test bagwiz_core)

  ament_add_gtest(bagwiz_sqlite3_reader_test test/io/sqlite3_reader_test.cpp)
  target_link_libraries(bagwiz_sqlite3_reader_test bagwiz_core)

  ament_add_gtest(bagwiz_metadata_computer_test test/io/metadata_computer_test.cpp)
  target_link_libraries(bagwiz_metadata_computer_test bagwiz_core)

  ament_add_gtest(
    bagwiz_message_decompressor_test test/io/message_decompressor_test.cpp)
  target_link_libraries(bagwiz_message_decompressor_test bagwiz_core)

  ament_add_gtest(
    bagwiz_message_compression_e2e_test test/io/message_compression_e2e_test.cpp)
  target_link_libraries(bagwiz_message_compression_e2e_test bagwiz_core)

  ament_add_gtest(
    bagwiz_file_decompressor_test test/io/file_decompressor_test.cpp)
  target_link_libraries(bagwiz_file_decompressor_test bagwiz_core)

  ament_add_gtest(
    bagwiz_file_compression_e2e_test test/io/file_compression_e2e_test.cpp)
  target_link_libraries(bagwiz_file_compression_e2e_test bagwiz_core)

  ament_add_gtest(bagwiz_writer_roundtrip_test test/io/writer_roundtrip_test.cpp)
  target_link_libraries(bagwiz_writer_roundtrip_test bagwiz_core)

  ament_add_gtest(bagwiz_detect_format_test test/io/detect_format_test.cpp)
  target_link_libraries(bagwiz_detect_format_test bagwiz_core)

  ament_add_gtest(
    bagwiz_create_options_inheriting_test test/io/create_options_inheriting_test.cpp)
  target_link_libraries(bagwiz_create_options_inheriting_test bagwiz_core)

  ament_add_gtest(
    bagwiz_create_options_preserving_storage_test
    test/io/create_options_preserving_storage_test.cpp)
  target_link_libraries(bagwiz_create_options_preserving_storage_test bagwiz_core)

  ament_add_gtest(
    bagwiz_msg_definition_resolver_test test/core/msg_definition_resolver_test.cpp)
  target_link_libraries(bagwiz_msg_definition_resolver_test bagwiz_core)

  # geo_pose_convert lives in bagwiz_core; GeographicLib is linked explicitly so
  # the test still resolves the projection symbols under a static-lib build.
  ament_add_gtest(
    bagwiz_geo_pose_convert_test test/core/msgtype_convert/geo_pose_convert_test.cpp)
  target_link_libraries(
    bagwiz_geo_pose_convert_test bagwiz_core ${GeographicLib_LIBRARIES})

  ament_add_gtest(bagwiz_convert_msgtype_geo_test
    test/commands/convert_msgtype_geo_test.cpp
    src/commands/convert_msgtype_geo.cpp)
  target_link_libraries(
    bagwiz_convert_msgtype_geo_test bagwiz_core fmt::fmt ${GeographicLib_LIBRARIES}
    ${sensor_msgs_TARGETS})
endif()

ament_package()
