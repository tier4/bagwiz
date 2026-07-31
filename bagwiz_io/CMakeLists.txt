cmake_minimum_required(VERSION 3.16)
project(bagwiz_io CXX)

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
find_package(bagwiz_base REQUIRED)
find_package(mcap_vendor REQUIRED)
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(SQLite3 REQUIRED)
# zstd is used by message_decompressor.cpp to handle rosbag2 MESSAGE-mode
# (per-message zstd) bags transparently. The vendor/module package below wires
# up CMAKE_MODULE_PATH so the inner find_package(zstd) can locate the imported
# `zstd::zstd` target. Humble and Jazzy ship zstd_vendor; newer (rolling) ROS
# distros replaced it with zstd_cmake_module (a thin module over the system
# libzstd). Both expose the same zstd::zstd target, so try zstd_vendor first and
# fall back.
find_package(zstd_vendor QUIET)
if(NOT zstd_vendor_FOUND)
  find_package(zstd_cmake_module REQUIRED)
endif()
find_package(zstd REQUIRED)

# mcap 1.x moved ReadJobQueue / ReadMessageJob / DecompressChunkJob under
# mcap::internal; mcap 0.x leaves them in mcap. The layout is tied to the
# distro's vendored mcap version, so detect it from the installed header and
# expose a compile definition for the src-local compatibility header.
set(_mcap_types_hpp "")
foreach(_mcap_include_dir IN LISTS mcap_vendor_INCLUDE_DIRS)
  if(EXISTS "${_mcap_include_dir}/mcap/types.hpp")
    set(_mcap_types_hpp "${_mcap_include_dir}/mcap/types.hpp")
    break()
  endif()
endforeach()
set(BAGWIZ_MCAP_READ_JOBS_INTERNAL_NAMESPACE OFF)
if(_mcap_types_hpp)
  file(STRINGS "${_mcap_types_hpp}" _mcap_version_line REGEX "^#define MCAP_LIBRARY_VERSION ")
  if(_mcap_version_line MATCHES "\"([0-9]+)\\.([0-9]+)\\.([0-9]+)\"")
    set(_mcap_major "${CMAKE_MATCH_1}")
    if(_mcap_major GREATER_EQUAL 1)
      set(BAGWIZ_MCAP_READ_JOBS_INTERNAL_NAMESPACE ON)
    endif()
  endif()
endif()

add_library(bagwiz_io
  src/io/bag_factory.cpp
  src/io/bag_open.cpp
  src/io/file_decompressor.cpp
  src/io/mcap_chunk_codec.cpp
  src/io/mcap_chunk_prefetch.cpp
  src/io/mcap_indexed_stream.cpp
  src/io/mcap_parallel_chunk_writer.cpp
  src/io/mcap_passthrough.cpp
  src/io/mcap_reader.cpp
  src/io/mcap_writer.cpp
  src/io/message_decompressor.cpp
  src/io/metadata_computer.cpp
  src/io/metadata_yaml.cpp
  src/io/sqlite3_reader.cpp
  src/io/sqlite3_slice_prefetch.cpp
  src/io/sqlite3_slice_schedule.cpp
  src/io/sqlite3_writer.cpp
  src/io/topics.cpp
)
if(BAGWIZ_MCAP_READ_JOBS_INTERNAL_NAMESPACE)
  target_compile_definitions(bagwiz_io PRIVATE BAGWIZ_MCAP_READ_JOBS_INTERNAL_NAMESPACE=1)
endif()
target_include_directories(bagwiz_io
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bagwiz_io PUBLIC bagwiz_base::bagwiz_base)
# sqlite3.h appears in a public header (sqlite3_helpers.hpp), so SQLite3 is
# linked PUBLIC and exported below. mcap, yaml-cpp, and zstd stay
# implementation details of the .cpp files and are linked PRIVATE.
target_link_libraries(bagwiz_io PUBLIC SQLite::SQLite3)
# ROS package dependencies of bagwiz_io. Jazzy's ament_target_dependencies
# accepts only the SYSTEM/INTERFACE/PUBLIC keywords, so these PRIVATE deps are
# linked through the per-package modern CMake targets (${pkg}_TARGETS, set by
# each find_package above) instead.
set(bagwiz_io_ros_deps
  mcap_vendor
)
foreach(bagwiz_dep IN LISTS bagwiz_io_ros_deps)
  target_link_libraries(bagwiz_io PRIVATE ${${bagwiz_dep}_TARGETS})
endforeach()
target_link_libraries(bagwiz_io PRIVATE yaml-cpp zstd::zstd)

install(TARGETS bagwiz_io
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# SQLite3 appears in a public header (sqlite3_helpers.hpp), so consumers need
# it for the include path alone. mcap and zstd are private
# link dependencies, but bagwiz_io is a static archive: their imported targets
# still show up in its link interface as $<LINK_ONLY:...>, so downstream
# packages must be able to re-create those targets when they find_package
# bagwiz_io. (yaml-cpp leaks only as a plain `-lyaml-cpp` link item, which the
# conda toolchain resolves without a target, so it needs no export.)
ament_export_dependencies(bagwiz_base SQLite3 mcap_vendor)
if(zstd_vendor_FOUND)
  ament_export_dependencies(zstd_vendor)
else()
  ament_export_dependencies(zstd_cmake_module)
endif()
ament_export_dependencies(zstd)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages used by the round-trip tests
  # (writer_roundtrip_test). These are not runtime deps of bagwiz_io — the
  # messages are (de)serialized through the introspection backend, which
  # dlopens the matching `.so` files at runtime — so they only need to be on
  # AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  # mcap is a private dep of bagwiz_io, so tests that build MCAP fixtures
  # directly (<mcap/writer.hpp>) re-link it explicitly.
  ament_add_gtest(bagwiz_mcap_reader_test test/io/mcap_reader_test.cpp)
  target_link_libraries(bagwiz_mcap_reader_test bagwiz_io ${mcap_vendor_TARGETS})

  ament_add_gtest(bagwiz_mcap_parallel_read_test test/io/mcap_parallel_read_test.cpp)
  target_link_libraries(bagwiz_mcap_parallel_read_test bagwiz_io ${mcap_vendor_TARGETS})

  ament_add_gtest(bagwiz_mcap_passthrough_test test/io/mcap_passthrough_test.cpp)
  target_link_libraries(bagwiz_mcap_passthrough_test bagwiz_io ${mcap_vendor_TARGETS})

  ament_add_gtest(bagwiz_mcap_parallel_write_test test/io/mcap_parallel_write_test.cpp)
  target_link_libraries(bagwiz_mcap_parallel_write_test bagwiz_io ${mcap_vendor_TARGETS})

  ament_add_gtest(bagwiz_sqlite3_reader_test test/io/sqlite3_reader_test.cpp)
  target_link_libraries(bagwiz_sqlite3_reader_test bagwiz_io)

  ament_add_gtest(bagwiz_sqlite3_parallel_read_test test/io/sqlite3_parallel_read_test.cpp)
  target_link_libraries(bagwiz_sqlite3_parallel_read_test bagwiz_io)

  ament_add_gtest(bagwiz_metadata_computer_test test/io/metadata_computer_test.cpp)
  target_link_libraries(bagwiz_metadata_computer_test bagwiz_io)

  # mcap, yaml-cpp, and zstd are private deps of bagwiz_io, so tests that build
  # or inspect bag fixtures through them directly re-link them explicitly.
  ament_add_gtest(
    bagwiz_message_decompressor_test test/io/message_decompressor_test.cpp)
  target_link_libraries(bagwiz_message_decompressor_test bagwiz_io zstd::zstd)

  ament_add_gtest(
    bagwiz_message_compression_e2e_test test/io/message_compression_e2e_test.cpp)
  target_link_libraries(
    bagwiz_message_compression_e2e_test bagwiz_io ${mcap_vendor_TARGETS} yaml-cpp zstd::zstd)

  ament_add_gtest(
    bagwiz_file_decompressor_test test/io/file_decompressor_test.cpp)
  target_link_libraries(bagwiz_file_decompressor_test bagwiz_io zstd::zstd)

  ament_add_gtest(
    bagwiz_file_compression_e2e_test test/io/file_compression_e2e_test.cpp)
  target_link_libraries(
    bagwiz_file_compression_e2e_test bagwiz_io yaml-cpp zstd::zstd)

  ament_add_gtest(bagwiz_writer_roundtrip_test test/io/writer_roundtrip_test.cpp)
  target_link_libraries(bagwiz_writer_roundtrip_test bagwiz_io yaml-cpp)

  ament_add_gtest(bagwiz_detect_format_test test/io/detect_format_test.cpp)
  target_link_libraries(bagwiz_detect_format_test bagwiz_io)

  ament_add_gtest(
    bagwiz_create_options_inheriting_test test/io/create_options_inheriting_test.cpp)
  target_link_libraries(bagwiz_create_options_inheriting_test bagwiz_io)

  ament_add_gtest(
    bagwiz_create_options_preserving_storage_test
    test/io/create_options_preserving_storage_test.cpp)
  target_link_libraries(bagwiz_create_options_preserving_storage_test bagwiz_io)

  ament_add_gtest(bagwiz_bag_open_test test/io/bag_open_test.cpp)
  target_link_libraries(bagwiz_bag_open_test bagwiz_io)

  ament_add_gtest(bagwiz_topics_test test/io/topics_test.cpp)
  target_link_libraries(bagwiz_topics_test bagwiz_io)

  # The multiplexer template is a src-local detail header; the test drives it
  # with a fake file-reader (the mcap/sqlite3 reader suites cover the two real
  # instantiations end to end).
  ament_add_gtest(bagwiz_shard_multiplexer_test test/io/shard_multiplexer_test.cpp)
  target_include_directories(bagwiz_shard_multiplexer_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(bagwiz_shard_multiplexer_test bagwiz_io)

  # The slice schedule is a src-local detail header; the test drives the pure
  # boundary arithmetic without a database.
  ament_add_gtest(
    bagwiz_sqlite3_slice_schedule_test test/io/sqlite3_slice_schedule_test.cpp)
  target_include_directories(
    bagwiz_sqlite3_slice_schedule_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(bagwiz_sqlite3_slice_schedule_test bagwiz_io)

  # The slice scanner is a src-local detail class; the test drives its worker
  # pool against a throwaway .db3 (the sqlite3 reader suite covers the wiring).
  ament_add_gtest(
    bagwiz_sqlite3_slice_prefetch_test test/io/sqlite3_slice_prefetch_test.cpp)
  target_include_directories(
    bagwiz_sqlite3_slice_prefetch_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(bagwiz_sqlite3_slice_prefetch_test bagwiz_io)

  # Drives the page-size knob through both the src-local resolver and a bag
  # written via the public factory, so the pragma is proven to reach the file.
  ament_add_gtest(bagwiz_sqlite3_page_size_test test/io/sqlite3_page_size_test.cpp)
  target_include_directories(
    bagwiz_sqlite3_page_size_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(bagwiz_sqlite3_page_size_test bagwiz_io)
endif()

ament_package()
