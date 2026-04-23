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
find_package(rcutils REQUIRED)
find_package(mcap_vendor REQUIRED)
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(SQLite3 REQUIRED)

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

FetchContent_MakeAvailable(cli11 fmt)

# ---------------------------------------------------------------------------
# bagwiz_core: shared library with logging, command registry, and IO
# interfaces. Command implementations live in the executable target so their
# static registrars pull into the final binary.
# ---------------------------------------------------------------------------
add_library(bagwiz_core
  src/core/logging.cpp
  src/core/topic_pattern.cpp
  src/commands/registry.cpp
  src/io/bag_factory.cpp
  src/io/mcap_reader.cpp
  src/io/mcap_writer.cpp
  src/io/metadata_yaml.cpp
  src/io/sqlite3_reader.cpp
  src/io/sqlite3_writer.cpp
)
target_include_directories(bagwiz_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bagwiz_core PUBLIC rcutils mcap_vendor yaml_cpp_vendor)
target_link_libraries(bagwiz_core PUBLIC yaml-cpp SQLite::SQLite3)

# ---------------------------------------------------------------------------
# bagwiz: CLI executable. Installs to bin/ so users run it directly after
# sourcing the ROS setup.
# ---------------------------------------------------------------------------
add_executable(bagwiz
  src/main.cpp
  src/commands/comp.cpp
  src/commands/ls.cpp
)
target_link_libraries(bagwiz PRIVATE bagwiz_core CLI11::CLI11 fmt::fmt)

install(TARGETS bagwiz RUNTIME DESTINATION bin)
install(TARGETS bagwiz_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rcutils mcap_vendor yaml_cpp_vendor SQLite3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(bagwiz_smoke_test test/smoke_test.cpp)
  target_link_libraries(bagwiz_smoke_test bagwiz_core)

  ament_add_gtest(bagwiz_topic_pattern_test test/core/topic_pattern_test.cpp)
  target_link_libraries(bagwiz_topic_pattern_test bagwiz_core)

  ament_add_gtest(bagwiz_mcap_reader_test test/io/mcap_reader_test.cpp)
  target_link_libraries(bagwiz_mcap_reader_test bagwiz_core)

  ament_add_gtest(bagwiz_sqlite3_reader_test test/io/sqlite3_reader_test.cpp)
  target_link_libraries(bagwiz_sqlite3_reader_test bagwiz_core)

  ament_add_gtest(bagwiz_writer_roundtrip_test test/io/writer_roundtrip_test.cpp)
  target_link_libraries(bagwiz_writer_roundtrip_test bagwiz_core)
endif()

ament_package()
