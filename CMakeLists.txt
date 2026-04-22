cmake_minimum_required(VERSION 3.16)
project(bagcli CXX)

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

# CLI11 is not available via rosdep; fetch at configure time.
include(FetchContent)
FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.4.2
  GIT_SHALLOW TRUE
)
set(CLI11_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cli11)

# ---------------------------------------------------------------------------
# bagcli_core: shared library with logging, command registry, and IO
# interfaces. Command implementations live in the executable target so their
# static registrars pull into the final binary.
# ---------------------------------------------------------------------------
add_library(bagcli_core
  src/core/logging.cpp
  src/commands/registry.cpp
)
target_include_directories(bagcli_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bagcli_core PUBLIC rcutils)

# ---------------------------------------------------------------------------
# bagcli: CLI executable. Installs to bin/ so users run it directly after
# sourcing the ROS setup.
# ---------------------------------------------------------------------------
add_executable(bagcli src/main.cpp)
target_link_libraries(bagcli PRIVATE bagcli_core CLI11::CLI11)

install(TARGETS bagcli RUNTIME DESTINATION bin)
install(TARGETS bagcli_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rcutils)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(bagcli_smoke_test test/smoke_test.cpp)
  target_link_libraries(bagcli_smoke_test bagcli_core)
endif()

ament_package()
