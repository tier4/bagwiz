cmake_minimum_required(VERSION 3.16)
project(bagwiz_tui CXX)

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
find_package(bagwiz_image REQUIRED)
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# libsixel (conda-forge `libsixel`) backs the Sixel backend of `bagwiz walk`'s
# image preview (src/core/tui/image/terminal_image_renderer.cpp). Its pkg-config
# file lists optional Requires.private integrations (libcurl, libjpeg, libpng,
# gdk-pixbuf-2.0) whose .pc files are not installed in the env, so
# `pkg_check_modules(... REQUIRED)` would fail dependency resolution even though
# `-lsixel` itself links and loads fine (the libsixel runtime .so deps are
# present). Locate the library and header directly instead. Consumed PRIVATE
# (every libsixel symbol stays inside the renderer .cpp) so it never enters
# bagwiz_tui's export set.
find_library(SIXEL_LIBRARY NAMES sixel REQUIRED)
find_path(SIXEL_INCLUDE_DIR NAMES sixel.h REQUIRED)

# terminal_image_renderer.cpp uses libavutil (imgutils / mem / pixfmt) for frame
# buffers and libswscale (sws_getContext / sws_scale / sws_freeContext) to scale
# frames, so unlike the video/image packages this checks for just those two
# components. Consumed PRIVATE behind the renderer .cpp, matching libsixel.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET libavutil libswscale)

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

add_library(bagwiz_tui
  src/core/joke/joke_layout.cpp
  src/core/joke/jokes.cpp
  src/core/tui/layout.cpp
  src/core/tui/pager.cpp
  src/core/tui/renderer.cpp
  src/core/tui/screen_scope.cpp
  src/core/tui/width.cpp
  src/core/tui/word_wrap.cpp
  src/core/tui/image/terminal_image_caps.cpp
  src/core/tui/image/terminal_image_renderer.cpp
)
target_include_directories(bagwiz_tui
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    # Generated, build-only: holds the embedded joke data (jokes_data.hpp).
    "${BAGWIZ_GENERATED_INCLUDE_DIR}"
)
target_link_libraries(bagwiz_tui PUBLIC bagwiz_base::bagwiz_base bagwiz_image::bagwiz_image)
# yaml-cpp (jokes.cpp), libsixel, and libavutil/libswscale
# (terminal_image_renderer.cpp) are implementation details behind .cpp files, so
# they are linked PRIVATE and stay out of bagwiz_tui's export set. The libav
# components are linked by absolute path (LIBAV_LINK_LIBRARIES) so no
# PkgConfig::LIBAV target reference leaks into the static archive's export set;
# libsixel and yaml-cpp leak only as plain `-l` items, which the conda toolchain
# resolves without targets.
target_include_directories(bagwiz_tui PRIVATE ${LIBAV_INCLUDE_DIRS})
target_link_libraries(bagwiz_tui PRIVATE yaml-cpp ${LIBAV_LINK_LIBRARIES} ${SIXEL_LIBRARY})
target_include_directories(bagwiz_tui PRIVATE ${SIXEL_INCLUDE_DIR})

install(TARGETS bagwiz_tui
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(bagwiz_base bagwiz_image)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(bagwiz_tui_width_test test/core/tui/width_test.cpp)
  target_link_libraries(bagwiz_tui_width_test bagwiz_tui)

  ament_add_gtest(bagwiz_tui_word_wrap_test test/core/tui/word_wrap_test.cpp)
  target_link_libraries(bagwiz_tui_word_wrap_test bagwiz_tui)

  ament_add_gtest(bagwiz_tui_layout_test test/core/tui/layout_test.cpp)
  target_link_libraries(bagwiz_tui_layout_test bagwiz_tui)

  ament_add_gtest(bagwiz_tui_renderer_test test/core/tui/renderer_test.cpp)
  target_link_libraries(bagwiz_tui_renderer_test bagwiz_tui)

  ament_add_gtest(bagwiz_tui_pager_test test/core/tui/pager_test.cpp)
  target_link_libraries(bagwiz_tui_pager_test bagwiz_tui)

  ament_add_gtest(
    bagwiz_terminal_image_caps_test test/core/tui/image/terminal_image_caps_test.cpp)
  target_link_libraries(bagwiz_terminal_image_caps_test bagwiz_tui)

  ament_add_gtest(
    bagwiz_terminal_image_renderer_test test/core/tui/image/terminal_image_renderer_test.cpp)
  target_link_libraries(bagwiz_terminal_image_renderer_test bagwiz_tui)

  ament_add_gtest(bagwiz_jokes_test test/core/joke/jokes_test.cpp)
  target_link_libraries(bagwiz_jokes_test bagwiz_tui)

  ament_add_gtest(bagwiz_joke_layout_test test/core/joke/joke_layout_test.cpp)
  target_link_libraries(bagwiz_joke_layout_test bagwiz_tui)
endif()

ament_package()
