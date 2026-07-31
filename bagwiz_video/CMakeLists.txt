cmake_minimum_required(VERSION 3.16)
project(bagwiz_video CXX)

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

# FFmpeg/libav (conda-forge `ffmpeg`) backs `bagwiz generate video`
# (video_encoder.cpp): muxing, H.264/MJPEG encoding, and pixel conversion. No
# ament/CMake package exists for it, so locate the libraries via pkg-config. It
# is consumed PRIVATE (the encoder is hidden behind a pimpl), so libav never
# enters the export set. pkg_check_modules creates the imported target
# PkgConfig::LIBAV spanning all four component libraries.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec libavformat libavutil libswscale)

add_library(bagwiz_video
  src/core/video/frame_rate.cpp
  src/core/video/video_encoder.cpp
)
target_include_directories(bagwiz_video
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# libav (FFmpeg) is an implementation detail of video_encoder.cpp (the public
# header returns only std types behind a pimpl), so keep it PRIVATE. It is
# linked by absolute path (LIBAV_LINK_LIBRARIES) so no PkgConfig::LIBAV target
# reference leaks into the static archive's export set.
target_include_directories(bagwiz_video PRIVATE ${LIBAV_INCLUDE_DIRS})
target_link_libraries(bagwiz_video PRIVATE ${LIBAV_LINK_LIBRARIES})

install(TARGETS bagwiz_video
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(bagwiz_frame_rate_test test/core/video/frame_rate_test.cpp)
  target_link_libraries(bagwiz_frame_rate_test bagwiz_video)

  # video_encoder lives in bagwiz_video; libav is linked explicitly so the test
  # still resolves the encode/mux symbols under a static-lib build (same reason
  # bagwiz_slam's gnss_projector_test re-links GeographicLib).
  ament_add_gtest(bagwiz_video_encoder_test test/core/video/video_encoder_test.cpp)
  target_link_libraries(bagwiz_video_encoder_test bagwiz_video PkgConfig::LIBAV)
endif()

ament_package()
