cmake_minimum_required(VERSION 3.16)
project(bagwiz_image CXX)

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
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# FFmpeg/libav (conda-forge `ffmpeg`) backs image_decoder.cpp / image_encoder.cpp
# (JPEG/PNG decode/encode). No ament/CMake package exists for it, so locate the
# libraries via pkg-config. It is consumed PRIVATE (the codecs are hidden behind
# pimpls), so libav never enters the export set. pkg_check_modules creates the
# imported target PkgConfig::LIBAV spanning all four component libraries.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec libavformat libavutil libswscale)

add_library(bagwiz_image
  src/core/image/camera_calibration_yaml.cpp
  src/core/image/camera_distortion.cpp
  src/core/image/camera_info.cpp
  src/core/image/camera_info_resolver.cpp
  src/core/image/compressed_image.cpp
  src/core/image/gradient.cpp
  src/core/image/image_decoder.cpp
  src/core/image/image_encoder.cpp
  src/core/image/packed_raster.cpp
  src/core/image/projection_matrix.cpp
  src/core/image/raw_image.cpp
  src/core/image/sampling.cpp
  src/core/image/srgb.cpp
  src/core/image/undistort.cpp
)
target_include_directories(bagwiz_image
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bagwiz_image PUBLIC bagwiz_io::bagwiz_io bagwiz_msg::bagwiz_msg)
# yaml-cpp (camera_calibration_yaml.cpp) and OpenCV (undistort.cpp /
# projection_matrix.cpp) are implementation details behind pimpls; they leak
# into the static archive's link interface only as plain `-l` items, which the
# conda toolchain resolves without targets. libav is linked by absolute path
# (LIBAV_LINK_LIBRARIES) so no PkgConfig::LIBAV target reference leaks into
# the export set — downstream packages never re-create that target.
target_include_directories(bagwiz_image PRIVATE ${LIBAV_INCLUDE_DIRS})
target_link_libraries(
  bagwiz_image PRIVATE yaml-cpp ${OpenCV_LIBS} ${LIBAV_LINK_LIBRARIES})

# bagwiz_image_test_fixture is a shared test utility, not production code: it
# synthesizes JPEG/PNG fixtures with libav and is used by this package's
# image_decoder_test and by the CLI package's generate_video_test. It is built
# unconditionally and installed/exported (unlike a BUILD_TESTING-only helper)
# so the CLI package can link it after find_package(bagwiz_image). libav is
# again linked by absolute path to keep target references out of the export.
add_library(bagwiz_image_test_fixture STATIC test/core/image/image_fixture.cpp)
target_include_directories(bagwiz_image_test_fixture
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    $<INSTALL_INTERFACE:include/bagwiz_image_test_fixture>
  PRIVATE
    ${LIBAV_INCLUDE_DIRS}
)
target_link_libraries(
  bagwiz_image_test_fixture PUBLIC bagwiz_image PRIVATE ${LIBAV_LINK_LIBRARIES})

install(TARGETS bagwiz_image bagwiz_image_test_fixture
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
# Installed under include/bagwiz_image_test_fixture/ so the build-tree include
# spelling ("core/image/image_fixture.hpp") keeps working for installed
# consumers of the exported fixture target.
install(FILES test/core/image/image_fixture.hpp
  DESTINATION include/bagwiz_image_test_fixture/core/image)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(bagwiz_io bagwiz_msg)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages used by the schema-driven tests
  # (camera_info_resolver_test and the raw/compressed image round-trips embed
  # sensor_msgs schemas). These are not runtime deps of bagwiz_image — the
  # messages are handled as schema text and CDR bytes — so they only need to
  # be on AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  ament_add_gtest(bagwiz_raw_image_test test/core/image/raw_image_test.cpp)
  target_link_libraries(bagwiz_raw_image_test bagwiz_image)

  ament_add_gtest(bagwiz_packed_raster_test test/core/image/packed_raster_test.cpp)
  target_link_libraries(bagwiz_packed_raster_test bagwiz_image)

  ament_add_gtest(bagwiz_camera_info_test test/core/image/camera_info_test.cpp)
  target_link_libraries(bagwiz_camera_info_test bagwiz_image)

  ament_add_gtest(
    bagwiz_camera_calibration_yaml_test
    test/core/image/camera_calibration_yaml_test.cpp)
  target_link_libraries(bagwiz_camera_calibration_yaml_test bagwiz_image)

  ament_add_gtest(bagwiz_image_undistort_test test/core/image/undistort_test.cpp)
  target_link_libraries(bagwiz_image_undistort_test bagwiz_image ${OpenCV_LIBS})

  ament_add_gtest(
    bagwiz_projection_matrix_test
    test/core/image/projection_matrix_test.cpp)
  target_link_libraries(bagwiz_projection_matrix_test bagwiz_image ${OpenCV_LIBS})

  ament_add_gtest(bagwiz_compressed_image_test test/core/image/compressed_image_test.cpp)
  target_link_libraries(bagwiz_compressed_image_test bagwiz_image)

  ament_add_gtest(bagwiz_image_sampling_test test/core/image/sampling_test.cpp)
  target_link_libraries(bagwiz_image_sampling_test bagwiz_image)

  ament_add_gtest(bagwiz_image_srgb_test test/core/image/srgb_test.cpp)
  target_link_libraries(bagwiz_image_srgb_test bagwiz_image)

  ament_add_gtest(bagwiz_image_gradient_test test/core/image/gradient_test.cpp)
  target_link_libraries(bagwiz_image_gradient_test bagwiz_image)

  # image_decoder lives in bagwiz_image; libav is linked explicitly so the test
  # can both drive the decoder and synthesize JPEG/PNG fixtures with libav
  # (same reason video_encoder_test re-links it). The fixture helper is shared
  # with the CLI package via the exported bagwiz_image_test_fixture target.
  ament_add_gtest(bagwiz_image_decoder_test test/core/image/image_decoder_test.cpp)
  target_link_libraries(
    bagwiz_image_decoder_test bagwiz_image bagwiz_image_test_fixture PkgConfig::LIBAV)

  # image_encoder lives in bagwiz_image; libav is linked explicitly so the test
  # resolves the encode symbols under a static-lib build (same reason
  # image_decoder_test re-links it). The round-trip checks decode the PNG it
  # produces, so the decoder path is exercised too.
  ament_add_gtest(bagwiz_image_encoder_test test/core/image/image_encoder_test.cpp)
  target_link_libraries(bagwiz_image_encoder_test bagwiz_image PkgConfig::LIBAV)

  ament_add_gtest(bagwiz_camera_info_resolver_test test/core/image/camera_info_resolver_test.cpp)
  target_link_libraries(bagwiz_camera_info_resolver_test bagwiz_image)
endif()

ament_package()
