cmake_minimum_required(VERSION 3.16)
project(bagwiz_slam CXX)

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

find_package(ament_cmake REQUIRED)
find_package(bagwiz_base REQUIRED)
find_package(bagwiz_io REQUIRED)
find_package(bagwiz_msg REQUIRED)
find_package(bagwiz_image REQUIRED)
find_package(bagwiz_tf REQUIRED)
find_package(bagwiz_pointcloud REQUIRED)

# GeographicLib provides the WGS84 -> ENU / UTM projections used by
# gnss_projector.cpp. It is consumed PRIVATE — an implementation detail that
# must not leak into bagwiz_slam's export set. Two packagings exist and both
# are supported:
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

# Only the GLIM-free SLAM sources build at this stage. The GLIM-bearing TUs
# (cloud_mapper.cpp, warmup_fill.cpp production link, scan_match_fill.cpp,
# cuda_device.cpp, progress_bar.cpp, map_viewer.cpp, cloud_voxelize_gpu.cu) and
# the BAGWIZ_WITH_SLAM / BAGWIZ_WITH_SLAM_CUDA / BAGWIZ_WITH_MAP_VIEWER option
# blocks are wired by the follow-up SLAM/viewer task; the headers stay
# installed either way.
add_library(bagwiz_slam
  src/core/slam/cloud_filters.cpp
  src/core/slam/colorize_rasterizer_cpu.cpp
  src/core/slam/gnss_alignment.cpp
  src/core/slam/gnss_projector.cpp
  src/core/slam/gnss_sample.cpp
  src/core/slam/imu_sample.cpp
  src/core/slam/lidar_scan.cpp
  src/core/slam/map_colorizer.cpp
  src/core/slam/map_viewer_open.cpp
  src/core/slam/point_cloud_io.cpp
)
target_include_directories(bagwiz_slam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bagwiz_slam
  PUBLIC
    bagwiz_base::bagwiz_base
    bagwiz_io::bagwiz_io
    bagwiz_msg::bagwiz_msg
    bagwiz_image::bagwiz_image
    bagwiz_tf::bagwiz_tf
    bagwiz_pointcloud::bagwiz_pointcloud
)
# GeographicLib (WGS84 -> ENU/UTM projection) is private to gnss_projector.cpp.
target_include_directories(bagwiz_slam PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(bagwiz_slam PRIVATE ${GeographicLib_LIBRARIES})

install(TARGETS bagwiz_slam
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# GeographicLib is a private link dependency, but bagwiz_slam is a static
# archive: its imported target still shows up in the link interface as
# $<LINK_ONLY:...>, so downstream packages must be able to re-create that
# target when they find_package bagwiz_slam.
ament_export_dependencies(
  bagwiz_base
  bagwiz_io
  bagwiz_msg
  bagwiz_image
  bagwiz_tf
  bagwiz_pointcloud
  GeographicLib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages used by the schema-driven tests (gnss_sample_test
  # and imu_sample_test embed sensor_msgs schemas). These are not runtime deps
  # of bagwiz_slam — the messages are handled as schema text and CDR bytes —
  # so they only need to be on AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  ament_add_gtest(bagwiz_slam_lidar_scan_test test/core/slam/lidar_scan_test.cpp)
  target_link_libraries(bagwiz_slam_lidar_scan_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_point_cloud_io_test test/core/slam/point_cloud_io_test.cpp)
  target_link_libraries(bagwiz_slam_point_cloud_io_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_map_colorizer_test test/core/slam/map_colorizer_test.cpp)
  target_link_libraries(bagwiz_slam_map_colorizer_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_cloud_filters_test test/core/slam/cloud_filters_test.cpp)
  target_link_libraries(bagwiz_slam_cloud_filters_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_imu_sample_test test/core/slam/imu_sample_test.cpp)
  target_link_libraries(bagwiz_slam_imu_sample_test bagwiz_slam)

  # Backward-IMU-propagation math for the SLAM warmup-window fill. Eigen-based
  # but GLIM-free, so it compiles the TU directly and pulls only Eigen (header-only)
  # — it needs no SLAM build and runs in the core test suite.
  find_package(Eigen3 QUIET)
  if(Eigen3_FOUND)
    ament_add_gtest(bagwiz_slam_warmup_fill_test
      test/core/slam/warmup_fill_test.cpp
      src/core/slam/warmup_fill.cpp)
    target_link_libraries(bagwiz_slam_warmup_fill_test bagwiz_slam Eigen3::Eigen)
  endif()

  ament_add_gtest(bagwiz_slam_gnss_sample_test test/core/slam/gnss_sample_test.cpp)
  target_link_libraries(bagwiz_slam_gnss_sample_test bagwiz_slam)

  # gnss_projector lives in bagwiz_slam; GeographicLib is linked explicitly (as
  # the geo_pose_convert test does) since it is otherwise a private dependency.
  ament_add_gtest(bagwiz_slam_gnss_projector_test test/core/slam/gnss_projector_test.cpp)
  target_link_libraries(bagwiz_slam_gnss_projector_test bagwiz_slam ${GeographicLib_LIBRARIES})

  ament_add_gtest(bagwiz_slam_gnss_alignment_test test/core/slam/gnss_alignment_test.cpp)
  target_link_libraries(bagwiz_slam_gnss_alignment_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_map_viewer_open_test test/core/slam/map_viewer_open_test.cpp)
  target_link_libraries(bagwiz_slam_map_viewer_open_test bagwiz_slam)
endif()

ament_package()
