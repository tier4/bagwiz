cmake_minimum_required(VERSION 3.16)
project(bagwiz_pointcloud CXX)

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
find_package(bagwiz_io REQUIRED)
find_package(bagwiz_msg REQUIRED)
find_package(bagwiz_image REQUIRED)
find_package(bagwiz_tf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(bagwiz_pointcloud
  src/core/pointcloud/cloud_concat.cpp
  src/core/pointcloud/cloud_transform.cpp
  src/core/pointcloud/color_mapper.cpp
  src/core/pointcloud/color_propagation.cpp
  src/core/pointcloud/concat_sync.cpp
  src/core/pointcloud/deskew.cpp
  src/core/pointcloud/fetcher.cpp
  src/core/pointcloud/kdtree.cpp
  src/core/pointcloud/normals.cpp
  src/core/pointcloud/outlier_removal.cpp
  src/core/pointcloud/overlay.cpp
  src/core/pointcloud/point_time.cpp
  src/core/pointcloud/pointcloud2.cpp
  src/core/pointcloud/projector.cpp
  src/core/pointcloud/projector_helpers.cpp
  src/core/pointcloud/static_extrinsic.cpp
)
target_include_directories(bagwiz_pointcloud
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(bagwiz_pointcloud
  PUBLIC bagwiz_io::bagwiz_io bagwiz_msg::bagwiz_msg bagwiz_image::bagwiz_image bagwiz_tf::bagwiz_tf)
# ROS package dependencies of bagwiz_pointcloud. ament_target_dependencies
# links each package's include dirs and libraries, but newer (rolling) ROS
# distros removed it in favour of the per-package modern CMake targets.
# Humble/Jazzy still provide the macro and keep using it; where it is absent,
# link the exported ${pkg}_TARGETS instead (each was found via find_package
# above). Both packages appear in public headers (deskew.hpp,
# projector_helpers.hpp), so they are PUBLIC and exported below.
set(bagwiz_pointcloud_ros_deps
  geometry_msgs
  tf2
)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(bagwiz_pointcloud PUBLIC ${bagwiz_pointcloud_ros_deps})
else()
  foreach(bagwiz_dep IN LISTS bagwiz_pointcloud_ros_deps)
    target_link_libraries(bagwiz_pointcloud PUBLIC ${${bagwiz_dep}_TARGETS})
  endforeach()
endif()
# OpenCV is an implementation detail of the point-cloud overlay path
# (overlay.cpp). Keep it PRIVATE so it does not leak into the export set.
target_link_libraries(bagwiz_pointcloud PRIVATE ${OpenCV_LIBS})

install(TARGETS bagwiz_pointcloud
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(bagwiz_io bagwiz_msg bagwiz_image bagwiz_tf geometry_msgs tf2)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages used by the schema-driven tests (fetcher_test
  # embeds sensor_msgs schemas). These are not runtime deps of
  # bagwiz_pointcloud — the messages are handled as schema text and CDR
  # bytes — so they only need to be on AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  ament_add_gtest(bagwiz_pointcloud2_test test/core/pointcloud/pointcloud2_test.cpp)
  target_link_libraries(bagwiz_pointcloud2_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_cloud_transform_test test/core/pointcloud/cloud_transform_test.cpp)
  target_link_libraries(bagwiz_cloud_transform_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_cloud_concat_test test/core/pointcloud/cloud_concat_test.cpp)
  target_link_libraries(bagwiz_cloud_concat_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_concat_sync_test test/core/pointcloud/concat_sync_test.cpp)
  target_link_libraries(bagwiz_concat_sync_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_point_time_test test/core/pointcloud/point_time_test.cpp)
  target_link_libraries(bagwiz_point_time_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_deskew_test test/core/pointcloud/deskew_test.cpp)
  target_link_libraries(bagwiz_deskew_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_color_mapper_test test/core/pointcloud/color_mapper_test.cpp)
  target_link_libraries(bagwiz_color_mapper_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_projector_test test/core/pointcloud/projector_test.cpp)
  target_link_libraries(bagwiz_projector_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_overlay_test test/core/pointcloud/overlay_test.cpp)
  target_link_libraries(bagwiz_overlay_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_pointcloud_fetcher_test test/core/pointcloud/fetcher_test.cpp)
  target_link_libraries(bagwiz_pointcloud_fetcher_test bagwiz_pointcloud)

  # Pins build_point_cloud_index's manual-bounds and intensity paths (the
  # value-scan-free header path no command exercises end to end).
  ament_add_gtest(bagwiz_index_bounds_test test/core/pointcloud/index_bounds_test.cpp)
  target_link_libraries(bagwiz_index_bounds_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_static_extrinsic_test test/core/pointcloud/static_extrinsic_test.cpp)
  target_link_libraries(bagwiz_static_extrinsic_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_frame_match_test test/core/pointcloud/frame_match_test.cpp)
  target_link_libraries(bagwiz_frame_match_test bagwiz_pointcloud)

  ament_add_gtest(
    bagwiz_pointcloud_property_ranges_test test/core/pointcloud/property_ranges_test.cpp)
  target_link_libraries(bagwiz_pointcloud_property_ranges_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_pointcloud_kdtree_test test/core/pointcloud/kdtree_test.cpp)
  target_link_libraries(bagwiz_pointcloud_kdtree_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_pointcloud_normals_test test/core/pointcloud/normals_test.cpp)
  target_link_libraries(bagwiz_pointcloud_normals_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_pointcloud_color_propagation_test test/core/pointcloud/color_propagation_test.cpp)
  target_link_libraries(bagwiz_pointcloud_color_propagation_test bagwiz_pointcloud)

  ament_add_gtest(bagwiz_pointcloud_outlier_removal_test test/core/pointcloud/outlier_removal_test.cpp)
  target_link_libraries(bagwiz_pointcloud_outlier_removal_test bagwiz_pointcloud)
endif()

ament_package()
