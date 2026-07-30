cmake_minimum_required(VERSION 3.16)
project(bagwiz CXX)

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
# consumed only here (the `map` command group) and in bagwiz_slam (the GLIM
# wrapper library). SLAM is the default `pixi run build-full` (humble/jazzy):
# it builds the GLIM stack into install/<distro>/glim-deps via
# scripts/build-glim-deps.sh and passes -DBAGWIZ_WITH_SLAM=ON plus that prefix
# on CMAKE_PREFIX_PATH. (The `build-core` task leaves this OFF.)
option(BAGWIZ_WITH_SLAM "Build the in-process GLIM `bagwiz map slam` command" OFF)
# GPU fast path for `bagwiz map slam --gpu` (CUDA odometry + VGICP_GPU mapping +
# GPU export voxelization). OFF by default; implies BAGWIZ_WITH_SLAM. Enabled via
# `pixi run -e <distro>-cuda build-full` (humble-cuda/jazzy-cuda), which builds
# the CUDA GLIM stack into install/<distro>-cuda/glim-deps-cuda and passes
# -DBAGWIZ_WITH_SLAM_CUDA=ON plus the CUDA toolchain. A CPU build (`pixi run
# build-full` in a CPU env) leaves this OFF and links no CUDA.
option(BAGWIZ_WITH_SLAM_CUDA "GPU-accelerate `bagwiz map slam --gpu` (implies BAGWIZ_WITH_SLAM)" OFF)
if(BAGWIZ_WITH_SLAM_CUDA)
  set(BAGWIZ_WITH_SLAM ON CACHE BOOL "Implied by BAGWIZ_WITH_SLAM_CUDA" FORCE)
endif()
# The viewer (a cpp-httplib loopback server + embedded Three.js assets + a
# browser launch) is entirely GLIM-free, so it gets its own option that simply
# DEFAULTS to BAGWIZ_WITH_SLAM: a SLAM build (the default `pixi run build-full`
# on humble/jazzy) gains the viewer, a core build (`build-core` or
# BAGWIZ_WITH_SLAM=OFF) pulls in neither cpp-httplib nor the viewer code, and
# the viewer can be built on its own with -DBAGWIZ_WITH_MAP_VIEWER=ON (no GLIM
# needed). The implementation lives in bagwiz_slam; this package only forwards
# the macro that map_slam.cpp's --viewer call guards on.
option(BAGWIZ_WITH_MAP_VIEWER
  "Build the `map slam --viewer` browser map viewer (fetches cpp-httplib)"
  ${BAGWIZ_WITH_SLAM})

find_package(ament_cmake REQUIRED)

# CLI11 and fmt are not available as rosdep keys on all supported distros, so
# fetch them at configure time. Both are small, header-light libraries. This
# block must run BEFORE find_package(bagwiz_slam) below: in SLAM builds that
# package's config re-resolves the GLIM stack (find_dependency(glim) ->
# spdlog -> find_dependency(fmt CONFIG)), which would create the toolchain's
# imported fmt::fmt target first and collide with the FetchContent population
# here. The toolchain's fmt-config.cmake carries the reverse guard
# (NOT TARGET fmt::fmt), so populating ours first is safe in both directions.
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
# 4.0 still accepts, which is a hard error on newer toolchains. Give the fetched
# subprojects a compatibility floor. Older CMake (Humble/Jazzy) does not know
# this variable and simply ignores it.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
FetchContent_MakeAvailable(cli11 fmt rang)
unset(CMAKE_POLICY_VERSION_MINIMUM)

find_package(bagwiz_base REQUIRED)
find_package(bagwiz_io REQUIRED)
find_package(bagwiz_msg REQUIRED)
find_package(bagwiz_image REQUIRED)
find_package(bagwiz_video REQUIRED)
find_package(bagwiz_tf REQUIRED)
find_package(bagwiz_pointcloud REQUIRED)
find_package(bagwiz_tui REQUIRED)
find_package(bagwiz_bag REQUIRED)
find_package(bagwiz_slam REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
# cam_info_common.cpp (the shared internals of cam_info_recompute_p and
# cam_info_replace) round-trips typed messages through rmw_(de)serialize,
# which is declared in <rmw/rmw.h> but DEFINED in librmw_implementation.so,
# so both packages are needed at link time.
find_package(rmw_implementation REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
# yaml_cpp_vendor sets yaml-cpp_DIR so the real yaml-cpp package is found.
# yaml-cpp and zstd are exercised directly by the completion test's bag
# fixtures (compressed MCAP writers), not by any command source.
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
# zstd provider split upstream: Humble and Jazzy ship zstd_vendor; newer
# (rolling) ROS distros replaced it with zstd_cmake_module (a thin module over
# the system libzstd). Both expose the same zstd::zstd target, so try
# zstd_vendor first and fall back.
find_package(zstd_vendor QUIET)
if(NOT zstd_vendor_FOUND)
  find_package(zstd_cmake_module REQUIRED)
endif()
find_package(zstd REQUIRED)

# FFmpeg/libav is exercised directly by the `generate video` command test,
# which synthesizes image fixtures and inspects the encoded output. No
# ament/CMake package exists for it, so locate the libraries via pkg-config.
# pkg_check_modules creates the imported target PkgConfig::LIBAV spanning all
# four component libraries.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec libavformat libavutil libswscale)

# ---------------------------------------------------------------------------
# bagwiz: CLI executable. Installs to bin/ so users run it directly after
# sourcing the ROS setup. Command implementations live in the executable
# target so their static registrars pull into the final binary. The `map`
# command group sources (map.cpp, map_slam.cpp, map_viewer.cpp) join the
# executable in the BAGWIZ_WITH_SLAM block below.
# ---------------------------------------------------------------------------
add_executable(bagwiz
  src/main.cpp
  src/commands/registry.cpp
  src/commands/help_formatter.cpp
  src/commands/parse_error.cpp
  src/commands/cam_info.cpp
  src/commands/cam_info_common.cpp
  src/commands/cam_info_dump.cpp
  src/commands/cam_info_recompute_p.cpp
  src/commands/cam_info_replace.cpp
  src/commands/check.cpp
  src/commands/completion.cpp
  src/commands/convert.cpp
  src/commands/generate.cpp
  src/commands/generate_video.cpp
  src/commands/generate_video_common.cpp
  src/commands/joke.cpp
  src/commands/ls.cpp
  src/commands/pcd.cpp
  src/commands/pcd_concat.cpp
  src/commands/pcd_concat_common.cpp
  src/commands/pcd_undistort.cpp
  src/commands/pcd_undistort_common.cpp
  src/commands/walk.cpp
  src/commands/walk_bag.cpp
  src/commands/walk_cursor.cpp
  src/commands/walk_frame.cpp
  src/commands/walk_overlay.cpp
  src/commands/walk_overlay_scan.cpp
  src/commands/walk_preview.cpp
  src/commands/walk_save.cpp
  src/commands/traj.cpp
  src/commands/traj_common.cpp
  src/commands/tf.cpp
  src/commands/tf_static_cp.cpp
  src/commands/topic.cpp
  src/commands/topic_drop.cpp
  src/commands/topic_keep.cpp
  src/commands/topic_rename.cpp
  src/commands/trim.cpp
  src/commands/trim_stamp.cpp
)
target_include_directories(bagwiz PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
# sensor_msgs is needed only by src/commands/cam_info_common.cpp (shared by
# cam_info_replace.cpp and cam_info_recompute_p.cpp), which builds a typed
# sensor_msgs/msg/CameraInfo to round-trip through rmw_(de)serialize.
# It is not a dependency of any bagwiz library, so link it on the
# executable directly. rmw, tf2, and geometry_msgs headers are likewise used
# by command sources directly.
target_link_libraries(bagwiz
  PRIVATE
    bagwiz_base::bagwiz_base
    bagwiz_io::bagwiz_io
    bagwiz_msg::bagwiz_msg
    bagwiz_image::bagwiz_image
    bagwiz_video::bagwiz_video
    bagwiz_tf::bagwiz_tf
    bagwiz_pointcloud::bagwiz_pointcloud
    bagwiz_tui::bagwiz_tui
    bagwiz_bag::bagwiz_bag
    bagwiz_slam::bagwiz_slam
    CLI11::CLI11
    fmt::fmt
    rang
    ${OpenCV_LIBS}
    ${sensor_msgs_TARGETS}
    ${rmw_TARGETS}
    ${rmw_implementation_TARGETS}
    ${tf2_TARGETS}
    ${geometry_msgs_TARGETS}
)

# --- Optional in-process GLIM SLAM command (`bagwiz map slam`) ----------------
# The GLIM-bearing objects live in bagwiz_slam's bagwiz_slam_glim static
# library (exported in bagwiz_slam's export set); the command sources here are
# GLIM-free but link that library plus the GLIM stack directly, mirroring the
# pre-split package.
if(BAGWIZ_WITH_SLAM)
  find_package(glim CONFIG REQUIRED)
  find_package(gtsam_points CONFIG REQUIRED)
  find_package(GTSAM CONFIG REQUIRED)

  target_sources(bagwiz PRIVATE
    src/commands/map.cpp
    src/commands/map_slam.cpp
    src/commands/map_slam_colorize.cpp
    src/commands/map_slam_mapping.cpp
    src/commands/map_slam_threads.cpp
    src/commands/map_viewer.cpp)
  target_link_libraries(bagwiz PRIVATE
    bagwiz_slam::bagwiz_slam_glim
    glim::glim
    gtsam_points::gtsam_points
    gtsam)
  target_compile_definitions(bagwiz PRIVATE BAGWIZ_WITH_SLAM)

  # GPU fast path: the .cu voxelizer stays in bagwiz_slam_glim; the executable
  # only links the CUDA runtime for the GLIM stack's own CUDA TUs and defines
  # the macro that switches on the CUDA code paths.
  if(BAGWIZ_WITH_SLAM_CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_link_libraries(bagwiz PRIVATE CUDA::cudart)
    target_compile_definitions(bagwiz PRIVATE BAGWIZ_WITH_SLAM_CUDA)
  endif()
endif()

# map_slam.cpp lives on the executable and guards its --viewer call on this
# macro (the viewer server itself is compiled into bagwiz_slam, which defines
# it there privately).
if(BAGWIZ_WITH_MAP_VIEWER)
  target_compile_definitions(bagwiz PRIVATE BAGWIZ_WITH_MAP_VIEWER)
endif()

install(TARGETS bagwiz RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Standard message packages exercised by the command tests (the tests
  # round-trip typed messages through rmw_(de)serialize and the introspection
  # backend, which dlopens the matching `.so` files at runtime), so they only
  # need to be on AMENT_PREFIX_PATH at test time.
  find_package(std_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  # Every command test links the full sibling stack (the same libraries the
  # executable links) so command sources compiled directly into a test behave
  # exactly as in the production binary.
  set(bagwiz_test_libs
    bagwiz_base::bagwiz_base
    bagwiz_io::bagwiz_io
    bagwiz_msg::bagwiz_msg
    bagwiz_image::bagwiz_image
    bagwiz_video::bagwiz_video
    bagwiz_tf::bagwiz_tf
    bagwiz_pointcloud::bagwiz_pointcloud
    bagwiz_tui::bagwiz_tui
    bagwiz_bag::bagwiz_bag
    bagwiz_slam::bagwiz_slam
  )

  # completion.cpp carries a BAGWIZ_REGISTER_COMMAND registrar, so the test
  # also compiles registry.cpp (which otherwise lives only in the `bagwiz`
  # executable — the registrars must not sit in a library, or the linker
  # strips them).
  ament_add_gtest(bagwiz_completion_test
    test/commands/completion_test.cpp
    src/commands/completion.cpp
    src/commands/registry.cpp)
  target_include_directories(bagwiz_completion_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # The test builds compressed MCAP fixtures itself, so it uses yaml-cpp and
  # zstd directly in addition to the CLI libraries.
  target_link_libraries(
    bagwiz_completion_test ${bagwiz_test_libs} CLI11::CLI11 fmt::fmt yaml-cpp zstd::zstd)

  ament_add_gtest(bagwiz_help_formatter_test
    test/commands/help_formatter_test.cpp
    src/commands/help_formatter.cpp)
  target_include_directories(bagwiz_help_formatter_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_help_formatter_test ${bagwiz_test_libs} CLI11::CLI11)

  ament_add_gtest(bagwiz_parse_error_test
    test/commands/parse_error_test.cpp
    src/commands/parse_error.cpp)
  target_include_directories(bagwiz_parse_error_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_parse_error_test ${bagwiz_test_libs} CLI11::CLI11)

  ament_add_gtest(bagwiz_tf_static_cp_test
    test/commands/tf_static_cp_test.cpp
    src/commands/tf_static_cp.cpp)
  target_include_directories(bagwiz_tf_static_cp_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_tf_static_cp_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_pcd_concat_command_test
    test/commands/pcd_concat_test.cpp
    src/commands/pcd_concat.cpp
    src/commands/pcd_concat_common.cpp)
  target_include_directories(bagwiz_pcd_concat_command_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_pcd_concat_command_test ${bagwiz_test_libs} fmt::fmt)

  # pcd_concat_common.hpp lives in src/commands (CLI-internal, not installed),
  # so this test adds src/commands to its include path (cam_info_common idiom).
  ament_add_gtest(bagwiz_pcd_concat_common_test
    test/commands/pcd_concat_common_test.cpp
    src/commands/pcd_concat_common.cpp)
  target_include_directories(bagwiz_pcd_concat_common_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_pcd_concat_common_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_pcd_undistort_command_test
    test/commands/pcd_undistort_test.cpp
    src/commands/pcd_undistort.cpp
    src/commands/pcd_undistort_common.cpp)
  target_include_directories(bagwiz_pcd_undistort_command_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_pcd_undistort_command_test ${bagwiz_test_libs} fmt::fmt)

  # pcd_undistort_common.hpp lives in src/commands (CLI-internal, not
  # installed), so this test adds src/commands to its include path
  # (pcd_concat_common idiom). rmw + geometry/nav_msgs are linked for the
  # typed round-trip through rmw_serialize (PoseStamped / Odometry fixtures).
  ament_add_gtest(bagwiz_pcd_undistort_common_test
    test/commands/pcd_undistort_common_test.cpp
    src/commands/pcd_undistort_common.cpp)
  target_include_directories(bagwiz_pcd_undistort_common_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(
    bagwiz_pcd_undistort_common_test ${bagwiz_test_libs} fmt::fmt
    ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${rmw_TARGETS} ${rmw_implementation_TARGETS})

  # smoke_test touches Registry::instance() directly, so it compiles
  # registry.cpp (which otherwise lives only in the `bagwiz` executable).
  ament_add_gtest(bagwiz_smoke_test test/smoke_test.cpp src/commands/registry.cpp)
  target_include_directories(bagwiz_smoke_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_smoke_test ${bagwiz_test_libs})

  ament_add_gtest(bagwiz_topic_drop_test
    test/commands/topic_drop_test.cpp
    src/commands/topic_drop.cpp)
  target_include_directories(bagwiz_topic_drop_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_topic_drop_test ${bagwiz_test_libs} fmt::fmt)

  # traj_common.hpp lives in src/commands (CLI-internal, not installed), so
  # this test adds src/commands to its include path (pcd_concat_common idiom).
  ament_add_gtest(bagwiz_traj_common_test
    test/commands/traj_common_test.cpp
    src/commands/traj_common.cpp)
  target_include_directories(bagwiz_traj_common_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_traj_common_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_topic_keep_test
    test/commands/topic_keep_test.cpp
    src/commands/topic_keep.cpp)
  target_include_directories(bagwiz_topic_keep_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_topic_keep_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_topic_rename_test
    test/commands/topic_rename_test.cpp
    src/commands/topic_rename.cpp)
  target_include_directories(bagwiz_topic_rename_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(bagwiz_topic_rename_test ${bagwiz_test_libs} fmt::fmt)

  # trim.cpp is a single TU carrying both run_trim and the TrimCommand
  # registrar + CLI11 wiring, so the test also compiles registry.cpp and
  # links CLI11 (completion_test idiom). trim_stamp.hpp is CLI-internal, so
  # the test adds src/commands to its include path (traj_common idiom).
  ament_add_gtest(bagwiz_trim_test
    test/commands/trim_test.cpp
    src/commands/trim.cpp
    src/commands/trim_stamp.cpp
    src/commands/registry.cpp)
  target_include_directories(bagwiz_trim_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_trim_test ${bagwiz_test_libs} CLI11::CLI11 fmt::fmt)

  ament_add_gtest(bagwiz_cam_info_recompute_p_test
    test/commands/cam_info_recompute_p_test.cpp
    src/commands/cam_info_recompute_p.cpp
    src/commands/cam_info_common.cpp)
  target_include_directories(bagwiz_cam_info_recompute_p_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(
    bagwiz_cam_info_recompute_p_test ${bagwiz_test_libs} fmt::fmt
    ${sensor_msgs_TARGETS} ${rmw_TARGETS} ${rmw_implementation_TARGETS})

  ament_add_gtest(bagwiz_cam_info_replace_test
    test/commands/cam_info_replace_test.cpp
    src/commands/cam_info_replace.cpp
    src/commands/cam_info_common.cpp)
  target_include_directories(bagwiz_cam_info_replace_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(
    bagwiz_cam_info_replace_test ${bagwiz_test_libs} fmt::fmt
    ${sensor_msgs_TARGETS} ${rmw_TARGETS} ${rmw_implementation_TARGETS})

  ament_add_gtest(bagwiz_cam_info_dump_test
    test/commands/cam_info_dump_test.cpp
    src/commands/cam_info_dump.cpp)
  target_include_directories(bagwiz_cam_info_dump_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(
    bagwiz_cam_info_dump_test ${bagwiz_test_libs} fmt::fmt
    ${sensor_msgs_TARGETS} ${rmw_TARGETS} ${rmw_implementation_TARGETS})

  # cam_info_common.hpp lives in src/commands (CLI-internal, not installed), so
  # this test adds src/commands to its include path in addition to include/.
  ament_add_gtest(bagwiz_cam_info_common_test
    test/commands/cam_info_common_test.cpp
    src/commands/cam_info_common.cpp)
  target_include_directories(bagwiz_cam_info_common_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(
    bagwiz_cam_info_common_test ${bagwiz_test_libs} fmt::fmt
    ${sensor_msgs_TARGETS} ${rmw_TARGETS} ${rmw_implementation_TARGETS})

  ament_add_gtest(bagwiz_generate_video_test
    test/commands/generate_video_test.cpp
    src/commands/generate_video.cpp
    src/commands/generate_video_common.cpp)
  target_include_directories(bagwiz_generate_video_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(
    bagwiz_generate_video_test ${bagwiz_test_libs} fmt::fmt
    bagwiz_image::bagwiz_image_test_fixture PkgConfig::LIBAV ${OpenCV_LIBS})

  # generate_video_common.hpp lives in src/commands (CLI-internal, not
  # installed), so this test adds src/commands to its include path
  # (pcd_concat_common idiom).
  ament_add_gtest(bagwiz_generate_video_common_test
    test/commands/generate_video_common_test.cpp
    src/commands/generate_video_common.cpp)
  target_include_directories(bagwiz_generate_video_common_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(
    bagwiz_generate_video_common_test ${bagwiz_test_libs} fmt::fmt ${OpenCV_LIBS})

  # The walk_*.hpp headers live in src/commands (CLI-internal, not installed),
  # so the walk unit tests add src/commands to their include path
  # (pcd_concat_common idiom).
  ament_add_gtest(bagwiz_walk_cursor_test
    test/commands/walk_cursor_test.cpp
    src/commands/walk_cursor.cpp)
  target_include_directories(bagwiz_walk_cursor_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_walk_cursor_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_walk_bag_test
    test/commands/walk_bag_test.cpp
    src/commands/walk_bag.cpp)
  target_include_directories(bagwiz_walk_bag_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_walk_bag_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_walk_frame_test
    test/commands/walk_frame_test.cpp
    src/commands/walk_frame.cpp
    src/commands/walk_cursor.cpp)
  target_include_directories(bagwiz_walk_frame_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_walk_frame_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_walk_scroll_hint_test
    test/commands/walk_scroll_hint_test.cpp
    src/commands/walk_frame.cpp
    src/commands/walk_cursor.cpp)
  target_include_directories(bagwiz_walk_scroll_hint_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_walk_scroll_hint_test ${bagwiz_test_libs} fmt::fmt)

  ament_add_gtest(bagwiz_walk_save_test
    test/commands/walk_save_test.cpp
    src/commands/walk_save.cpp)
  target_include_directories(bagwiz_walk_save_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_walk_save_test ${bagwiz_test_libs} fmt::fmt)

  # walk.cpp carries a BAGWIZ_REGISTER_COMMAND registrar, so the test also
  # compiles registry.cpp (completion_test idiom).
  ament_add_gtest(bagwiz_walk_test
    test/commands/walk_test.cpp
    src/commands/walk.cpp
    src/commands/walk_bag.cpp
    src/commands/walk_cursor.cpp
    src/commands/walk_frame.cpp
    src/commands/walk_overlay.cpp
    src/commands/walk_overlay_scan.cpp
    src/commands/walk_preview.cpp
    src/commands/walk_save.cpp
    src/commands/registry.cpp)
  target_include_directories(bagwiz_walk_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(
    bagwiz_walk_test ${bagwiz_test_libs} CLI11::CLI11 fmt::fmt ${OpenCV_LIBS} ${tf2_TARGETS})

  ament_add_gtest(bagwiz_walk_overlay_scan_test
    test/commands/walk_overlay_scan_test.cpp
    src/commands/walk_overlay_scan.cpp)
  target_include_directories(bagwiz_walk_overlay_scan_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(
    bagwiz_walk_overlay_scan_test ${bagwiz_test_libs} fmt::fmt ${tf2_TARGETS})

  # The map_slam_* headers live in src/commands (CLI-internal, not installed),
  # so these tests add src/commands to their include path (pcd_concat_common
  # idiom).
  #
  # map_slam_mapping.cpp drives CloudMapper::finish, so its test compiles only
  # under BAGWIZ_WITH_SLAM and links the GLIM-bearing library like the
  # executable does.
  if(BAGWIZ_WITH_SLAM)
    ament_add_gtest(bagwiz_map_slam_mapping_test
      test/commands/map_slam_mapping_test.cpp
      src/commands/map_slam_mapping.cpp
      src/commands/map_slam_threads.cpp)
    target_include_directories(bagwiz_map_slam_mapping_test
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
    target_link_libraries(bagwiz_map_slam_mapping_test
      ${bagwiz_test_libs} fmt::fmt
      bagwiz_slam::bagwiz_slam_glim glim::glim gtsam_points::gtsam_points gtsam)
  endif()

  # map_slam_colorize.cpp is GLIM-free: it uses only the plain bagwiz_slam
  # colorizer types plus the leaf map_slam_threads.cpp. The executable compiles
  # it only under BAGWIZ_WITH_SLAM (alongside map_slam.cpp), but this test
  # compiles those two units directly and links no GLIM stack, so the colorize
  # rules stay covered in a core build too.
  ament_add_gtest(bagwiz_map_slam_colorize_test
    test/commands/map_slam_colorize_test.cpp
    src/commands/map_slam_colorize.cpp
    src/commands/map_slam_threads.cpp)
  target_include_directories(bagwiz_map_slam_colorize_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/commands)
  target_link_libraries(bagwiz_map_slam_colorize_test ${bagwiz_test_libs} fmt::fmt)
endif()

ament_package()
