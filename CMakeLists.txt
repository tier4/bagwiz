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
# `zstd::zstd` target. Humble and Jazzy ship zstd_vendor; newer (rolling) ROS
# distros replaced it with zstd_cmake_module (a thin module over the system
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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
# nlohmann_json (header-only) backs `tf static --json`. Used only inside
# bagwiz_core's tf_transform_format.cpp, so it is linked PRIVATE and does
# not leak into the export set.
find_package(nlohmann_json REQUIRED)

# GeographicLib provides the WGS84 -> ENU / UTM projections used by
# `convert msg geo` (geo_pose_convert.cpp). It is consumed PRIVATE — an
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

# FFmpeg/libav (conda-forge `ffmpeg`) backs `bagwiz generate video`
# (src/core/video/video_encoder.cpp): muxing, H.264/MJPEG encoding, and pixel
# conversion. No ament/CMake package exists for it, so locate the libraries via
# pkg-config. It is consumed PRIVATE by bagwiz_core (the encoder is hidden behind
# a pimpl), so libav never enters the export set — the same approach used for
# GeographicLib above. pkg_check_modules creates the imported target
# PkgConfig::LIBAV spanning all four component libraries.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec libavformat libavutil libswscale)

# libsixel (conda-forge `libsixel`) backs the Sixel backend of `bagwiz walk`'s
# image preview (src/core/tui/image/terminal_image_renderer.cpp). Its pkg-config
# file lists optional Requires.private integrations (libcurl, libjpeg, libpng,
# gdk-pixbuf-2.0) whose .pc files are not installed in the env, so
# `pkg_check_modules(... REQUIRED)` would fail dependency resolution even though
# `-lsixel` itself links and loads fine (the libsixel runtime .so deps are
# present). Locate the library and header directly instead. Consumed PRIVATE
# (every libsixel symbol stays inside the renderer .cpp) so it never enters
# bagwiz_core's export set.
find_library(SIXEL_LIBRARY NAMES sixel REQUIRED)
find_path(SIXEL_INCLUDE_DIR NAMES sixel.h REQUIRED)

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
# 4.0 still accepts, which is a hard error on newer toolchains. Give the fetched
# subprojects a compatibility floor. Older CMake (Humble/Jazzy) does not know
# this variable and simply ignores it.
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
  src/core/pipeline/stage_profiler.cpp
  src/core/pipeline/rewrite_backend.cpp
  src/core/pipeline/sequential_backend.cpp
  src/core/pipeline/topic_router.cpp
  src/core/pipeline/pipelined_backend.cpp
  src/core/pipeline/backend_select.cpp
  src/core/topic_match.cpp
  src/core/camera_info_resolver.cpp
  src/core/broken_bag.cpp
  src/core/tf_chain.cpp
  src/core/tf_merge_check.cpp
  src/core/tf_message_wire.cpp
  src/core/tf_transform_format.cpp
  src/core/tf_value_extract.cpp
  src/core/tf_buffer_loader.cpp
  src/core/tf_walk_timeline.cpp
  src/core/trajectory.cpp
  src/core/msg_convert/geo_pose_convert.cpp
  src/core/image/raw_image.cpp
  src/core/image/compressed_image.cpp
  src/core/image/image_decoder.cpp
  src/core/image/image_encoder.cpp
  src/core/image/camera_info.cpp
  src/core/image/camera_calibration_yaml.cpp
  src/core/image/packed_raster.cpp
  src/core/image/undistort.cpp
  src/core/pointcloud/pointcloud2.cpp
  src/core/pointcloud/color_mapper.cpp
  src/core/pointcloud/projector.cpp
  src/core/pointcloud/fetcher.cpp
  src/core/pointcloud/overlay.cpp
  src/core/pointcloud/projector_helpers.cpp
  src/core/slam/lidar_scan.cpp
  src/core/slam/imu_sample.cpp
  src/core/slam/gnss_sample.cpp
  src/core/slam/gnss_projector.cpp
  src/core/slam/gnss_alignment.cpp
  src/core/slam/point_cloud_io.cpp
  src/core/slam/cloud_filters.cpp
  src/core/slam/map_viewer_open.cpp
  src/core/video/frame_rate.cpp
  src/core/video/video_encoder.cpp
  src/core/tui/layout.cpp
  src/core/tui/pager.cpp
  src/core/tui/renderer.cpp
  src/core/tui/screen_scope.cpp
  src/core/tui/signal_handler.cpp
  src/core/tui/width.cpp
  src/core/tui/word_wrap.cpp
  src/core/tui/image/terminal_image_caps.cpp
  src/core/tui/image/terminal_image_renderer.cpp
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
# package's include dirs and libraries, but newer (rolling) ROS distros removed
# it in favour of the per-package modern CMake targets. Humble/Jazzy still
# provide the macro and keep using it; where it is absent, link the exported
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
# Threads::Threads supplies -pthread for the std::thread in PipelinedBackend.
find_package(Threads REQUIRED)
target_link_libraries(bagwiz_core PUBLIC yaml-cpp SQLite::SQLite3 zstd::zstd Threads::Threads ${CMAKE_DL_LIBS})
# GeographicLib (WGS84 -> ENU/UTM projection) is private to geo_pose_convert.cpp.
target_include_directories(bagwiz_core PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(bagwiz_core PRIVATE ${GeographicLib_LIBRARIES})
# nlohmann_json is an implementation detail of tf_transform_format.cpp (the
# public header returns std::string), so keep it PRIVATE and out of the
# export set.
target_link_libraries(bagwiz_core PRIVATE nlohmann_json::nlohmann_json)
# libav (FFmpeg) is an implementation detail of video_encoder.cpp (the public
# header returns only std types behind a pimpl), so keep it PRIVATE and out of
# the export set.
target_link_libraries(bagwiz_core PRIVATE PkgConfig::LIBAV)
# libsixel is an implementation detail of terminal_image_renderer.cpp (the public
# header returns only std types), so keep it PRIVATE and out of the export set.
target_link_libraries(bagwiz_core PRIVATE ${SIXEL_LIBRARY})
target_include_directories(bagwiz_core PRIVATE ${SIXEL_INCLUDE_DIR})
# OpenCV is an implementation detail of the shared undistortion helper and the
# `generate video --undistort` / point-cloud overlay paths. Keep it PRIVATE so
# it does not leak into bagwiz_core's export set.
find_package(OpenCV REQUIRED)
target_link_libraries(bagwiz_core PRIVATE ${OpenCV_LIBS})
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
  src/commands/help_formatter.cpp
  src/commands/cam_info.cpp
  src/commands/cam_info_replace.cpp
  src/commands/check.cpp
  src/commands/completion.cpp
  src/commands/convert.cpp
  src/commands/convert_msg_geo.cpp
  src/commands/generate.cpp
  src/commands/generate_video.cpp
  src/commands/joke.cpp
  src/commands/ls.cpp
  src/commands/walk.cpp
  src/commands/traj.cpp
  src/commands/tf.cpp
  src/commands/tf_walk.cpp
  src/commands/tf_static_cp.cpp
  src/commands/topic.cpp
  src/commands/topic_drop.cpp
  src/commands/topic_keep.cpp
  src/commands/topic_rename.cpp
)
# sensor_msgs is needed only by src/commands/cam_info_replace.cpp, which builds
# a typed sensor_msgs/msg/CameraInfo to round-trip through rmw_(de)serialize.
# It is not a bagwiz_core dependency, so link it on the executable directly.
target_link_libraries(bagwiz PRIVATE bagwiz_core CLI11::CLI11 fmt::fmt rang ${OpenCV_LIBS} ${sensor_msgs_TARGETS})

# --- Optional in-process GLIM SLAM command (`bagwiz map slam`) ----------------
# SLAM is the default `pixi run build-full` (humble/jazzy): it builds the GLIM stack
# into install/<distro>/glim-deps via scripts/build-glim-deps.sh and passes
# -DBAGWIZ_WITH_SLAM=ON plus that prefix on CMAKE_PREFIX_PATH. (The `build-core`
# task leaves this OFF.) The
# GLIM types stay confined to cloud_mapper.cpp (pimpl); the
# `map` command group (map.cpp) and its `slam` action (map_slam.cpp) are GLIM-free.
option(BAGWIZ_WITH_SLAM "Build the in-process GLIM `bagwiz map slam` command" OFF)
# GPU fast path for `bagwiz map slam --gpu` (CUDA odometry + VGICP_GPU mapping +
# GPU export voxelization). OFF by default; implies BAGWIZ_WITH_SLAM. Enabled via
# `pixi run -e <distro>-cuda build-full` (humble-cuda/jazzy-cuda), which builds the
# CUDA GLIM stack into install/<distro>-cuda/glim-deps-cuda and passes
# -DBAGWIZ_WITH_SLAM_CUDA=ON plus the CUDA toolchain. A CPU build (`pixi run
# build-full` in a CPU env) leaves this OFF and links no CUDA.
option(BAGWIZ_WITH_SLAM_CUDA "GPU-accelerate `bagwiz map slam --gpu` (implies BAGWIZ_WITH_SLAM)" OFF)
if(BAGWIZ_WITH_SLAM_CUDA)
  set(BAGWIZ_WITH_SLAM ON CACHE BOOL "Implied by BAGWIZ_WITH_SLAM_CUDA" FORCE)
endif()
if(BAGWIZ_WITH_SLAM)
  find_package(glim CONFIG REQUIRED)
  find_package(gtsam_points CONFIG REQUIRED)
  # GTSAM is pulled transitively by gtsam_points, but cloud_mapper.cpp now uses
  # GTSAM directly (PoseTranslationPrior / noise models for the GNSS constraint),
  # so depend on it explicitly for its headers and the `gtsam` link target.
  find_package(GTSAM CONFIG REQUIRED)

  # Header-only progress bar for `map slam` (p-ranav/indicators). indicators
  # auto-detects it is a subproject (PROJECT_NAME already set) and skips its own
  # install()/export rules, so unlike cpp-httplib below it needs no install
  # toggle and links cleanly into the (non-exported) bagwiz executable.
  FetchContent_Declare(
    indicators
    GIT_REPOSITORY https://github.com/p-ranav/indicators.git
    GIT_TAG v2.3
    GIT_SHALLOW TRUE
  )
  set(INDICATORS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(INDICATORS_SAMPLES OFF CACHE BOOL "" FORCE)
  set(INDICATORS_DEMO OFF CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  FetchContent_MakeAvailable(indicators)
  unset(CMAKE_POLICY_VERSION_MINIMUM)

  target_sources(bagwiz PRIVATE
    src/core/slam/cloud_mapper.cpp
    src/core/slam/cuda_device.cpp
    src/core/slam/progress_bar.cpp
    src/commands/map.cpp
    src/commands/map_filter.cpp
    src/commands/map_slam.cpp
    src/commands/map_viewer.cpp)
  target_link_libraries(bagwiz PRIVATE glim::glim gtsam_points::gtsam_points gtsam)
  # Header-only: pull indicators' include dir as SYSTEM (not a link) so its own
  # -Wshadow chatter stays out of our build, mirroring httplib below.
  target_include_directories(bagwiz SYSTEM PRIVATE
    $<TARGET_PROPERTY:indicators::indicators,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(bagwiz PRIVATE BAGWIZ_WITH_SLAM)

  # GPU fast path: link the CUDA runtime, compile the GPU export voxelizer, and
  # define the macro that switches on the CUDA code paths. gtsam_points_cuda is
  # pulled in transitively via gtsam_points::gtsam_points (once the deps are
  # CUDA-built), so bagwiz needs only CUDA::cudart directly for its own TUs
  # (cuda_device.cpp + cloud_voxelize_gpu.cu).
  if(BAGWIZ_WITH_SLAM_CUDA)
    if(NOT TARGET gtsam_points::gtsam_points_cuda)
      message(FATAL_ERROR
        "BAGWIZ_WITH_SLAM_CUDA=ON but gtsam_points has no CUDA target. Rebuild the GLIM deps "
        "with CUDA: scripts/build-glim-deps.sh --cuda (pixi run -e humble-cuda build-full).")
    endif()
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
      message(WARNING
        "CMAKE_CUDA_ARCHITECTURES not set; defaulting to 86 (sm_86 / RTX 30xx). For another GPU "
        "pass -DCMAKE_CUDA_ARCHITECTURES=<arch> (e.g. 75 = RTX 20xx, 89 = RTX 40xx). The "
        "voxelize_gpu kernel falls back to CPU on an arch mismatch, but GLIM's VGICP_GPU mapping "
        "does not — a wrong arch then fails the --gpu run. `pixi run -e humble-cuda build-full` sets 86.")
      set(CMAKE_CUDA_ARCHITECTURES 86)
    endif()
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_sources(bagwiz PRIVATE src/core/slam/cloud_voxelize_gpu.cu)
    set_source_files_properties(src/core/slam/cloud_voxelize_gpu.cu PROPERTIES LANGUAGE CUDA)
    target_link_libraries(bagwiz PRIVATE CUDA::cudart)
    target_compile_definitions(bagwiz PRIVATE BAGWIZ_WITH_SLAM_CUDA)
  endif()
endif()

# --- Optional browser map viewer for `bagwiz map slam --viewer` ---------------
# The viewer (a cpp-httplib loopback server + embedded Three.js assets + a
# browser launch) is entirely GLIM-free, so it gets its own option that simply
# DEFAULTS to BAGWIZ_WITH_SLAM: a SLAM build (the default `pixi run build-full` on
# humble/jazzy) gains the viewer, a core build (`build-core` or
# BAGWIZ_WITH_SLAM=OFF) pulls in neither cpp-httplib nor this code, and the viewer can be built and
# tested on its own with -DBAGWIZ_WITH_MAP_VIEWER=ON (no GLIM needed).
option(BAGWIZ_WITH_MAP_VIEWER
  "Build the `map slam --viewer` browser map viewer (fetches cpp-httplib)"
  ${BAGWIZ_WITH_SLAM})
if(BAGWIZ_WITH_MAP_VIEWER)
  FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.18.3
    GIT_SHALLOW TRUE
  )
  # Header-only (no split compile) and, crucially, no install() rules: httplib's
  # own install export otherwise collides with ament_cmake_symlink_install.
  set(HTTPLIB_COMPILE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_INSTALL OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_REQUIRE_OPENSSL OFF CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  FetchContent_MakeAvailable(httplib)
  unset(CMAKE_POLICY_VERSION_MINIMUM)

  # The viewer client is TypeScript (assets/slam/*.ts). Compile it to ES modules
  # at configure time, then embed the generated JS (HTML + JS) into the binary as
  # string_view literals via configure_file (@ONLY so the assets' own ${...}/;
  # characters are left untouched). three.js is only a devDependency for type
  # checking — it is loaded from a CDN at runtime — so its types are installed
  # first. npm/tsc come from the pixi `nodejs` dependency. Editing a .ts/.html or
  # tsconfig retriggers configure (and thus tsc) through CONFIGURE_DEPENDS.
  set(_viewer_assets_dir "${CMAKE_CURRENT_SOURCE_DIR}/assets/slam")
  set(_viewer_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated_viewer")
  set(_viewer_js_dir "${_viewer_gen_dir}/js")

  find_program(NPM_EXECUTABLE npm REQUIRED)
  if(NOT EXISTS "${_viewer_assets_dir}/node_modules")
    message(STATUS "bagwiz: installing map-viewer TypeScript toolchain (npm ci)")
    execute_process(
      COMMAND "${NPM_EXECUTABLE}" ci --no-audit --no-fund
      WORKING_DIRECTORY "${_viewer_assets_dir}"
      RESULT_VARIABLE _npm_result)
    if(NOT _npm_result EQUAL 0)
      message(FATAL_ERROR "bagwiz: npm ci failed for the map viewer (exit ${_npm_result}); "
        "remove ${_viewer_assets_dir}/node_modules and retry with network access")
    endif()
  endif()
  execute_process(
    COMMAND "${_viewer_assets_dir}/node_modules/.bin/tsc" -p tsconfig.json
            --outDir "${_viewer_js_dir}"
    WORKING_DIRECTORY "${_viewer_assets_dir}"
    RESULT_VARIABLE _tsc_result)
  if(NOT _tsc_result EQUAL 0)
    message(FATAL_ERROR "bagwiz: TypeScript compile of the map viewer failed (exit ${_tsc_result})")
  endif()

  file(READ "${_viewer_assets_dir}/map_viewer.html" VIEWER_HTML)
  file(READ "${_viewer_js_dir}/map_viewer.js" VIEWER_JS)
  file(READ "${_viewer_js_dir}/map_colormaps.js" VIEWER_COLORMAPS)
  file(READ "${_viewer_js_dir}/map_viewer_overlay.js" VIEWER_OVERLAY)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/map_viewer_assets.hpp.in"
    "${_viewer_gen_dir}/bagwiz/core/slam/map_viewer_assets.hpp"
    @ONLY)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${_viewer_assets_dir}/map_viewer.html"
    "${_viewer_assets_dir}/map_viewer.ts"
    "${_viewer_assets_dir}/map_colormaps.ts"
    "${_viewer_assets_dir}/map_viewer_overlay.ts"
    "${_viewer_assets_dir}/tsconfig.json")

  target_sources(bagwiz_core PRIVATE src/core/slam/map_viewer.cpp)
  target_include_directories(bagwiz_core PRIVATE "${_viewer_gen_dir}")
  # Header-only httplib: pull its include dir (not a link) so the uninstallable
  # FetchContent target never lands in bagwiz_core's export set, mirroring fmt.
  target_include_directories(bagwiz_core PRIVATE
    $<TARGET_PROPERTY:httplib::httplib,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(bagwiz_core PRIVATE BAGWIZ_WITH_MAP_VIEWER)
  # map_slam.cpp lives on the bagwiz executable and guards its --viewer call on this.
  target_compile_definitions(bagwiz PRIVATE BAGWIZ_WITH_MAP_VIEWER)
endif()

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

  ament_add_gtest(bagwiz_help_formatter_test
    test/commands/help_formatter_test.cpp
    src/commands/help_formatter.cpp)
  target_link_libraries(bagwiz_help_formatter_test bagwiz_core CLI11::CLI11)

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

  ament_add_gtest(bagwiz_slam_lidar_scan_test test/core/slam/lidar_scan_test.cpp)
  target_link_libraries(bagwiz_slam_lidar_scan_test bagwiz_core)

  ament_add_gtest(bagwiz_slam_point_cloud_io_test test/core/slam/point_cloud_io_test.cpp)
  target_link_libraries(bagwiz_slam_point_cloud_io_test bagwiz_core)

  ament_add_gtest(bagwiz_slam_cloud_filters_test test/core/slam/cloud_filters_test.cpp)
  target_link_libraries(bagwiz_slam_cloud_filters_test bagwiz_core)

  ament_add_gtest(bagwiz_slam_imu_sample_test test/core/slam/imu_sample_test.cpp)
  target_link_libraries(bagwiz_slam_imu_sample_test bagwiz_core)

  ament_add_gtest(bagwiz_slam_gnss_sample_test test/core/slam/gnss_sample_test.cpp)
  target_link_libraries(bagwiz_slam_gnss_sample_test bagwiz_core)

  # gnss_projector lives in bagwiz_core; GeographicLib is linked explicitly (as
  # the geo_pose_convert test does) since it is otherwise a private dependency.
  ament_add_gtest(bagwiz_slam_gnss_projector_test test/core/slam/gnss_projector_test.cpp)
  target_link_libraries(bagwiz_slam_gnss_projector_test bagwiz_core ${GeographicLib_LIBRARIES})

  ament_add_gtest(bagwiz_slam_gnss_alignment_test test/core/slam/gnss_alignment_test.cpp)
  target_link_libraries(bagwiz_slam_gnss_alignment_test bagwiz_core)

  ament_add_gtest(bagwiz_slam_map_viewer_open_test test/core/slam/map_viewer_open_test.cpp)
  target_link_libraries(bagwiz_slam_map_viewer_open_test bagwiz_core)

  # Viewer HTTP server: route + verbatim-streaming test. Gated like its source on
  # the (GLIM-free) viewer option, so it runs in a -DBAGWIZ_WITH_MAP_VIEWER=ON
  # build as well as a full SLAM build.
  if(BAGWIZ_WITH_MAP_VIEWER)
    ament_add_gtest(bagwiz_slam_map_viewer_server_test test/core/slam/map_viewer_server_test.cpp)
    target_link_libraries(bagwiz_slam_map_viewer_server_test bagwiz_core)
    target_include_directories(bagwiz_slam_map_viewer_server_test PRIVATE
      $<TARGET_PROPERTY:httplib::httplib,INTERFACE_INCLUDE_DIRECTORIES>)
  endif()

  # GLIM-linked integration test for the mapping wrapper; only when SLAM is on
  # (find_package(glim) ran in the BAGWIZ_WITH_SLAM block above).
  if(BAGWIZ_WITH_SLAM)
    # Compile the same SLAM translation units as the production binary (cloud_mapper
    # + cuda_device, the latter GPU-free without BAGWIZ_WITH_SLAM_CUDA) so this test
    # cannot silently diverge from how bagwiz links.
    ament_add_gtest(bagwiz_slam_cloud_mapper_test
      test/core/slam/cloud_mapper_test.cpp
      src/core/slam/cloud_mapper.cpp
      src/core/slam/cuda_device.cpp)
    target_link_libraries(bagwiz_slam_cloud_mapper_test
      bagwiz_core glim::glim gtsam_points::gtsam_points gtsam)

    # GPU variant: only when CUDA SLAM is on. CI never sets BAGWIZ_WITH_SLAM_CUDA,
    # so this never compiles there; on a GPU-less CUDA build the test body
    # GTEST_SKIPs at runtime (cudaGetDeviceCount == 0), so it stays green too.
    if(BAGWIZ_WITH_SLAM_CUDA)
      ament_add_gtest(bagwiz_slam_cloud_mapper_gpu_test
        test/core/slam/cloud_mapper_gpu_test.cpp
        src/core/slam/cloud_mapper.cpp
        src/core/slam/cuda_device.cpp
        src/core/slam/cloud_voxelize_gpu.cu)
      set_source_files_properties(src/core/slam/cloud_voxelize_gpu.cu PROPERTIES LANGUAGE CUDA)
      target_link_libraries(bagwiz_slam_cloud_mapper_gpu_test
        bagwiz_core glim::glim gtsam_points::gtsam_points gtsam CUDA::cudart)
      target_compile_definitions(bagwiz_slam_cloud_mapper_gpu_test PRIVATE
        BAGWIZ_WITH_SLAM BAGWIZ_WITH_SLAM_CUDA)
    endif()

    # Progress-bar helpers + no-op-when-disabled behavior. GLIM-free, but gated
    # on SLAM since that is where indicators is fetched; its include dir is
    # pulled like the viewer server test pulls httplib's.
    ament_add_gtest(bagwiz_slam_progress_bar_test
      test/core/slam/progress_bar_test.cpp
      src/core/slam/progress_bar.cpp)
    target_link_libraries(bagwiz_slam_progress_bar_test bagwiz_core)
    target_include_directories(bagwiz_slam_progress_bar_test SYSTEM PRIVATE
      $<TARGET_PROPERTY:indicators::indicators,INTERFACE_INCLUDE_DIRECTORIES>)
  endif()

  ament_add_gtest(bagwiz_decoder_test test/core/decoder/decoder_test.cpp)
  target_link_libraries(bagwiz_decoder_test bagwiz_core)

  ament_add_gtest(bagwiz_mcap_schema_decode_test test/io/mcap_schema_decode_test.cpp)
  target_link_libraries(bagwiz_mcap_schema_decode_test bagwiz_core)

  ament_add_gtest(bagwiz_terminal_input_test test/core/terminal_input_test.cpp)
  target_link_libraries(bagwiz_terminal_input_test bagwiz_core)

  ament_add_gtest(bagwiz_tui_width_test test/core/tui/width_test.cpp)
  target_link_libraries(bagwiz_tui_width_test bagwiz_core)

  ament_add_gtest(bagwiz_tui_word_wrap_test test/core/tui/word_wrap_test.cpp)
  target_link_libraries(bagwiz_tui_word_wrap_test bagwiz_core)

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

  ament_add_gtest(
    bagwiz_terminal_image_caps_test test/core/tui/image/terminal_image_caps_test.cpp)
  target_link_libraries(bagwiz_terminal_image_caps_test bagwiz_core)

  ament_add_gtest(
    bagwiz_terminal_image_renderer_test test/core/tui/image/terminal_image_renderer_test.cpp)
  target_link_libraries(bagwiz_terminal_image_renderer_test bagwiz_core)

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

  ament_add_gtest(bagwiz_stage_profiler_test test/core/pipeline/stage_profiler_test.cpp)
  target_link_libraries(bagwiz_stage_profiler_test bagwiz_core)

  ament_add_gtest(bagwiz_rewrite_backend_test test/core/pipeline/rewrite_backend_test.cpp)
  target_link_libraries(bagwiz_rewrite_backend_test bagwiz_core)

  ament_add_gtest(bagwiz_pipelined_backend_test test/core/pipeline/pipelined_backend_test.cpp)
  target_link_libraries(bagwiz_pipelined_backend_test bagwiz_core)

  ament_add_gtest(bagwiz_bag_inplace_test test/core/bag_inplace_test.cpp)
  target_link_libraries(bagwiz_bag_inplace_test bagwiz_core)

  ament_add_gtest(bagwiz_topic_match_test test/core/topic_match_test.cpp)
  target_link_libraries(bagwiz_topic_match_test bagwiz_core)

  ament_add_gtest(bagwiz_camera_info_resolver_test test/core/camera_info_resolver_test.cpp)
  target_link_libraries(bagwiz_camera_info_resolver_test bagwiz_core)

  ament_add_gtest(bagwiz_topic_drop_test
    test/commands/topic_drop_test.cpp
    src/commands/topic_drop.cpp)
  target_link_libraries(bagwiz_topic_drop_test bagwiz_core fmt::fmt)

  ament_add_gtest(bagwiz_topic_keep_test
    test/commands/topic_keep_test.cpp
    src/commands/topic_keep.cpp)
  target_link_libraries(bagwiz_topic_keep_test bagwiz_core fmt::fmt)

  ament_add_gtest(bagwiz_topic_rename_test
    test/commands/topic_rename_test.cpp
    src/commands/topic_rename.cpp)
  target_link_libraries(bagwiz_topic_rename_test bagwiz_core fmt::fmt)

  ament_add_gtest(bagwiz_raw_image_test test/core/image/raw_image_test.cpp)
  target_link_libraries(bagwiz_raw_image_test bagwiz_core)

  ament_add_gtest(bagwiz_packed_raster_test test/core/image/packed_raster_test.cpp)
  target_link_libraries(bagwiz_packed_raster_test bagwiz_core)

  ament_add_gtest(bagwiz_camera_info_test test/core/image/camera_info_test.cpp)
  target_link_libraries(bagwiz_camera_info_test bagwiz_core)

  ament_add_gtest(
    bagwiz_camera_calibration_yaml_test
    test/core/image/camera_calibration_yaml_test.cpp)
  target_link_libraries(bagwiz_camera_calibration_yaml_test bagwiz_core)

  ament_add_gtest(bagwiz_image_undistort_test test/core/image/undistort_test.cpp)
  target_link_libraries(bagwiz_image_undistort_test bagwiz_core ${OpenCV_LIBS})

  ament_add_gtest(bagwiz_compressed_image_test test/core/image/compressed_image_test.cpp)
  target_link_libraries(bagwiz_compressed_image_test bagwiz_core)

  ament_add_gtest(bagwiz_pointcloud2_test test/core/pointcloud/pointcloud2_test.cpp)
  target_link_libraries(bagwiz_pointcloud2_test bagwiz_core)

  ament_add_gtest(bagwiz_color_mapper_test test/core/pointcloud/color_mapper_test.cpp)
  target_link_libraries(bagwiz_color_mapper_test bagwiz_core)

  ament_add_gtest(bagwiz_projector_test test/core/pointcloud/projector_test.cpp)
  target_link_libraries(bagwiz_projector_test bagwiz_core)

  # image_decoder lives in bagwiz_core; libav is linked explicitly so the test
  # can both drive the decoder and synthesize JPEG/PNG fixtures with libav
  # (same reason video_encoder_test re-links it).
  ament_add_gtest(bagwiz_image_decoder_test
    test/core/image/image_decoder_test.cpp
    test/core/image/image_fixture.cpp)
  target_include_directories(bagwiz_image_decoder_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(bagwiz_image_decoder_test bagwiz_core PkgConfig::LIBAV)

  # image_encoder lives in bagwiz_core; libav is linked explicitly so the test
  # resolves the encode symbols under a static-lib build (same reason
  # image_decoder_test re-links it). The round-trip checks decode the PNG it
  # produces, so the decoder path is exercised too.
  ament_add_gtest(bagwiz_image_encoder_test test/core/image/image_encoder_test.cpp)
  target_link_libraries(bagwiz_image_encoder_test bagwiz_core PkgConfig::LIBAV)

  ament_add_gtest(bagwiz_frame_rate_test test/core/video/frame_rate_test.cpp)
  target_link_libraries(bagwiz_frame_rate_test bagwiz_core)

  # video_encoder lives in bagwiz_core; libav is linked explicitly so the test
  # still resolves the encode/mux symbols under a static-lib build (same reason
  # geo_pose_convert_test re-links GeographicLib).
  ament_add_gtest(bagwiz_video_encoder_test test/core/video/video_encoder_test.cpp)
  target_link_libraries(bagwiz_video_encoder_test bagwiz_core PkgConfig::LIBAV)

  ament_add_gtest(bagwiz_generate_video_test
    test/commands/generate_video_test.cpp
    test/core/image/image_fixture.cpp
    src/commands/generate_video.cpp)
  target_include_directories(bagwiz_generate_video_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(bagwiz_generate_video_test bagwiz_core fmt::fmt PkgConfig::LIBAV ${OpenCV_LIBS})

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
    bagwiz_geo_pose_convert_test test/core/msg_convert/geo_pose_convert_test.cpp)
  target_link_libraries(
    bagwiz_geo_pose_convert_test bagwiz_core ${GeographicLib_LIBRARIES})

  ament_add_gtest(bagwiz_convert_msg_geo_test
    test/commands/convert_msg_geo_test.cpp
    src/commands/convert_msg_geo.cpp)
  target_link_libraries(
    bagwiz_convert_msg_geo_test bagwiz_core fmt::fmt ${GeographicLib_LIBRARIES}
    ${sensor_msgs_TARGETS})

  ament_add_gtest(bagwiz_cam_info_replace_test
    test/commands/cam_info_replace_test.cpp
    src/commands/cam_info_replace.cpp)
  target_link_libraries(
    bagwiz_cam_info_replace_test bagwiz_core fmt::fmt ${sensor_msgs_TARGETS})
endif()

ament_package()
