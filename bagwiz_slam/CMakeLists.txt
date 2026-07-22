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

# --- Build options ------------------------------------------------------------
# scripts/bagwiz-build.sh passes -DBAGWIZ_WITH_SLAM=ON plus the glim-deps prefix
# on CMAKE_PREFIX_PATH to every package (colcon --cmake-args), so every
# bagwiz_* package declares these options: an undeclared cache variable makes
# CMake warn "Manually-specified variables were not used" per package. They are
# consumed only here and in bagwiz (the CLI's `map` command group). SLAM is the
# default `pixi run build-full` (humble/jazzy), which builds the GLIM stack into
# install/<distro>/glim-deps via scripts/build-glim-deps.sh. (The `build-core`
# task leaves this OFF.) The GLIM types stay confined to cloud_mapper.cpp
# (pimpl); the `map` command group and its `slam` action are GLIM-free.
option(BAGWIZ_WITH_SLAM "Build the in-process GLIM `bagwiz map slam` command" OFF)
# GPU fast path for `bagwiz map slam --gpu` (CUDA odometry + VGICP_GPU mapping +
# GPU export voxelization). OFF by default; implies BAGWIZ_WITH_SLAM. Enabled via
# `pixi run -e <distro>-cuda build-full` (humble-cuda/jazzy-cuda), which builds
# the CUDA GLIM stack into install/<distro>-cuda/glim-deps-cuda and passes
# -DBAGWIZ_WITH_SLAM_CUDA=ON plus the CUDA toolchain. A CPU build leaves this
# OFF and links no CUDA.
option(BAGWIZ_WITH_SLAM_CUDA "GPU-accelerate `bagwiz map slam --gpu` (implies BAGWIZ_WITH_SLAM)" OFF)
if(BAGWIZ_WITH_SLAM_CUDA)
  set(BAGWIZ_WITH_SLAM ON CACHE BOOL "Implied by BAGWIZ_WITH_SLAM_CUDA" FORCE)
endif()
# The viewer (a cpp-httplib loopback server + embedded Three.js assets + a
# browser launch) is entirely GLIM-free, so it gets its own option that simply
# DEFAULTS to BAGWIZ_WITH_SLAM: a SLAM build (the default `pixi run build-full`
# on humble/jazzy) gains the viewer, a core build (`build-core` or
# BAGWIZ_WITH_SLAM=OFF) pulls in neither cpp-httplib nor this code, and the
# viewer can be built and tested on its own with -DBAGWIZ_WITH_MAP_VIEWER=ON
# (no GLIM needed).
option(BAGWIZ_WITH_MAP_VIEWER
  "Build the `map slam --viewer` browser map viewer (fetches cpp-httplib)"
  ${BAGWIZ_WITH_SLAM})

include(FetchContent)

# fmt is populated before find_package(glim) deliberately: the glim -> spdlog
# -> find_dependency(fmt CONFIG) chain would otherwise create the toolchain's
# imported fmt::fmt target first, and FetchContent'ing fmt after that collides
# ("add_library cannot create ALIAS target fmt::fmt ..."). The toolchain's
# fmt-config.cmake carries the reverse guard (NOT TARGET fmt::fmt), so
# populating ours first is collision-free in both directions. Consumed
# header-only by map_viewer.cpp in the viewer block below.
if(BAGWIZ_WITH_MAP_VIEWER)
  FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 11.0.2
    GIT_SHALLOW TRUE
  )
  set(FMT_TEST OFF CACHE BOOL "" FORCE)
  set(FMT_DOC OFF CACHE BOOL "" FORCE)
  set(FMT_INSTALL OFF CACHE BOOL "" FORCE)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  FetchContent_MakeAvailable(fmt)
  unset(CMAKE_POLICY_VERSION_MINIMUM)
endif()

# The GLIM-free SLAM sources build in every configuration. The GLIM-bearing
# TUs (cloud_mapper.cpp, warmup_fill.cpp, scan_match_fill.cpp, cuda_device.cpp,
# progress_bar.cpp) form the separate bagwiz_slam_glim static library below
# (SLAM builds only); map_viewer.cpp joins THIS library when
# BAGWIZ_WITH_MAP_VIEWER is on (it is GLIM-free). Headers stay installed either
# way.
add_library(bagwiz_slam
  src/core/slam/cloud_filters.cpp
  src/core/slam/colorize_rasterizer_cpu.cpp
  src/core/slam/colorize_weight.cpp
  src/core/slam/gnss_alignment.cpp
  src/core/slam/gnss_projector.cpp
  src/core/slam/gnss_sample.cpp
  src/core/slam/imu_sample.cpp
  src/core/slam/lidar_scan.cpp
  src/core/slam/map_colorizer.cpp
  src/core/slam/map_viewer_open.cpp
  src/core/slam/point_cloud_io.cpp
  src/core/slam/propagation_radius.cpp
  src/core/slam/scan_image_pairer.cpp
  src/core/slam/scan_to_world.cpp
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

# ---------------------------------------------------------------------------
# bagwiz_slam_glim: the GLIM-bearing production objects, kept in a separate
# STATIC library so the plain bagwiz_slam archive stays GLIM-free. Installed
# and exported in the same export set (the bagwiz CLI links it from the
# install tree). Gated tests LINK this library instead of recompiling its TUs,
# so the tested objects are the shipped ones by construction. When SLAM is OFF
# none of this exists and core builds are unaffected.
# ---------------------------------------------------------------------------
if(BAGWIZ_WITH_SLAM)
  find_package(glim CONFIG REQUIRED)
  find_package(gtsam_points CONFIG REQUIRED)
  # GTSAM is pulled transitively by gtsam_points, but cloud_mapper.cpp uses
  # GTSAM directly (PoseTranslationPrior / noise models for the GNSS
  # constraint), so depend on it explicitly for its headers and the `gtsam`
  # link target.
  find_package(GTSAM CONFIG REQUIRED)

  # Header-only progress bar for `map slam` (p-ranav/indicators). indicators
  # auto-detects it is a subproject (PROJECT_NAME already set) and skips its
  # own install()/export rules, so unlike cpp-httplib below it needs no install
  # toggle and is used include-only behind the glim library.
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

  add_library(bagwiz_slam_glim STATIC
    src/core/slam/cloud_mapper.cpp
    src/core/slam/warmup_fill.cpp
    src/core/slam/scan_match_fill.cpp
    src/core/slam/cuda_device.cpp
    src/core/slam/progress_bar.cpp
  )
  # Its public headers live in the same include/ tree as bagwiz_slam's.
  target_include_directories(bagwiz_slam_glim
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  # PUBLIC: consumers of the glim library must re-resolve the GLIM stack
  # (glim::glim is a static archive too, so its own link interface matters at
  # the final link); the matching find_dependency() calls are exported below.
  target_link_libraries(bagwiz_slam_glim
    PUBLIC
      bagwiz_slam
      glim::glim
      gtsam_points::gtsam_points
      gtsam
  )
  # PUBLIC: glim_estimator.hpp (an installed header) switches its CUDA members
  # on BAGWIZ_WITH_SLAM_CUDA, so both macros are part of the library's compile
  # contract for consumers, not only of its own TUs.
  target_compile_definitions(bagwiz_slam_glim PUBLIC BAGWIZ_WITH_SLAM)
  # Header-only: pull indicators' include dir as SYSTEM (not a link) so its own
  # -Wshadow chatter stays out of our build, mirroring httplib below.
  target_include_directories(bagwiz_slam_glim SYSTEM PRIVATE
    $<TARGET_PROPERTY:indicators::indicators,INTERFACE_INCLUDE_DIRECTORIES>)

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
    target_sources(bagwiz_slam_glim PRIVATE
      src/core/slam/cloud_voxelize_gpu.cu
      src/core/slam/colorize_rasterizer_gpu.cu)
    set_source_files_properties(
      src/core/slam/cloud_voxelize_gpu.cu
      src/core/slam/colorize_rasterizer_gpu.cu
      PROPERTIES LANGUAGE CUDA)
    target_link_libraries(bagwiz_slam_glim PUBLIC CUDA::cudart)
    target_compile_definitions(bagwiz_slam_glim PUBLIC BAGWIZ_WITH_SLAM_CUDA)
    # CUDA::cudart is an imported target this static archive references from
    # its link interface, so export it like glim/gtsam_points/GTSAM below:
    # downstream consumers must be able to re-create it via find_dependency.
    ament_export_dependencies(CUDAToolkit)
  endif()

  install(TARGETS bagwiz_slam_glim
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )
  # glim/gtsam_points/GTSAM are plain (non-ament) CMake packages, but
  # ament_export_dependencies still emits plain find_dependency() calls for
  # them into this package's config, so downstream consumers (the bagwiz CLI
  # linking bagwiz_slam_glim from the install tree) re-resolve them. That
  # works because the glim-deps prefix is on CMAKE_PREFIX_PATH in full builds;
  # core builds (SLAM=OFF) never reach this block and export no such
  # references.
  ament_export_dependencies(glim gtsam_points GTSAM)
endif()

# --- Optional browser map viewer for `bagwiz map slam --viewer` ---------------
# GLIM-free (see the option above): FetchContent cpp-httplib, compile the
# TypeScript client with the package-local npm/tsc toolchain, embed the
# generated JS/HTML as string_view literals via configure_file, and add
# map_viewer.cpp to the plain bagwiz_slam library.
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

  # The viewer client is TypeScript (assets/slam/*.ts, inside this package).
  # Compile it to ES modules at configure time, then embed the generated JS
  # (HTML + JS) into the binary as string_view literals via configure_file
  # (@ONLY so the assets' own ${...}/; characters are left untouched). three.js
  # is only a devDependency for type checking — it is loaded from a CDN at
  # runtime — so its types are installed first. npm/tsc come from the pixi
  # `nodejs` dependency. Editing a .ts/.html or tsconfig retriggers configure
  # (and thus tsc) through CONFIGURE_DEPENDS.
  set(_viewer_assets_dir "${CMAKE_CURRENT_SOURCE_DIR}/assets/slam")
  set(_viewer_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated_viewer")
  set(_viewer_js_dir "${_viewer_gen_dir}/js")

  find_program(NPM_EXECUTABLE npm REQUIRED)
  if(NOT EXISTS "${_viewer_assets_dir}/node_modules")
    message(STATUS "bagwiz_slam: installing map-viewer TypeScript toolchain (npm ci)")
    execute_process(
      COMMAND "${NPM_EXECUTABLE}" ci --no-audit --no-fund
      WORKING_DIRECTORY "${_viewer_assets_dir}"
      RESULT_VARIABLE _npm_result)
    if(NOT _npm_result EQUAL 0)
      message(FATAL_ERROR "bagwiz_slam: npm ci failed for the map viewer (exit ${_npm_result}); "
        "remove ${_viewer_assets_dir}/node_modules and retry with network access")
    endif()
  endif()
  execute_process(
    COMMAND "${_viewer_assets_dir}/node_modules/.bin/tsc" -p tsconfig.json
            --outDir "${_viewer_js_dir}"
    WORKING_DIRECTORY "${_viewer_assets_dir}"
    RESULT_VARIABLE _tsc_result)
  if(NOT _tsc_result EQUAL 0)
    message(FATAL_ERROR "bagwiz_slam: TypeScript compile of the map viewer failed (exit ${_tsc_result})")
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

  # GLIM-free: the viewer server joins the plain library, NOT bagwiz_slam_glim.
  target_sources(bagwiz_slam PRIVATE src/core/slam/map_viewer.cpp)
  target_include_directories(bagwiz_slam PRIVATE "${_viewer_gen_dir}")
  # Header-only httplib: pull its include dir (not a link) so the uninstallable
  # FetchContent target never lands in bagwiz_slam's export set, mirroring fmt.
  target_include_directories(bagwiz_slam PRIVATE
    $<TARGET_PROPERTY:httplib::httplib,INTERFACE_INCLUDE_DIRECTORIES>)
  # fmt is used header-only inside map_viewer.cpp. Linking fmt::fmt would force
  # it into bagwiz_slam's export set, which fails because fmt is fetched via
  # FetchContent and not installable.
  target_compile_definitions(bagwiz_slam PRIVATE FMT_HEADER_ONLY)
  target_include_directories(bagwiz_slam PRIVATE
    $<TARGET_PROPERTY:fmt::fmt,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(bagwiz_slam PRIVATE BAGWIZ_WITH_MAP_VIEWER)
  # The bagwiz executable's map_slam.cpp guards its --viewer call on the same
  # macro; the CLI package defines it there itself.
endif()

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

  ament_add_gtest(bagwiz_slam_colorize_weight_test test/core/slam/colorize_weight_test.cpp)
  target_link_libraries(bagwiz_slam_colorize_weight_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_scan_to_world_test test/core/slam/scan_to_world_test.cpp)
  target_link_libraries(bagwiz_slam_scan_to_world_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_scan_image_pairer_test test/core/slam/scan_image_pairer_test.cpp)
  target_link_libraries(bagwiz_slam_scan_image_pairer_test bagwiz_slam)

  ament_add_gtest(bagwiz_slam_propagation_radius_test test/core/slam/propagation_radius_test.cpp)
  target_link_libraries(bagwiz_slam_propagation_radius_test bagwiz_slam)

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

  # Viewer HTTP server: route + verbatim-streaming test. Gated like its source on
  # the (GLIM-free) viewer option, so it runs in a -DBAGWIZ_WITH_MAP_VIEWER=ON
  # build as well as a full SLAM build.
  if(BAGWIZ_WITH_MAP_VIEWER)
    ament_add_gtest(bagwiz_slam_map_viewer_server_test test/core/slam/map_viewer_server_test.cpp)
    target_link_libraries(bagwiz_slam_map_viewer_server_test bagwiz_slam)
    target_include_directories(bagwiz_slam_map_viewer_server_test PRIVATE
      $<TARGET_PROPERTY:httplib::httplib,INTERFACE_INCLUDE_DIRECTORIES>)
  endif()

  # GLIM-linked integration tests; only when SLAM is on (find_package(glim) ran
  # in the BAGWIZ_WITH_SLAM block above). These LINK the production
  # bagwiz_slam_glim library instead of recompiling its TUs the way the
  # pre-split package did: the library is the single source of the objects, so
  # the tested and shipped code cannot diverge.
  if(BAGWIZ_WITH_SLAM)
    # Integration test for the mapping wrapper. The glim/gtsam targets are
    # linked explicitly (as the pre-split package did) although they also come
    # transitively through bagwiz_slam_glim's PUBLIC link interface.
    ament_add_gtest(bagwiz_slam_cloud_mapper_test test/core/slam/cloud_mapper_test.cpp)
    target_link_libraries(bagwiz_slam_cloud_mapper_test
      bagwiz_slam_glim glim::glim gtsam_points::gtsam_points gtsam)

    # gtsam_points-linked (GLIM-free) registration core for the scan-matching
    # warmup/cooldown fill.
    ament_add_gtest(bagwiz_slam_scan_match_fill_test test/core/slam/scan_match_fill_test.cpp)
    target_link_libraries(bagwiz_slam_scan_match_fill_test
      bagwiz_slam_glim gtsam_points::gtsam_points gtsam)

    # GPU variant: only when CUDA SLAM is on. CI never sets BAGWIZ_WITH_SLAM_CUDA,
    # so this never compiles there; on a GPU-less CUDA build the test body
    # GTEST_SKIPs at runtime (cudaGetDeviceCount == 0), so it stays green too.
    if(BAGWIZ_WITH_SLAM_CUDA)
      ament_add_gtest(bagwiz_slam_cloud_mapper_gpu_test test/core/slam/cloud_mapper_gpu_test.cpp)
      target_link_libraries(bagwiz_slam_cloud_mapper_gpu_test bagwiz_slam_glim CUDA::cudart)
      target_compile_definitions(bagwiz_slam_cloud_mapper_gpu_test PRIVATE
        BAGWIZ_WITH_SLAM BAGWIZ_WITH_SLAM_CUDA)

      ament_add_gtest(bagwiz_slam_colorize_rasterizer_gpu_test
        test/core/slam/colorize_rasterizer_gpu_test.cpp)
      target_link_libraries(bagwiz_slam_colorize_rasterizer_gpu_test bagwiz_slam_glim CUDA::cudart)
      target_compile_definitions(bagwiz_slam_colorize_rasterizer_gpu_test PRIVATE
        BAGWIZ_WITH_SLAM BAGWIZ_WITH_SLAM_CUDA)
    endif()

    # Progress-bar helpers + no-op-when-disabled behavior. GLIM-free, but gated
    # on SLAM since that is where indicators is fetched; its include dir is
    # pulled like the viewer server test pulls httplib's.
    ament_add_gtest(bagwiz_slam_progress_bar_test test/core/slam/progress_bar_test.cpp)
    target_link_libraries(bagwiz_slam_progress_bar_test bagwiz_slam_glim)
    target_include_directories(bagwiz_slam_progress_bar_test SYSTEM PRIVATE
      $<TARGET_PROPERTY:indicators::indicators,INTERFACE_INCLUDE_DIRECTORIES>)
  endif()
endif()

ament_package()
