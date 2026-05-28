#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Default parallel job count: half of the CPU count from nproc(1) (logical
# processors), minimum 1. nproc is in coreutils and avoids parsing lscpu.
default_parallel_workers() {
    local cores half
    cores=$(nproc 2>/dev/null || echo 2)
    half=$((cores / 2))
    if [[ ${half} -lt 1 ]]; then
        half=1
    fi
    echo "${half}"
}

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  -c, --clean              Remove install/, build/, log/ before building (clean build).
      --build-type <T>     CMake configuration: release (default), info, or debug.
                           Maps to Release, RelWithDebInfo, or Debug respectively.
      --builder <B>        Underlying CMake generator: make (default) or ninja.
                           If ninja is requested but not installed, the script
                           exits with a message asking you to install it.
  -j, --parallel <N>       Number of parallel colcon workers (positive integer).
                           Default: half of the CPU count from nproc(1) (minimum 1).
      --native             Append -march=native to release compile flags.
                           Produces a non-portable binary tied to this host's
                           CPU in exchange for letting hot paths use the full
                           instruction set (AVX2/AVX-512/BMI2/...).
                           Ignored on debug.
      --unroll             Append -funroll-loops to release compile flags.
                           Helps tight inner loops but increases code size.
                           Ignored on debug.
      --docker             Build the bagwiz Docker image for the host
                           architecture instead of running a native colcon
                           build. Does not require a sourced ROS environment.
                           The colcon-only options above are ignored.
      --distro <D>         ROS 2 distribution to build the image for: humble or
                           jazzy (default: jazzy). Only used with --docker.
      --tag <T>            Image tag to assign (default: bagwiz:<distro>).
                           Only used with --docker.
  -h, --help               Show this help message and exit.

With no options, performs an incremental colcon build with build type release
using GNU Make as the CMake generator.
EOF
}

ensure_ninja_installed() {
    if command -v ninja >/dev/null 2>&1; then
        return 0
    fi
    echo "[build.sh] 'ninja' is not installed. Please install it and re-run, or use --builder make." >&2
    exit 1
}

clean=0
build_type="release"
builder="make"
parallel_workers=""
native=0
unroll=0
docker=0
distro="jazzy"
image_tag=""

while [[ $# -gt 0 ]]; do
    case "$1" in
    --clean | -c)
        clean=1
        shift
        ;;
    --build-type)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[build.sh] --build-type requires a value (release, info, or debug)." >&2
            exit 1
        fi
        build_type="${1}"
        shift
        ;;
    --build-type=*)
        build_type="${1#*=}"
        shift
        ;;
    --builder)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[build.sh] --builder requires a value (make or ninja)." >&2
            exit 1
        fi
        builder="${1}"
        shift
        ;;
    --builder=*)
        builder="${1#*=}"
        shift
        ;;
    -j | --parallel)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[build.sh] -j / --parallel requires a positive integer (e.g. -j 8)." >&2
            exit 1
        fi
        parallel_workers="${1}"
        shift
        ;;
    --parallel=*)
        parallel_workers="${1#*=}"
        shift
        ;;
    -j*)
        parallel_workers="${1#-j}"
        if [[ -z ${parallel_workers} ]]; then
            echo "[build.sh] -j requires a positive integer (e.g. -j 8)." >&2
            exit 1
        fi
        shift
        ;;
    --native)
        native=1
        shift
        ;;
    --unroll)
        unroll=1
        shift
        ;;
    --docker)
        docker=1
        shift
        ;;
    --distro)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[build.sh] --distro requires a value (humble or jazzy)." >&2
            exit 1
        fi
        distro="${1}"
        shift
        ;;
    --distro=*)
        distro="${1#*=}"
        shift
        ;;
    --tag)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[build.sh] --tag requires a value (e.g. bagwiz:jazzy)." >&2
            exit 1
        fi
        image_tag="${1}"
        shift
        ;;
    --tag=*)
        image_tag="${1#*=}"
        shift
        ;;
    --help | -h)
        usage
        exit 0
        ;;
    *)
        echo "[build.sh] Unknown argument: ${1}" >&2
        usage >&2
        exit 1
        ;;
    esac
done

# Docker image build mode. Builds the image defined by ./Dockerfile for the
# host architecture. This path is independent of the native colcon build and
# deliberately runs before the ROS_DISTRO check below: the build happens inside
# the container, so the host needs neither a sourced ROS environment nor any
# ROS packages installed.
if [[ ${docker} -eq 1 ]]; then
    case "${distro}" in
    humble | jazzy) ;;
    *)
        echo "[build.sh] Invalid --distro '${distro}'. Use humble or jazzy." >&2
        exit 1
        ;;
    esac

    if ! command -v docker >/dev/null 2>&1; then
        echo "[build.sh] 'docker' is not installed or not on PATH." >&2
        exit 1
    fi

    if [[ -z ${image_tag} ]]; then
        image_tag="bagwiz:${distro}"
    fi

    # The colcon-tuning options do not affect the image, which always builds in
    # release mode as defined by the Dockerfile.
    if [[ ${clean} -eq 1 || ${native} -eq 1 || ${unroll} -eq 1 ||
        ${builder} != "make" || ${build_type} != "release" || -n ${parallel_workers} ]]; then
        echo "[build.sh] --docker ignores the native colcon options" \
            "(--clean, --build-type, --builder, -j/--parallel, --native, --unroll)." >&2
    fi

    echo "[build.sh] Building Docker image '${image_tag}' for the host architecture (ROS_DISTRO=${distro})"
    exec docker build --build-arg "ROS_DISTRO=${distro}" --tag "${image_tag}" "${SCRIPT_DIR}"
fi

if [[ -z ${ROS_DISTRO:-} ]]; then
    echo "[build.sh] ROS_DISTRO is not set. Source your ROS environment first." >&2
    # shellcheck disable=SC2016  # show literal ${ROS_DISTRO} in the suggested command
    echo '[build.sh] Example: source /opt/ros/${ROS_DISTRO}/setup.bash' >&2
    exit 1
fi

case "${build_type}" in
release)
    cmake_build_type="Release"
    ;;
info)
    cmake_build_type="RelWithDebInfo"
    ;;
debug)
    cmake_build_type="Debug"
    ;;
*)
    echo "[build.sh] Invalid --build-type '${build_type}'. Use release, info, or debug." >&2
    exit 1
    ;;
esac

case "${builder}" in
make)
    cmake_generator="Unix Makefiles"
    ;;
ninja)
    cmake_generator="Ninja"
    ensure_ninja_installed
    ;;
*)
    echo "[build.sh] Invalid --builder '${builder}'. Use make or ninja." >&2
    exit 1
    ;;
esac

if [[ -z ${parallel_workers} ]]; then
    parallel_workers="$(default_parallel_workers)"
fi

if ! [[ ${parallel_workers} =~ ^[1-9][0-9]*$ ]]; then
    echo "[build.sh] Parallel worker count must be a positive integer, got: '${parallel_workers}'" >&2
    exit 1
fi

if [[ ${clean} -eq 1 ]]; then
    echo "[build.sh] Clean build: removing install/, build/, log/"
    rm -rf "${SCRIPT_DIR}/install" "${SCRIPT_DIR}/build" "${SCRIPT_DIR}/log"
fi

# Strip stale references to this workspace's install/ tree from
# colon-separated env vars. If a previous `source install/setup.bash`
# left `${SCRIPT_DIR}/install/<pkg>` in AMENT_PREFIX_PATH or
# CMAKE_PREFIX_PATH and then `-c` removed the directory, colcon emits
# a "path doesn't exist" warning on every subsequent build. Filtering
# those entries here resolves the warning at its source without
# requiring the user to open a fresh shell.
strip_workspace_install_paths() {
    local var_name="$1"
    local current="${!var_name:-}"
    if [[ -z ${current} ]]; then
        return
    fi
    local prefix="${SCRIPT_DIR}/install"
    local filtered=""
    local entry
    local IFS=:
    for entry in ${current}; do
        if [[ ${entry} == "${prefix}" || ${entry} == "${prefix}/"* ]]; then
            continue
        fi
        if [[ -z ${filtered} ]]; then
            filtered="${entry}"
        else
            filtered="${filtered}:${entry}"
        fi
    done
    if [[ ${filtered} != "${current}" ]]; then
        export "${var_name}=${filtered}"
    fi
}

strip_workspace_install_paths AMENT_PREFIX_PATH
strip_workspace_install_paths CMAKE_PREFIX_PATH
strip_workspace_install_paths COLCON_PREFIX_PATH

echo "[build.sh] CMAKE_BUILD_TYPE=${cmake_build_type}"
echo "[build.sh] CMake generator=${cmake_generator}"
echo "[build.sh] parallel workers=${parallel_workers}"

# --native and --unroll each contribute one flag to the release
# CMAKE_CXX_FLAGS variants. Debug intentionally skips them.
extra_release_flag_args=()
extra_release_flags=""
if [[ ${native} -eq 1 ]]; then
    extra_release_flags+=" -march=native"
fi
if [[ ${unroll} -eq 1 ]]; then
    extra_release_flags+=" -funroll-loops"
fi
if [[ -n ${extra_release_flags} ]]; then
    case "${cmake_build_type}" in
    Release | RelWithDebInfo)
        echo "[build.sh] appending release flags:${extra_release_flags}"
        extra_release_flag_args+=("-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG${extra_release_flags}")
        extra_release_flag_args+=("-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g -DNDEBUG${extra_release_flags}")
        ;;
    Debug)
        echo "[build.sh] --native/--unroll: ignored for Debug build"
        ;;
    esac
fi

# bagwiz keeps its package.xml at the workspace root rather than under
# src/. colcon stops recursing once it identifies a package, so
# --base-paths is limited to this directory.
colcon build \
    --symlink-install \
    --parallel-workers "${parallel_workers}" \
    --base-paths "${SCRIPT_DIR}" \
    --packages-up-to bagwiz \
    --cmake-args -G "${cmake_generator}" "-DCMAKE_BUILD_TYPE=${cmake_build_type}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "${extra_release_flag_args[@]}"
