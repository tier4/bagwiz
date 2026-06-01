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
  -h, --help               Show this help message and exit.

With no options, performs an incremental colcon build with build type release
using GNU Make as the CMake generator. The built binary lives at
install/bagwiz/bin/bagwiz.
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

# Require a sourced ROS 2 environment. ROS_DISTRO alone is not enough (it can be
# exported without sourcing); AMENT_PREFIX_PATH is set when a distro's
# setup.bash is sourced, so require both. If not sourced, print the exact source
# command for each ROS 2 distro installed under /opt/ros (or, when none is
# found, tell the user to install ROS 2) and exit. Done after argument parsing
# so --help works without ROS sourced.
ros_install_root="${ROS_INSTALL_ROOT:-/opt/ros}"
if [[ -n ${ROS_DISTRO:-} && -n ${AMENT_PREFIX_PATH:-} ]]; then
    echo "[build.sh] Using sourced ROS 2 distro: ${ROS_DISTRO}"
else
    installed_distros=()
    for entry in "${ros_install_root}"/*/; do
        [[ -f "${entry}setup.bash" ]] && installed_distros+=("$(basename "${entry}")")
    done
    if [[ ${#installed_distros[@]} -eq 0 ]]; then
        echo "[build.sh] ROS 2 is not sourced and no installation was found under ${ros_install_root}." >&2
        echo "[build.sh] Install ROS 2 first: https://docs.ros.org/en/rolling/Installation.html" >&2
        exit 1
    fi
    echo "[build.sh] ROS 2 is not sourced. Source an installed distro, then re-run this command:" >&2
    for distro in "${installed_distros[@]}"; do
        echo "[build.sh]     source ${ros_install_root}/${distro}/setup.bash" >&2
    done
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

echo "[build.sh] Done. The binary is at install/bagwiz/bin/bagwiz."
