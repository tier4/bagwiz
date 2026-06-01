#!/usr/bin/env bash
# ros-source.sh - Verify a ROS 2 environment is sourced for the calling script.
#
# This file is meant to be SOURCED, not executed. setup.sh and build.sh source
# it and call ensure_ros_sourced, which checks that a ROS 2 environment is
# active and otherwise exits with concrete instructions. It deliberately does
# NOT source ROS itself: an executed script's `source` only affects that
# script's own process, not the parent shell, so the user must source ROS in
# their own shell for it to persist.
#
# Behavior of ensure_ros_sourced <tag>:
#   - ROS already sourced (ROS_DISTRO and AMENT_PREFIX_PATH set) -> report and
#     return 0.
#   - Not sourced, with one or more distros installed under ROS_INSTALL_ROOT
#     (default /opt/ros) -> print the exact `source .../setup.bash` command for
#     each installed distro and exit 1.
#   - Not sourced and no ROS install found -> tell the user to install ROS 2 and
#     exit 1.
#
# Override the install root with ROS_INSTALL_ROOT for non-standard layouts.

# Root directory ROS 2 distros are installed under (one subdir per distro).
ROS_INSTALL_ROOT="${ROS_INSTALL_ROOT:-/opt/ros}"

# Print the names of installed ROS 2 distros (one per line). A distro counts as
# installed when its setup.bash exists. Prints nothing when none are found.
_discover_ros_distros() {
    local entry
    for entry in "${ROS_INSTALL_ROOT}"/*/; do
        [[ -f "${entry}setup.bash" ]] || continue
        basename "${entry}"
    done
}

# Decide whether a ROS 2 environment is actually sourced. ROS_DISTRO alone is
# not enough: it can be exported by a shell rc or a container's ENV without the
# environment ever being sourced. AMENT_PREFIX_PATH is populated when a distro's
# setup.bash is sourced and is absent when only ROS_DISTRO is set, so require
# both before treating the environment as sourced.
_ros_is_sourced() {
    [[ -n ${ROS_DISTRO:-} && -n ${AMENT_PREFIX_PATH:-} ]]
}

# Verify a ROS 2 environment is sourced; otherwise exit with instructions.
# Usage: ensure_ros_sourced <tag>   (tag prefixes messages).
ensure_ros_sourced() {
    local tag="$1"

    if _ros_is_sourced; then
        echo "[${tag}] Using sourced ROS 2 distro: ${ROS_DISTRO}"
        return 0
    fi

    local distros=()
    mapfile -t distros < <(_discover_ros_distros)

    if [[ ${#distros[@]} -eq 0 ]]; then
        echo "[${tag}] ROS 2 is not sourced and no installation was found under ${ROS_INSTALL_ROOT}." >&2
        echo "[${tag}] Install ROS 2 first: https://docs.ros.org/en/rolling/Installation.html" >&2
        exit 1
    fi

    echo "[${tag}] ROS 2 is not sourced. Source an installed distro, then re-run this command:" >&2
    local distro
    for distro in "${distros[@]}"; do
        echo "[${tag}]     source ${ROS_INSTALL_ROOT}/${distro}/setup.bash" >&2
    done
    exit 1
}
