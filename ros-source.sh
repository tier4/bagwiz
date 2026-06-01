#!/usr/bin/env bash
# ros-source.sh - Ensure a ROS 2 environment is sourced for the calling script.
#
# This file is meant to be SOURCED, not executed. setup.sh and build.sh source
# it and call ensure_ros_sourced to guarantee a ROS 2 environment is sourced
# before they run rosdep / colcon. Because those scripts are executed (their own
# process), the `source` performed here applies to the rest of that script's run.
#
# Behavior of ensure_ros_sourced <tag>:
#   - ROS already sourced (ROS_DISTRO and AMENT_PREFIX_PATH set) -> report and
#     return.
#   - No ROS install found under ROS_INSTALL_ROOT (default /opt/ros) -> tell the
#     user to install ROS 2 and exit 1 (no interactive install).
#   - One or more distros installed -> list them and prompt for a choice on an
#     interactive terminal, then source the selected distro. With no TTY (e.g.
#     CI, pipelines) it exits 1 with instructions instead of hanging.
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

# Ensure a ROS 2 environment is sourced, sourcing a user-selected distro if not.
# Usage: ensure_ros_sourced <tag>   (tag is the caller name used in messages)
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

    if [[ ! -t 0 ]]; then
        echo "[${tag}] ROS 2 is not sourced and there is no terminal to choose a distro." >&2
        echo "[${tag}] Source it first, e.g.: source ${ROS_INSTALL_ROOT}/${distros[0]}/setup.bash" >&2
        exit 1
    fi

    echo "[${tag}] ROS 2 is not sourced. Installed distros under ${ROS_INSTALL_ROOT}:"
    local i
    for i in "${!distros[@]}"; do
        printf '  [%d] %s\n' "$((i + 1))" "${distros[i]}"
    done

    local choice selected
    while true; do
        if ! read -rp "[${tag}] Select a distro to source [1-${#distros[@]}]: " choice; then
            echo "" >&2
            echo "[${tag}] No selection made; aborting." >&2
            exit 1
        fi
        if [[ ${choice} =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#distros[@]})); then
            selected="${distros[choice - 1]}"
            break
        fi
        echo "[${tag}] Invalid selection: '${choice}'. Enter a number between 1 and ${#distros[@]}." >&2
    done

    local setup_file="${ROS_INSTALL_ROOT}/${selected}/setup.bash"
    echo "[${tag}] Sourcing ${setup_file}"
    # ROS setup scripts reference unbound vars and run commands that may return
    # non-zero, both of which trip `set -eu`. Relax around the source only, then
    # restore (pipefail is left untouched).
    set +eu
    # shellcheck disable=SC1090  # path is computed from the chosen distro
    source "${setup_file}"
    set -eu

    if ! _ros_is_sourced; then
        echo "[${tag}] Sourcing ${setup_file} did not establish a ROS 2 environment; aborting." >&2
        exit 1
    fi
    echo "[${tag}] Now using ROS 2 distro: ${ROS_DISTRO}"
}
