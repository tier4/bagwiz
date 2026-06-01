#!/usr/bin/env bash
# ros-source.sh - Ensure a ROS 2 environment is sourced for the calling script.
#
# This file is meant to be SOURCED, not executed. setup.sh and build.sh source
# it and call ensure_ros_sourced to guarantee a ROS 2 environment is sourced
# before they run rosdep / colcon. Because those scripts are executed (their own
# process), the `source` performed here applies to the rest of that script's run.
#
# A script cannot source ROS into the parent interactive shell, so the distro a
# user picks during ./setup.sh would be lost by the time they run ./build.sh.
# To avoid asking twice, the chosen distro is remembered in a small state file
# (ROS_DISTRO_STATE_FILE) and reused automatically on the next run.
#
# Behavior of ensure_ros_sourced <tag>:
#   - ROS already sourced (ROS_DISTRO and AMENT_PREFIX_PATH set) -> use it,
#     remember the distro, and return.
#   - No ROS install found under ROS_INSTALL_ROOT (default /opt/ros) -> tell the
#     user to install ROS 2 and exit 1 (no interactive install).
#   - A previously selected distro is remembered and still installed -> source
#     it without prompting (so ./setup.sh's choice carries over to ./build.sh).
#   - Otherwise, one or more distros installed -> list them and prompt for a
#     choice on an interactive terminal, then remember and source it. With no
#     TTY (e.g. CI, pipelines) it exits 1 with instructions instead of hanging.
#
# Override the install root with ROS_INSTALL_ROOT and the remembered-distro
# location with ROS_DISTRO_STATE_FILE for non-standard layouts.

# Root directory ROS 2 distros are installed under (one subdir per distro).
ROS_INSTALL_ROOT="${ROS_INSTALL_ROOT:-/opt/ros}"

# Where the most recently selected distro is remembered. Defaults next to this
# script so a checkout's chosen distro persists across runs (and clean builds).
_ROS_SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_STATE_FILE="${ROS_DISTRO_STATE_FILE:-${_ROS_SOURCE_DIR}/.ros_distro}"

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

# Return success if <needle> equals one of the remaining arguments.
_distro_in_list() {
    local needle="$1" item
    shift
    for item in "$@"; do
        [[ ${item} == "${needle}" ]] && return 0
    done
    return 1
}

# Remember the selected distro (best effort; a read-only checkout is not fatal).
_save_ros_distro() {
    printf '%s\n' "$1" >"${ROS_DISTRO_STATE_FILE}" 2>/dev/null || true
}

# Print the remembered distro, or return non-zero if none is recorded.
_read_saved_ros_distro() {
    [[ -f ${ROS_DISTRO_STATE_FILE} ]] || return 1
    local saved
    IFS= read -r saved <"${ROS_DISTRO_STATE_FILE}" || return 1
    [[ -n ${saved} ]] || return 1
    printf '%s' "${saved}"
}

# Source the given distro's setup.bash into the current (script) process.
_source_ros_distro() {
    local tag="$1" distro="$2"
    local setup_file="${ROS_INSTALL_ROOT}/${distro}/setup.bash"
    if [[ ! -f ${setup_file} ]]; then
        echo "[${tag}] ${setup_file} not found; aborting." >&2
        exit 1
    fi
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

# Ensure a ROS 2 environment is sourced, sourcing a remembered or user-selected
# distro if not. Usage: ensure_ros_sourced <tag>  (tag prefixes messages).
ensure_ros_sourced() {
    local tag="$1"

    if _ros_is_sourced; then
        echo "[${tag}] Using sourced ROS 2 distro: ${ROS_DISTRO}"
        _save_ros_distro "${ROS_DISTRO}"
        return 0
    fi

    local distros=()
    mapfile -t distros < <(_discover_ros_distros)

    if [[ ${#distros[@]} -eq 0 ]]; then
        echo "[${tag}] ROS 2 is not sourced and no installation was found under ${ROS_INSTALL_ROOT}." >&2
        echo "[${tag}] Install ROS 2 first: https://docs.ros.org/en/rolling/Installation.html" >&2
        exit 1
    fi

    # Reuse the distro chosen on a previous run (e.g. during ./setup.sh) so a
    # follow-up ./build.sh does not prompt again. This runs before the TTY guard
    # so it works in non-interactive contexts too.
    local saved
    if saved="$(_read_saved_ros_distro)" && _distro_in_list "${saved}" "${distros[@]}"; then
        echo "[${tag}] Reusing previously selected ROS 2 distro: ${saved}"
        echo "[${tag}] (delete ${ROS_DISTRO_STATE_FILE} to choose again)"
        _source_ros_distro "${tag}" "${saved}"
        return 0
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

    _save_ros_distro "${selected}"
    _source_ros_distro "${tag}" "${selected}"
}
