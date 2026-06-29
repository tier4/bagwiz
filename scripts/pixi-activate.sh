# shellcheck shell=sh
# Sourced by pixi on environment activation (see [target.unix.activation] in
# pixi.toml), AFTER RoboStack's own conda activation has pointed ROS_DISTRO,
# AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH and LD_LIBRARY_PATH inside the pixi
# environment ($CONDA_PREFIX). pixi keeps the environment variables this script
# exports.
#
# It does two optional, guarded things (both no-ops before the first build and
# when no overlay is configured):
#   1. Source bagwiz's own colcon overlay (install/setup.sh) once built, so the
#      freshly built `bagwiz` binary lands on PATH inside the environment.
#   2. Source one or more EXTERNAL ROS 2 workspace overlays listed (colon-
#      separated) in BAGWIZ_OVERLAY, layering the user's custom message packages
#      on top. bagwiz then finds their msg/*.msg via AMENT_PREFIX_PATH and
#      dlopen()s their introspection typesupport via LD_LIBRARY_PATH at runtime,
#      with no rebuild of bagwiz required.
#
# Build external overlays inside (or compatibly with) the same pixi/RoboStack
# distro so their shared-library ABI (libstdc++/glibc) matches bagwiz's.
#
# Written in POSIX shell (sourced setup.sh, not setup.bash) so it works whether
# pixi runs activation scripts under sh or bash.

# Workspace root: pixi exposes it under different names across versions.
_bagwiz_ws="${PIXI_PROJECT_ROOT:-${PIXI_WORKSPACE_ROOT:-$PWD}}"
# Each distro installs into its own base (install/<distro>); see [tasks.build-full].
_bagwiz_env="${PIXI_ENVIRONMENT_NAME:-default}"

# 1. bagwiz's own overlay for this distro (exists only after `pixi run build-full`).
if [ -f "${_bagwiz_ws}/install/${_bagwiz_env}/setup.sh" ]; then
    # shellcheck disable=SC1090,SC1091
    . "${_bagwiz_ws}/install/${_bagwiz_env}/setup.sh"
fi

# 2. External overlays from BAGWIZ_OVERLAY (colon-separated, sourced in order).
if [ -n "${BAGWIZ_OVERLAY:-}" ]; then
    _bagwiz_old_ifs="${IFS}"
    IFS=':'
    for _bagwiz_ov in ${BAGWIZ_OVERLAY}; do
        if [ -f "${_bagwiz_ov}/install/setup.sh" ]; then
            # shellcheck disable=SC1090,SC1091
            . "${_bagwiz_ov}/install/setup.sh"
        elif [ -f "${_bagwiz_ov}/install/local_setup.sh" ]; then
            # shellcheck disable=SC1090,SC1091
            . "${_bagwiz_ov}/install/local_setup.sh"
        fi
    done
    IFS="${_bagwiz_old_ifs}"
    unset _bagwiz_ov _bagwiz_old_ifs
fi

unset _bagwiz_ws _bagwiz_env
