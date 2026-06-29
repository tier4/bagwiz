#!/usr/bin/env bash
# bagwiz-run.sh - fast launcher for the installed `bagwiz` binary.
#
# bagwiz's binary is built inside a pixi-managed ROS 2 environment and resolves
# its ROS libraries and message schemas through that environment at run time, so
# it cannot run with a bare PATH. The obvious launcher delegates every call to
# `pixi run`, but that re-activates the whole conda/RoboStack/colcon environment
# on each invocation (~0.5 s) and prints pixi's "activating environment" spinner.
# For an interactive CLI - and especially for shell completion, which spawns
# `bagwiz __complete` on every TAB - that per-call activation dominates latency.
#
# Activation is, however, deterministic: for a given build and lockfile it always
# produces the same handful of environment variables. So instead of re-activating
# every time, this launcher activates ONCE via pixi, snapshots the result to a
# small sourceable cache, and on every later call just sources that cache (a few
# `export`s) and execs the binary directly - no pixi, no spinner, ~20x faster.
# The snapshot is rebuilt automatically when it goes stale (any input that
# changes what activation produces is newer than the cache), so a rebuild,
# `pixi update`, or an edit to the activation config is picked up transparently.
#
# The binary itself needs very little from activation: its ELF RPATH already
# points at the environment's lib directory, and it spawns no subprocesses, so
# only the variables the linked ROS libraries consume at run time are cached:
# AMENT_PREFIX_PATH and LD_LIBRARY_PATH, plus the rmw selection and the ROS_*
# scalars. Each variable is reproduced exactly as `pixi run` would set it: pixi
# REBUILDS AMENT_PREFIX_PATH from the conda/colcon chain and discards the
# caller's value, but PREPENDS to LD_LIBRARY_PATH and keeps the caller's tail.
# The cache encodes that per-variable difference (detected through a sentinel
# seeded into each variable during capture) and `printf %q`-quotes every value so
# it round-trips through `source` unchanged, even if the checkout path contains
# shell metacharacters.
set -eu

# Repository root: this script lives at <repo>/scripts/bagwiz-run.sh. Resolve
# symlinks so a symlinked launcher still finds the repo.
_self="$(readlink -f -- "${BASH_SOURCE[0]}" 2>/dev/null || printf '%s' "${BASH_SOURCE[0]}")"
_self_dir="$(cd -- "$(dirname -- "${_self}")" && pwd)"
BAGWIZ_REPO="$(dirname -- "${_self_dir}")"
manifest="${BAGWIZ_REPO}/pixi.toml"
activate_hook="${_self_dir}/pixi-activate.sh"

# Which built distro to run. An explicit BAGWIZ_DISTRO wins; otherwise the
# default the launcher was installed with (BAGWIZ_DEFAULT_DISTRO), else Humble.
distro="${BAGWIZ_DISTRO:-${BAGWIZ_DEFAULT_DISTRO:-humble}}"
# The distro indexes both the binary path and the cache filename; reject empty,
# '.'/'..', and any path- or shell-unsafe character (including '/') so a stray
# BAGWIZ_DISTRO cannot traverse out of install/ or produce an odd cache name.
case "${distro}" in
'' | . | .. | *[!A-Za-z0-9._-]*)
    echo "bagwiz: invalid distro '${distro}': use letters, digits, '.', '_', or '-'." >&2
    exit 2
    ;;
esac
bin="${BAGWIZ_REPO}/install/${distro}/bagwiz/bin/bagwiz"

if [ ! -x "${bin}" ]; then
    echo "bagwiz: no build found for '${distro}' at ${bin}" >&2
    case "${distro}" in
    *-gpu) echo "bagwiz: build it first: pixi run -e ${distro} build-full-cuda" >&2 ;;
    *) echo "bagwiz: build it first: pixi run -e ${distro} build-full" >&2 ;;
    esac
    exit 127
fi

# Cache the snapshot per (repository, distro) so multiple checkouts or distros
# never share one file. cksum keys the filename on the absolute repo path.
cache_dir="${XDG_CACHE_HOME:-${HOME:-/tmp}/.cache}/bagwiz"
cache_key="$(printf '%s\n' "${BAGWIZ_REPO}" | cksum | cut -d' ' -f1)"
cache="${cache_dir}/env-${distro}-${cache_key}.sh"
lock="${BAGWIZ_REPO}/pixi.lock"
# colcon regenerates this prefix-chain setup on every build; activation sources
# it (see scripts/pixi-activate.sh), so its contents feed the captured AMENT/LD.
chain_setup="${BAGWIZ_REPO}/install/${distro}/setup.sh"

# The snapshot is stale when it is missing or older than any input that changes
# what activation produces or how it is captured: the lockfile (dependency set),
# the manifest (e.g. [activation.env]), the activation hook, the colcon prefix
# chain, the built binary, or this launcher itself (its capture/encoding logic,
# so a pulled update that changes the cache format regenerates rather than
# silently reusing an old one). It cannot detect rebuilds of EXTERNAL overlay
# workspaces chained into that setup; delete the cache after rebuilding those.
snapshot_is_stale() {
    [ ! -f "${cache}" ] && return 0
    [ -e "${lock}" ] && [ "${lock}" -nt "${cache}" ] && return 0
    [ -e "${manifest}" ] && [ "${manifest}" -nt "${cache}" ] && return 0
    [ -e "${activate_hook}" ] && [ "${activate_hook}" -nt "${cache}" ] && return 0
    [ -e "${chain_setup}" ] && [ "${chain_setup}" -nt "${cache}" ] && return 0
    [ -e "${_self}" ] && [ "${_self}" -nt "${cache}" ] && return 0
    [ "${bin}" -nt "${cache}" ] && return 0
    return 1
}

# Quote a value so it survives being written into the cache and sourced back
# unchanged - no $, backtick, quote, or whitespace is re-interpreted.
shquote() { printf '%q' "$1"; }

# Emit one path variable into the cache, reproducing pixi's per-variable
# behavior, detected via the sentinel seeded into the variable during capture:
#   - value ends in ":<sentinel>" -> pixi PREPENDS and keeps the caller's live
#     value: layer the captured delta in front of whatever the caller has.
#   - sentinel absent             -> pixi REBUILDS the variable and discards the
#     caller's value: set it verbatim.
#   - value IS the bare sentinel  -> pixi added nothing: leave it to the caller.
emit_path_var() {
    local name="$1" value="$2" sentinel="$3"
    case "${value}" in
    "" | "${sentinel}") ;;
    *":${sentinel}")
        # The ${name:+...} suffix is written literally for the cache to expand
        # when sourced (it layers onto the caller's live value), not here.
        # shellcheck disable=SC2016
        printf 'export %s=%s${%s:+:${%s}}\n' \
            "${name}" "$(shquote "${value%":${sentinel}"}")" "${name}" "${name}"
        ;;
    *)
        printf 'export %s=%s\n' "${name}" "$(shquote "${value}")"
        ;;
    esac
}

# (Re)generate the snapshot via pixi. This is the only place pixi runs; on the
# common path it is skipped entirely. The caller holds the regen lock.
regenerate_snapshot() {
    # pixi is needed only to (re)build the snapshot, never on the hot path.
    if ! command -v pixi >/dev/null 2>&1; then
        if [ -x "${HOME:-}/.pixi/bin/pixi" ]; then
            PATH="${HOME:-}/.pixi/bin:${PATH}"
        else
            echo "bagwiz: pixi not found on PATH; cannot prepare the '${distro}' environment." >&2
            echo "bagwiz: install it from https://pixi.sh" >&2
            exit 127
        fi
    fi

    echo "bagwiz: caching the '${distro}' environment (one-time; rebuilt after a rebuild)..." >&2

    # Seed the two path variables with sentinels and clear BAGWIZ_OVERLAY, then
    # dump the fully activated environment. Comparing each variable against its
    # sentinel reveals whether pixi prepended to or rebuilt it, independent of
    # the caller's environment. stdout is a pipe (not a TTY), so pixi prints no
    # activation spinner during capture.
    local sentinel_ament="@@BAGWIZ_AMENT_SENTINEL@@"
    local sentinel_ld="@@BAGWIZ_LD_SENTINEL@@"
    local raw
    if ! raw="$(
        env AMENT_PREFIX_PATH="${sentinel_ament}" LD_LIBRARY_PATH="${sentinel_ld}" BAGWIZ_OVERLAY= \
            pixi run --manifest-path "${manifest}" -e "${distro}" -- env 2>/dev/null
    )"; then
        echo "bagwiz: failed to activate the '${distro}' environment via pixi." >&2
        echo "bagwiz: try 'pixi run -e ${distro} build' and rerun." >&2
        exit 1
    fi

    local ament ld
    ament="$(printf '%s\n' "${raw}" | sed -n 's/^AMENT_PREFIX_PATH=//p')"
    ld="$(printf '%s\n' "${raw}" | sed -n 's/^LD_LIBRARY_PATH=//p')"

    # Refuse to cache a broken activation: a real environment always contributes
    # an AMENT_PREFIX_PATH prefix. Caching an empty one would stick (the file
    # exists, so it looks fresh) and silently break message decoding.
    if [ -z "${ament}" ] || [ "${ament}" = "${sentinel_ament}" ]; then
        echo "bagwiz: activation produced no AMENT_PREFIX_PATH; refusing to cache a broken environment." >&2
        exit 1
    fi

    local tmp
    if ! tmp="$(mktemp "${cache_dir}/env.XXXXXX")"; then
        echo "bagwiz: cannot create a temp file in ${cache_dir}; check that it is writable." >&2
        exit 1
    fi
    # Write the snapshot. Wrapping the block in `if !` neutralizes `set -e`
    # inside it (so a falsy `[ -n ]` test cannot abort mid-write) and lets us
    # remove the temp file on any failure instead of leaking it.
    if ! {
        echo "# bagwiz activation snapshot for '${distro}'. Auto-generated by"
        echo "# scripts/bagwiz-run.sh; rebuilt automatically when pixi.lock, pixi.toml,"
        echo "# scripts/pixi-activate.sh, or the built binary changes. Safe to delete."
        emit_path_var AMENT_PREFIX_PATH "${ament}" "${sentinel_ament}"
        emit_path_var LD_LIBRARY_PATH "${ld}" "${sentinel_ld}"
        local var val
        for var in RMW_IMPLEMENTATION ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ETC_DIR; do
            val="$(printf '%s\n' "${raw}" | sed -n "s/^${var}=//p")"
            if [ -n "${val}" ]; then
                printf 'export %s=%s\n' "${var}" "$(shquote "${val}")"
            fi
        done
    } >"${tmp}"; then
        rm -f "${tmp}"
        echo "bagwiz: failed to write the environment cache." >&2
        exit 1
    fi
    if ! mv -f "${tmp}" "${cache}"; then
        rm -f "${tmp}"
        echo "bagwiz: failed to install the environment cache at ${cache}." >&2
        exit 1
    fi
}

if snapshot_is_stale; then
    if ! mkdir -p "${cache_dir}"; then
        echo "bagwiz: cannot create cache directory ${cache_dir} (override with XDG_CACHE_HOME)." >&2
        exit 1
    fi
    # Serialize concurrent regenerations (e.g. a burst of `bagwiz __complete` on
    # the first TAB) so only one process pays the activation cost; the others
    # wait, then re-check and reuse the fresh cache. flock is optional - without
    # it the worst case is a few redundant one-time activations.
    if command -v flock >/dev/null 2>&1; then
        exec 9>"${cache_dir}/.regen.lock"
        flock 9
        snapshot_is_stale && regenerate_snapshot
        exec 9>&-
    else
        regenerate_snapshot
    fi
fi

# From here on we source environment files (the cache, then any overlays) that
# may reference unset variables and are not written to be `set -u`/`-e` clean,
# so relax both before sourcing.
set +eu

# shellcheck source=/dev/null
. "${cache}"

# External overlays (BAGWIZ_OVERLAY): layered live on top of the cached base so
# custom message workspaces are picked up without rebuilding the snapshot. This
# mirrors the overlay handling in scripts/pixi-activate.sh.
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
fi

# When the SLAM build linked the vendored GLIM stack
# (install/<distro>/glim-deps, produced by `pixi run build-full`), put its lib
# dir on LD_LIBRARY_PATH so `bagwiz map slam` resolves libglim / libgtsam* at run
# time. No-op for core builds (e.g. lyrical): the directory simply does not exist.
_bagwiz_glim_lib="${BAGWIZ_REPO}/install/${distro}/glim-deps/lib"
if [ -d "${_bagwiz_glim_lib}" ]; then
    export LD_LIBRARY_PATH="${_bagwiz_glim_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
# A GPU build (a *-gpu environment) additionally has the CUDA stack at
# install/<env>/glim-deps-cuda (CUDA libglim + libgtsam_points_cuda); place it
# AHEAD of glim-deps so its CUDA libglim wins over the CPU one. Auto-detected by
# the directory's presence — only a *-gpu env's install dir has it, so a CPU
# launcher never loads the CUDA libglim (which would pull a libcudart it never
# linked). The conda CUDA runtime (libcudart/libcusolver) comes from the env
# activation; no system /usr/local/cuda is needed.
_bagwiz_glim_cuda_lib="${BAGWIZ_REPO}/install/${distro}/glim-deps-cuda/lib"
if [ -d "${_bagwiz_glim_cuda_lib}" ]; then
    export LD_LIBRARY_PATH="${_bagwiz_glim_cuda_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

exec "${bin}" "$@"
