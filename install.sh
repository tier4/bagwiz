#!/usr/bin/env bash
# install.sh - Install a `bagwiz` launcher onto your PATH that runs bagwiz inside
# its pixi-managed ROS 2 environment.
#
# bagwiz is built and run through pixi (see pixi.toml). The built binary is
# dynamically linked against the ROS libraries that live inside the pixi
# environment and resolves its message schemas (introspection typesupport and
# `.msg` text) through that environment at runtime, so it only works with the
# environment activated. Copying the bare binary onto PATH would break those
# lookups. Instead this installs a small launcher that delegates to
# `pixi run`, which activates the right environment first.
#
# Prerequisites:
#   - pixi (https://pixi.sh)
#   - a build for the chosen distro, e.g. `pixi run -e humble build`

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Default install directory. ~/.local/bin is on PATH for most distributions'
# default login shells and needs no root privileges.
BAGWIZ_INSTALL_DIR="${HOME}/.local/bin"

# The pixi environment (ROS distro) the launcher targets by default. Matches
# pixi.toml's default environment (humble). Overridable per invocation through
# the BAGWIZ_DISTRO environment variable.
BAGWIZ_LAUNCH_DISTRO="humble"

# Whether to replace an already-installed launcher. Off by default so a re-run
# never silently clobbers an existing one; pass --overwrite to update it.
OVERWRITE=false

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Install a 'bagwiz' launcher that runs bagwiz through its pixi environment.
Build first, e.g. 'pixi run -e humble build'.

Options:
  -d, --install-dir <D>    Directory to install the launcher into.
                           Default: ${BAGWIZ_INSTALL_DIR}.
      --distro <D>         pixi environment (ROS distro) the launcher targets by
                           default: humble, jazzy, or lyrical.
                           Default: ${BAGWIZ_LAUNCH_DISTRO}. Overridable at run
                           time via the BAGWIZ_DISTRO environment variable.
      --overwrite          Replace an existing launcher at the destination.
                           Without it, an already-installed launcher is left
                           untouched and the script exits with an error.
  -h, --help               Show this help message and exit.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
    -d | --install-dir)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[install.sh] --install-dir requires a directory path." >&2
            exit 1
        fi
        BAGWIZ_INSTALL_DIR="${1}"
        shift
        ;;
    --install-dir=*)
        BAGWIZ_INSTALL_DIR="${1#*=}"
        shift
        ;;
    -d*)
        BAGWIZ_INSTALL_DIR="${1#-d}"
        shift
        ;;
    --distro)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[install.sh] --distro requires a value (humble, jazzy, lyrical)." >&2
            exit 1
        fi
        BAGWIZ_LAUNCH_DISTRO="${1}"
        shift
        ;;
    --distro=*)
        BAGWIZ_LAUNCH_DISTRO="${1#*=}"
        shift
        ;;
    --overwrite)
        OVERWRITE=true
        shift
        ;;
    --help | -h)
        usage
        exit 0
        ;;
    *)
        echo "[install.sh] Unknown argument: ${1}" >&2
        usage >&2
        exit 1
        ;;
    esac
done

# The distro is interpolated into the generated launcher; reject anything with
# shell metacharacters so a stray quote or '$' cannot corrupt it. This also
# catches typos early without hardcoding the distro list (any built distro is
# selectable at run time via BAGWIZ_DISTRO).
if [[ ! ${BAGWIZ_LAUNCH_DISTRO} =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "[install.sh] Invalid --distro '${BAGWIZ_LAUNCH_DISTRO}': use letters, digits, '.', '_', or '-'." >&2
    exit 1
fi

# Warn (don't fail) if the targeted distro has not been built yet: the launcher
# runs the per-distro binary, so it only works once that build exists.
built_binary="${SCRIPT_DIR}/install/${BAGWIZ_LAUNCH_DISTRO}/bagwiz/bin/bagwiz"
if [[ ! -e ${built_binary} ]]; then
    echo "[install.sh] Note: no build found for '${BAGWIZ_LAUNCH_DISTRO}' at" >&2
    echo "[install.sh]       ${built_binary}" >&2
    echo "[install.sh]       Build it before running bagwiz: pixi run -e ${BAGWIZ_LAUNCH_DISTRO} build" >&2
fi

# Create the destination directory if it does not exist yet. mkdir -p is
# idempotent, so re-installing into an existing directory is a no-op.
if [[ ! -d ${BAGWIZ_INSTALL_DIR} ]]; then
    echo "[install.sh] Creating install directory ${BAGWIZ_INSTALL_DIR}"
    mkdir -p "${BAGWIZ_INSTALL_DIR}"
fi

dest="${BAGWIZ_INSTALL_DIR}/bagwiz"

# Refuse to clobber an existing install unless --overwrite was given, so a re-run
# never silently replaces a launcher the user may not want changed.
if [[ -e ${dest} && ${OVERWRITE} != true ]]; then
    echo "[install.sh] bagwiz is already installed at ${dest}." >&2
    echo "[install.sh] Pass --overwrite to replace (update) it." >&2
    exit 1
fi

echo "[install.sh] Installing bagwiz launcher to ${dest} (distro: ${BAGWIZ_LAUNCH_DISTRO})"

# The launcher delegates to scripts/bagwiz-run.sh, which runs the per-distro
# binary from a cached snapshot of the pixi environment instead of re-activating
# pixi on every call (see that script for the rationale and how the snapshot is
# kept fresh). Make sure it is present before installing a shim that points at it.
runner="${SCRIPT_DIR}/scripts/bagwiz-run.sh"
if [[ ! -x ${runner} ]]; then
    echo "[install.sh] Missing launcher runtime: ${runner}" >&2
    echo "[install.sh] Re-checkout the repository; install.sh needs scripts/bagwiz-run.sh." >&2
    exit 1
fi
# Shell-quote the runner path so a repo checkout path containing a space, '$',
# backtick, or quote still produces a launcher that execs the right file.
printf -v runner_q '%q' "${runner}"

# Write the launcher: a thin shim that records which repository and default
# distro to use and hands off to scripts/bagwiz-run.sh. BAGWIZ_DISTRO still
# overrides the distro at run time, and BAGWIZ_OVERLAY still layers extra
# message workspaces.
cat >"${dest}" <<EOF
#!/usr/bin/env bash
# bagwiz launcher (generated by bagwiz/install.sh). Runs bagwiz from its
# pixi-managed ROS 2 environment via scripts/bagwiz-run.sh.
#   - Set BAGWIZ_DISTRO to target a different built distro (humble/jazzy/
#     lyrical); it must be built first: pixi run -e <distro> build.
#   - Set BAGWIZ_OVERLAY (colon-separated workspace paths) to layer your own
#     ROS 2 message packages on top.
exec env BAGWIZ_DEFAULT_DISTRO="${BAGWIZ_LAUNCH_DISTRO}" \\
    ${runner_q} "\$@"
EOF
chmod 0755 "${dest}"

case ":${PATH}:" in
*":${BAGWIZ_INSTALL_DIR}:"*)
    echo "[install.sh] ${BAGWIZ_INSTALL_DIR} is on your PATH; run 'bagwiz' from anywhere."
    ;;
*)
    # Pick the startup file matching the current login shell so the
    # suggested command lands somewhere the user actually sources.
    case "$(basename "${SHELL:-}")" in
    zsh) shell_rc="${HOME}/.zshrc" ;;
    bash) shell_rc="${HOME}/.bashrc" ;;
    *) shell_rc="your shell startup file" ;;
    esac
    echo "[install.sh] Note: ${BAGWIZ_INSTALL_DIR} is NOT on your PATH, so 'bagwiz' will" >&2
    echo "[install.sh]       not be found until you add it. To make it permanent, append" >&2
    echo "[install.sh]       it to ${shell_rc}:" >&2
    echo "[install.sh]           echo 'export PATH=\"${BAGWIZ_INSTALL_DIR}:\$PATH\"' >> ${shell_rc}" >&2
    echo "[install.sh]       Then reload it (e.g. 'source ${shell_rc}') or open a new shell." >&2
    ;;
esac
