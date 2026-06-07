#!/bin/bash
# install.sh - Copy the built bagwiz binary onto your PATH.
#
# Run after ./build.sh has produced install/bagwiz/bin/bagwiz. colcon's
# --symlink-install leaves that path as a symlink into build/, so this copies
# the dereferenced binary: the installed copy keeps working after a clean
# rebuild removes build/.
#
# Requires: a successful ./build.sh first.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Default install directory. ~/.local/bin is on PATH for most distributions'
# default login shells and needs no root privileges.
BAGWIZ_INSTALL_DIR="${HOME}/.local/bin"

# Whether to replace an already-installed bagwiz. Off by default so a re-run
# never silently clobbers an existing binary; pass --overwrite to update it.
OVERWRITE=false

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Copy the built bagwiz binary onto your PATH. Run ./build.sh first.

Options:
  -d, --install-dir <D>    Directory to install the binary into.
                           Default: ${BAGWIZ_INSTALL_DIR}.
      --overwrite          Replace an existing bagwiz at the destination.
                           Without it, an already-installed binary is left
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

src="${SCRIPT_DIR}/install/bagwiz/bin/bagwiz"
if [[ ! -e ${src} ]]; then
    echo "[install.sh] Built binary not found at ${src}" >&2
    echo "[install.sh] Run ./build.sh first." >&2
    exit 1
fi

# Create the destination directory if it does not exist yet. mkdir -p is
# idempotent, so re-installing into an existing directory is a no-op.
if [[ ! -d ${BAGWIZ_INSTALL_DIR} ]]; then
    echo "[install.sh] Creating install directory ${BAGWIZ_INSTALL_DIR}"
    mkdir -p "${BAGWIZ_INSTALL_DIR}"
fi

dest="${BAGWIZ_INSTALL_DIR}/bagwiz"

# Refuse to clobber an existing install unless --overwrite was given, so a
# re-run never silently replaces a binary the user may not want changed.
if [[ -e ${dest} && ${OVERWRITE} != true ]]; then
    echo "[install.sh] bagwiz is already installed at ${dest}." >&2
    echo "[install.sh] Pass --overwrite to replace (update) it." >&2
    exit 1
fi

echo "[install.sh] Installing bagwiz to ${dest}"
# install(1) dereferences the symlink and sets the executable mode in one
# step. -D also creates any leading directories as a safety net.
install -D -m 0755 "${src}" "${dest}"

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
