#!/usr/bin/env bash
# Competition migration builds use the low-resource entry by default.
set -e
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/build_autoaim_safe.sh" "$@"
