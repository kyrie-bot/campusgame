#!/usr/bin/env bash
# Reuse the same bounded job policy and explicit communication-package whitelist.
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$root/../../build_autoaim_safe.sh" --bridge-only "$@"
