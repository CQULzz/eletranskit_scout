#!/usr/bin/env bash
set -eo pipefail

KIT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# This is deliberately an explicit test entry point.  The normal
# start_real_car.sh path keeps using live Fixposition localization.
exec "${KIT_ROOT}/start_real_car.sh" static_pose_fallback:=true "$@"
