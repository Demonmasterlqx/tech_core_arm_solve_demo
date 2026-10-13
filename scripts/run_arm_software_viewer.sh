#!/usr/bin/env bash
# CPU-only OpenGL; xvfb-run cleans its temporary X server and authority file.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
arm_binary="${ARM_CONTROL_BINARY:-$project_dir/build/arm_control}"
export CUDA_VISIBLE_DEVICES=""
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export __GLX_VENDOR_LIBRARY_NAME=mesa
export LP_NUM_THREADS=4
if [[ -z "${DISPLAY:-}" ]]; then
  exec xvfb-run -a -s '-screen 0 1440x1000x24 -nolisten tcp' "$arm_binary" "$@"
else
  exec "$arm_binary" "$@"
fi
