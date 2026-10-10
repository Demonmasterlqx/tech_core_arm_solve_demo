#!/usr/bin/env bash
# Mesa/llvmpipe explicitly avoids creating a context on the shared GPUs.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export CUDA_VISIBLE_DEVICES=""
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export __GLX_VENDOR_LIBRARY_NAME=mesa
export LP_NUM_THREADS=4
if [[ -z "${DISPLAY:-}" ]]; then
  # xvfb-run owns its temporary auth files and terminates Xvfb on exit.
  exec xvfb-run -a -s '-screen 0 1440x1000x24 -nolisten tcp' "$project_dir/build/tech_core" "$@"
else
  exec "$project_dir/build/tech_core" "$@"
fi
