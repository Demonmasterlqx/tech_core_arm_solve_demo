#!/usr/bin/env bash
# CPU kinematics and explicit Mesa software rendering on shared machines.
# xvfb-run owns and cleans its X server / authority directory on every exit.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
demo_binary="${ARM_DEMO_BINARY:-${RM26_DEMO_BINARY:-$project_dir/build/arm_demo}}"
if [[ ! -x "$demo_binary" ]]; then
  printf 'Build the demo first:\ncmake -S "%s" -B "%s/build" -DCMAKE_BUILD_TYPE=Release\ncmake --build "%s/build" -j4\n' "$project_dir" "$project_dir" "$project_dir" >&2
  exit 1
fi
export CUDA_VISIBLE_DEVICES=""
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export __GLX_VENDOR_LIBRARY_NAME=mesa
export LP_NUM_THREADS=4
demo_args=(--log "$project_dir/artifacts/keyboard_planning_demo/viewer.log" "$@")
if [[ -z "${DISPLAY:-}" ]]; then
  printf 'No DISPLAY: starting Xvfb. Keyboard interaction needs a visible desktop display.\n' >&2
  exec xvfb-run -a -s '-screen 0 1440x1000x24 -nolisten tcp' "$demo_binary" "${demo_args[@]}"
else
  exec "$demo_binary" "${demo_args[@]}"
fi
