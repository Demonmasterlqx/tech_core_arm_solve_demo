#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
version=3.3.7
destination="$project_dir/.deps/mujoco-$version"
if [[ -f "$destination/include/mujoco/mujoco.h" && -f "$destination/lib/libmujoco.so.$version" ]]; then
  echo "MuJoCo $version ready: $destination"
  exit 0
fi
if [[ "$(uname -s)/$(uname -m)" != Linux/x86_64 ]]; then
  echo 'Automatic bootstrap supports Linux x86_64; use -DMUJOCO_ROOT=/path/to/mujoco for other platforms.' >&2
  exit 1
fi
mkdir -p "$project_dir/.deps"
download_dir="$(mktemp -d "$project_dir/.deps/download.XXXXXX")"
trap 'rm -rf -- "$download_dir"' EXIT
archive="$download_dir/mujoco.tar.gz"
curl --fail --location --retry 3 --connect-timeout 20 --max-time 240 \
  "https://github.com/google-deepmind/mujoco/releases/download/$version/mujoco-$version-linux-x86_64.tar.gz" -o "$archive"
echo "1895a2f71e86d9a6026703ebdc1fae32af07792805ac56f9a73abceca30dbe1c  $archive" | sha256sum --check --status
tar -xzf "$archive" -C "$download_dir"
mv "$download_dir/mujoco-$version" "$destination"
echo "MuJoCo $version installed and SHA-256 verified: $destination"
