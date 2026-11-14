#!/usr/bin/env bash
# Sets up a working Python environment for this project on Apple Silicon macOS.
#
# The original requirements.txt was frozen on a 2021-era Linux/x86 box (torch
# 1.9.0a0 source build, tensorflow_macos 0.1a3, Python 3.7/3.8) and cannot be
# installed as-is on modern macOS/arm64. This script builds a fresh venv with
# current, arm64-native dependency versions instead, and applies two small
# source patches needed to build `motion_imitation` and `pybullet` from
# source on a modern toolchain (see patches/*.patch for what and why).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PYTHON_BIN="${PYTHON_BIN:-/opt/homebrew/bin/python3.11}"
if [ ! -x "$PYTHON_BIN" ]; then
  echo "Python 3.11 not found at $PYTHON_BIN -- install it first: brew install python@3.11" >&2
  exit 1
fi

if [ ! -d .venv ]; then
  "$PYTHON_BIN" -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate

# gym==0.21.0 ships metadata with a version specifier pip>=24.1 refuses to
# parse ("opencv-python (>=3.)"); pin pip to the last version that tolerates it.
pip install "pip<24.1"

pip install -r requirements.txt

# pkg_resources (needed transitively by pybullet_envs) was removed from
# setuptools in recent releases; pin to the last version that still ships it.
pip install "setuptools<81"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

echo "== motion_imitation (patched: no MPC C-extension build, Python 3.10+ collections.abc fix) =="
pip download --no-binary :all: --no-deps -d "$WORKDIR" motion_imitation==0.0.6
tar xzf "$WORKDIR"/motion_imitation-0.0.6.tar.gz -C "$WORKDIR"
patch -p2 -d "$WORKDIR/motion_imitation-0.0.6" < patches/motion_imitation-0.0.6-macos-py311.patch
pip install --no-deps "$WORKDIR/motion_imitation-0.0.6"

echo "== pybullet (patched: macOS zlib OS_CODE/fdopen macro collision on modern SDKs) =="
pip download --no-binary :all: --no-deps -d "$WORKDIR" pybullet
PB_TARBALL="$(ls "$WORKDIR"/pybullet-*.tar.gz)"
tar xzf "$PB_TARBALL" -C "$WORKDIR"
PB_DIR="$(basename "$PB_TARBALL" .tar.gz)"
patch -p1 -d "$WORKDIR/$PB_DIR" < patches/pybullet-3.2.7-macos-zutil.patch
pip install "$WORKDIR/$PB_DIR"

pip install -e .

echo "== Smoke test: build OpenDynamicImitationEnv-v0 and step it =="
python - <<'EOF'
from quadruped_envs.od_imitation_env import build_imitation_env
from quadruped_envs import od
import os

motion_file = os.path.join("quadruped_envs", "data", "motions", "od", "pace.txt")
env = build_imitation_env(motion_files=[motion_file], num_parallel_envs=1,
                           enable_rendering=False, enable_randomizer=False, robot_class=od.OD)
env.reset()
for _ in range(20):
    env.step(env.action_space.sample())
env.close()
print("OK: environment builds and steps.")
EOF

echo "Setup complete."
