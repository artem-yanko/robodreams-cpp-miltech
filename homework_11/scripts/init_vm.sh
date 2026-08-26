#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0"
  exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

echo "[1/6] Installing packages"
apt-get update
apt-get install -y \
  build-essential \
  cmake \
  ninja-build \
  git \
  socat \
  gpiod \
  libgpiod-dev

echo "[2/6] Mounting configfs/debugfs if needed"
mkdir -p /sys/kernel/config /sys/kernel/debug
mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config
mountpoint -q /sys/kernel/debug || mount -t debugfs none /sys/kernel/debug

echo "[3/6] Checking gpio-sim module"
if modprobe gpio-sim 2>/dev/null; then
  echo "gpio-sim: OK"
else
  echo "gpio-sim: NOT AVAILABLE"
  echo "Kernel: $(uname -r)"
  echo "If checker --sim fails, install a kernel build that includes gpio-sim."
fi

echo "[4/6] Making checker binaries executable"
find "${REPO_DIR}/homework_11/data/checker" -maxdepth 1 -type f -name 'checker_*' -exec chmod +x {} \;

echo "[5/6] Printing useful versions"
cmake --version | head -n 1
g++ --version | head -n 1
socat -V | head -n 1

echo "[6/6] Done"
