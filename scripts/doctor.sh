#!/usr/bin/env bash
set -uo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$REPO_ROOT/ros2_ws"
failures=0

section() {
  printf '\n== %s ==\n' "$1"
}

require_command() {
  if command -v "$1" >/dev/null 2>&1; then
    printf '[ok] %s: %s\n' "$1" "$(command -v "$1")"
  else
    printf '[error] missing command: %s\n' "$1"
    failures=$((failures + 1))
  fi
}

section "Platform"
if command -v lsb_release >/dev/null 2>&1; then
  lsb_release -ds
fi
uname -a
printf 'WSL_DISTRO_NAME=%s\n' "${WSL_DISTRO_NAME:-not-wsl}"

section "Storage"
df -h "$REPO_ROOT"

section "Network"
ip -brief address || true
ip route || true

section "USB"
if command -v lsusb >/dev/null 2>&1; then
  lsusb || true
else
  echo "[warn] lsusb is unavailable; install usbutils for USB diagnostics."
fi

section "GPU"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version,memory.total \
    --format=csv,noheader || true
else
  echo "[info] NVIDIA GPU is not exposed in this environment."
fi

section "Required commands"
for command_name in git python3 colcon; do
  require_command "$command_name"
done

section "ROS 2"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-default}"
else
  echo "[error] /opt/ros/${ROS_DISTRO}/setup.bash is missing."
  failures=$((failures + 1))
fi

require_command ros2
require_command xacro

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "$WORKSPACE/install/setup.bash"
  set -u
fi

section "Package discovery"
for package_name in zzx_interfaces mybot_description mybot mybot_navigation2; do
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    printf '[ok] %s -> %s\n' \
      "$package_name" "$(ros2 pkg prefix "$package_name")"
  else
    printf '[error] ROS package not discoverable: %s\n' "$package_name"
    failures=$((failures + 1))
  fi
done

section "Repository"
git -C "$REPO_ROOT" status --short --branch
printf 'commit=%s\n' "$(git -C "$REPO_ROOT" rev-parse HEAD)"

section "Result"
if ((failures > 0)); then
  printf 'doctor found %d required check failure(s).\n' "$failures"
  exit 1
fi
echo "doctor checks passed. Optional hardware warnings do not fail the check."
