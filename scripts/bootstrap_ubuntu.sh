#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE="${WORKSPACE:-$HOME/vision_guided_manipulation/ros2_ws}"
INSTALL_VISION_STACK="${INSTALL_VISION_STACK:-0}"

if [[ "$(. /etc/os-release && printf '%s' "$VERSION_ID")" != "22.04" ]]; then
  echo "This baseline supports Ubuntu 22.04 only." >&2
  exit 2
fi

if [[ "$ROS_DISTRO" != "humble" ]]; then
  echo "This baseline supports ROS_DISTRO=humble only." >&2
  exit 2
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  curl \
  git \
  python3-colcon-common-extensions \
  python3-jsonschema \
  python3-opencv \
  python3-pip \
  python3-pyaudio \
  python3-rosdep \
  python3-scipy \
  python3-yaml \
  python3-venv \
  python3-vcstool \
  "ros-${ROS_DISTRO}-cartographer-ros" \
  "ros-${ROS_DISTRO}-control-msgs" \
  "ros-${ROS_DISTRO}-gazebo-ros-pkgs" \
  "ros-${ROS_DISTRO}-gazebo-ros2-control" \
  "ros-${ROS_DISTRO}-joint-trajectory-controller" \
  "ros-${ROS_DISTRO}-moveit" \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-ros2-control" \
  "ros-${ROS_DISTRO}-ros2-controllers" \
  "ros-${ROS_DISTRO}-xacro"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep install --from-paths "$WORKSPACE/src" --ignore-src -r -y

if [[ "$INSTALL_VISION_STACK" == "1" ]]; then
  python3 -m venv --system-site-packages "$WORKSPACE/../.venv"
  "$WORKSPACE/../.venv/bin/python" -m pip install --upgrade pip
  "$WORKSPACE/../.venv/bin/python" -m pip install \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    -r "$WORKSPACE/../requirements/vision-cu128.lock.txt"
fi

echo "Bootstrap complete. Run scripts/doctor.sh before building."
