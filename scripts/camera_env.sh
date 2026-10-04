#!/usr/bin/env bash
# Source in each camera, viewer and recorder terminal; no system-wide changes.
_zzx_camera_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://${_zzx_camera_root}/ros2_ws/src/zzx_camera/config/cyclonedds_wsl.xml"
unset _zzx_camera_root
