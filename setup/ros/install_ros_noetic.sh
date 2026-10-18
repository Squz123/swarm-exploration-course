#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
source /etc/os-release
if [ "${ID:-}" != ubuntu ] || [ "${VERSION_CODENAME:-}" != focal ]; then
  echo 'This installer supports Ubuntu 20.04 (focal) and ROS Noetic only.' >&2; exit 2
fi
SUDO=()
if [ "$(id -u)" != 0 ]; then SUDO=(sudo); fi
export DEBIAN_FRONTEND=noninteractive
exec > >(tee -a "$ROOT/setup/ros/logs/install-ros.log") 2>&1
"${SUDO[@]}" apt-get -o Acquire::ForceIPv4=true update
"${SUDO[@]}" apt-get -o Acquire::ForceIPv4=true install -y --no-install-recommends \
  ca-certificates curl gnupg build-essential cmake pkg-config git \
  python3-numpy python3-scipy python3-yaml python3-empy libeigen3-dev libarmadillo-dev
if [ ! -f /opt/ros/noetic/setup.bash ]; then
  # Public ROS signing key, not a user credential. Keep signature checks enabled.
  gpg --show-keys --with-colons "$ROOT/setup/ros/ros.key" | grep -q 'C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'
  key_temp=$(mktemp)
  trap 'rm -f -- "$key_temp"' EXIT
  if head -c20 "$ROOT/setup/ros/ros.key" | grep -q -- '-----BEGIN'; then
    gpg --batch --yes --dearmor --output "$key_temp" "$ROOT/setup/ros/ros.key"
  else
    cp "$ROOT/setup/ros/ros.key" "$key_temp"
  fi
  "${SUDO[@]}" install -m644 "$key_temp" /usr/share/keyrings/swarm-course-ros.gpg
  printf 'deb [arch=%s signed-by=/usr/share/keyrings/swarm-course-ros.gpg] http://packages.ros.org/ros/ubuntu focal main\n' "$(dpkg --print-architecture)" | \
    "${SUDO[@]}" tee /etc/apt/sources.list.d/swarm-course-ros.list >/dev/null
  "${SUDO[@]}" apt-get -o Acquire::ForceIPv4=true update
fi
"${SUDO[@]}" apt-get -o Acquire::ForceIPv4=true install -y --no-install-recommends \
  python3-catkin-pkg python3-catkin-pkg-modules python3-rospkg python3-rospkg-modules \
  ros-noetic-ros-base ros-noetic-rviz ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-cmake-modules ros-noetic-tf2-geometry-msgs ros-noetic-interactive-markers ros-noetic-rqt-gui-cpp
printf '\nROS_INSTALL_COMPLETE\n'
