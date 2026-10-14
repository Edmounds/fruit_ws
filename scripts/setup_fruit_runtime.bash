#!/usr/bin/env bash

# Build a deterministic ROS runtime environment instead of inheriting paths
# from other workspaces, ~/.local pip packages, or /usr/local OpenCV builds.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "setup_fruit_runtime.bash must be sourced, not executed" >&2
  exit 1
fi

if [[ -n "${VIRTUAL_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
  echo "Deactivate the Python training environment before starting ROS" >&2
  return 1
fi

if [[ "$(command -v python3)" != "/usr/bin/python3" ]]; then
  echo "ROS runtime requires /usr/bin/python3; check PATH before sourcing" >&2
  return 1
fi

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PKG_CONFIG_PATH
unset PYTHONPATH
unset ROS_PACKAGE_PATH

source /opt/ros/humble/setup.bash
source /home/rc4/fruit_ws/install/local_setup.bash

if [[ -r /home/rc4/fruit_ws/install/fruit_robviz_bridge/lib/libfoxglove_service_timeout.so ]]; then
  export LD_PRELOAD="${LD_PRELOAD:+${LD_PRELOAD}:}/home/rc4/fruit_ws/install/fruit_robviz_bridge/lib/libfoxglove_service_timeout.so"
fi

if [[ ! -r /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]]; then
  echo "ROS runtime requires ros-humble-rmw-cyclonedds-cpp" >&2
  echo "Install it with: sudo apt install ros-humble-rmw-cyclonedds-cpp" >&2
  return 1
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export PYTHONNOUSERSITE=1
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/usr/lib/python3/dist-packages"
export PIP_REQUIRE_VIRTUALENV=true

python3 -c '
import cv2
import numpy

assert cv2.__version__ == "4.5.4", (cv2.__version__, cv2.__file__)
assert cv2.__file__.startswith("/usr/lib/python3/dist-packages/"), cv2.__file__
assert numpy.__version__.split(".", maxsplit=1)[0] == "1", numpy.__version__
assert numpy.__file__.startswith("/usr/lib/python3/dist-packages/"), numpy.__file__
'
