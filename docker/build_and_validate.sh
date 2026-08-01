#!/usr/bin/env bash
# Phase 2 build + validate, end to end, inside the ros:jazzy container.
#
#   1. build the image (ROS 2 Jazzy + Eigen + MuJoCo 3.10.0 C lib)
#   2. colcon build the workspace (arm_msgs, arm_control, arm_bringup)
#   3. run the headless C++ control core -> cpp_baseline_so101.csv
#   4. arm_bench: C++ trajectory vs the Phase 1.5 oracle (delta ~ 0)
#
# The repo is bind-mounted at /work, so all outputs land back on the host.
# Requires the oracle CSV to exist first (run on the host):
#   python run_baseline_so101.py --csv oracle_baseline_so101.csv
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=so101-dev:jazzy

echo "== [1/4] build image =="
docker build -t "${IMAGE}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}/docker"

echo "== [2-4] build + run + validate in container =="
docker run --rm -v "${REPO_ROOT}:/work" -w /work "${IMAGE}" bash -lc '
  set -euo pipefail
  source /opt/ros/jazzy/setup.bash
  cd /work/ros2_ws
  echo "== [2/4] colcon build =="
  colcon build --cmake-args -DMUJOCO_DIR=/opt/mujoco
  source install/setup.bash

  echo "== [3/4] run headless C++ core =="
  ./install/arm_control/lib/arm_control/main_headless \
      /work/models/so101/scene_torque.xml /work/cpp_baseline_so101.csv

  echo "== [4/4] validate vs oracle =="
  python3 /work/tools/arm_bench.py \
      --cpp /work/cpp_baseline_so101.csv \
      --oracle /work/oracle_baseline_so101.csv
'
echo "== done =="
