#!/usr/bin/env bash
# Phase 2 regression + Phase 4 build and validation, end to end.
#
#   1. build the image (ROS 2 Jazzy + Eigen + Pinocchio + MuJoCo)
#   2. colcon build the workspace (arm_msgs, arm_control, arm_bringup)
#   3. gate empty and known-payload rigid-body model equivalence
#   4. retain the exact Phase 2 C++ vs Python PD regression
#   5. run the four-case bounded-reference Phase 4 matrix
#   6. validate computed-torque tracking, stability, and saturation
#
# The repo is bind-mounted at /work, so all outputs land back on the host.
# Requires the oracle CSV to exist first (run on the host):
#   python run_baseline_so101.py --csv oracle_baseline_so101.csv
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=so101-dev:jazzy

echo "== [1/6] build image =="
docker build -t "${IMAGE}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}/docker"

echo "== [2-6] build + run + validate in container =="
docker run --rm -v "${REPO_ROOT}:/work" -w /work "${IMAGE}" bash -lc '
  source /opt/ros/jazzy/setup.bash
  set -eo pipefail
  cd /work/ros2_ws
  echo "== [2/6] colcon build =="
  colcon build --cmake-args -DMUJOCO_DIR=/opt/mujoco
  source install/setup.bash
  runner=./install/arm_control/lib/arm_control/main_headless
  gate=./install/arm_control/lib/arm_control/validate_dynamics

  echo "== [3/6] rigid-body equivalence gates =="
  $gate /work/models/so101/scene_torque.xml \
      /work/models/so101/so101_dynamics.urdf
  $gate /work/models/so101/scene_torque_payload.xml \
      /work/models/so101/so101_dynamics_payload.urdf

  echo "== [4/6] Phase 2 regression =="
  $runner \
      /work/models/so101/scene_torque.xml /work/cpp_baseline_so101.csv
  python3 /work/tools/arm_bench.py \
      --cpp /work/cpp_baseline_so101.csv \
      --oracle /work/oracle_baseline_so101.csv

  echo "== [5/6] Phase 4 controller matrix =="
  $runner /work/models/so101/scene_torque.xml \
      /work/cpp_pd_empty_smooth.csv --reference smooth
  $runner /work/models/so101/scene_torque.xml \
      /work/cpp_ct_empty_so101.csv --controller computed_torque \
      --reference smooth --urdf /work/models/so101/so101_dynamics.urdf
  $runner /work/models/so101/scene_torque_payload.xml \
      /work/cpp_pd_payload_smooth.csv --reference smooth
  $runner /work/models/so101/scene_torque_payload.xml \
      /work/cpp_ct_payload_so101.csv --controller computed_torque \
      --reference smooth \
      --urdf /work/models/so101/so101_dynamics_payload.urdf

  echo "== [6/6] Phase 4 acceptance =="
  python3 /work/tools/phase4_bench.py \
      --pd-empty /work/cpp_pd_empty_smooth.csv \
      --ct-empty /work/cpp_ct_empty_so101.csv \
      --pd-payload /work/cpp_pd_payload_smooth.csv \
      --ct-payload /work/cpp_ct_payload_so101.csv \
      --oracle /work/oracle_baseline_so101.csv
'
echo "== done =="
