#!/usr/bin/env bash
# Phase 2 regression + Phase 4/5/6 build and validation, end to end.
#
#   1. build the image (ROS 2 Jazzy + Eigen + Pinocchio + MuJoCo)
#   2. colcon build the workspace (arm_msgs, arm_control, arm_bringup)
#   3. gate empty and known-payload rigid-body model equivalence
#   4. retain the exact Phase 2 C++ vs Python PD regression
#   5. run the four-case bounded-reference Phase 4 matrix
#   6. validate computed-torque tracking, stability, and saturation
#   7. run the deterministic RLS validator and unknown-payload matrix
#   8. validate mass identification and tracking recovery
#   9. validate the momentum DOB and run the disturbance matrix
#  10. validate rejection, bandwidth, force sensing, and transfer
#
# The repo is bind-mounted at /work, so all outputs land back on the host.
# Requires the oracle CSV to exist first (run on the host):
#   python run_baseline_so101.py --csv oracle_baseline_so101.csv
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=so101-dev:jazzy

echo "== [1/10] build image =="
docker build -t "${IMAGE}" -f "${REPO_ROOT}/docker/Dockerfile" "${REPO_ROOT}/docker"

echo "== [2-10] build + run + validate in container =="
docker run --rm -v "${REPO_ROOT}:/work" -w /work "${IMAGE}" bash -lc '
  source /opt/ros/jazzy/setup.bash
  set -eo pipefail
  cd /work/ros2_ws
  echo "== [2/10] colcon build =="
  colcon build --cmake-args -DMUJOCO_DIR=/opt/mujoco
  source install/setup.bash
  runner=./install/arm_control/lib/arm_control/main_headless
  gate=./install/arm_control/lib/arm_control/validate_dynamics

  echo "== [3/10] rigid-body equivalence gates =="
  $gate /work/models/so101/scene_torque.xml \
      /work/models/so101/so101_dynamics.urdf
  $gate /work/models/so101/scene_torque_payload.xml \
      /work/models/so101/so101_dynamics_payload.urdf

  echo "== [4/10] Phase 2 regression =="
  $runner \
      /work/models/so101/scene_torque.xml /work/cpp_baseline_so101.csv
  python3 /work/tools/arm_bench.py \
      --cpp /work/cpp_baseline_so101.csv \
      --oracle /work/oracle_baseline_so101.csv

  echo "== [5/10] Phase 4 controller matrix =="
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

  echo "== [6/10] Phase 4 acceptance =="
  python3 /work/tools/phase4_bench.py \
      --pd-empty /work/cpp_pd_empty_smooth.csv \
      --ct-empty /work/cpp_ct_empty_so101.csv \
      --pd-payload /work/cpp_pd_payload_smooth.csv \
      --ct-payload /work/cpp_ct_payload_so101.csv \
      --oracle /work/oracle_baseline_so101.csv

  echo "== [7/10] Phase 5 estimator + unknown-payload matrix =="
  ./install/arm_control/lib/arm_control/validate_payload_estimator
  $runner /work/models/so101/scene_torque.xml \
      /work/cpp_adaptive_empty_so101.csv \
      --controller adaptive_computed_torque --reference smooth \
      --urdf /work/models/so101/so101_dynamics.urdf \
      --payload-urdf /work/models/so101/so101_dynamics_payload.urdf
  $runner /work/models/so101/scene_torque_payload.xml \
      /work/cpp_ct_mismatch_010_so101.csv \
      --controller computed_torque --reference smooth \
      --urdf /work/models/so101/so101_dynamics.urdf \
      --plant-payload-mass 0.10
  $runner /work/models/so101/scene_torque_payload.xml \
      /work/cpp_adaptive_010_so101.csv \
      --controller adaptive_computed_torque --reference smooth \
      --urdf /work/models/so101/so101_dynamics.urdf \
      --payload-urdf /work/models/so101/so101_dynamics_payload.urdf \
      --plant-payload-mass 0.10
  $runner /work/models/so101/scene_torque_payload.xml \
      /work/cpp_ct_mismatch_020_so101.csv \
      --controller computed_torque --reference smooth \
      --urdf /work/models/so101/so101_dynamics.urdf \
      --plant-payload-mass 0.20
  $runner /work/models/so101/scene_torque_payload.xml \
      /work/cpp_adaptive_020_so101.csv \
      --controller adaptive_computed_torque --reference smooth \
      --urdf /work/models/so101/so101_dynamics.urdf \
      --payload-urdf /work/models/so101/so101_dynamics_payload.urdf \
      --plant-payload-mass 0.20

  echo "== [8/10] Phase 5 acceptance =="
  python3 /work/tools/phase5_bench.py \
      --empty-adaptive /work/cpp_adaptive_empty_so101.csv \
      --mismatch-010 /work/cpp_ct_mismatch_010_so101.csv \
      --adaptive-010 /work/cpp_adaptive_010_so101.csv \
      --mismatch-020 /work/cpp_ct_mismatch_020_so101.csv \
      --adaptive-020 /work/cpp_adaptive_020_so101.csv \
      --known-020 /work/cpp_ct_payload_so101.csv \
      --oracle /work/oracle_baseline_so101.csv

  echo "== [9/10] Phase 6 DOB validator + matrix =="
  dob_gate=./install/arm_control/lib/arm_control/validate_disturbance_observer
  empty=/work/models/so101/scene_torque.xml
  payload=/work/models/so101/scene_torque_payload.xml
  urdf=/work/models/so101/so101_dynamics.urdf
  payload_urdf=/work/models/so101/so101_dynamics_payload.urdf
  $dob_gate $urdf $empty

  $runner $empty /work/p6_a.csv --controller computed_torque \
      --reference smooth --urdf $urdf --duration 5
  $runner $empty /work/p6_b.csv --controller computed_torque \
      --reference smooth --urdf $urdf --force-onset 2.5 \
      --force 3 0 0 --duration 5
  $runner $empty /work/p6_c.csv --controller computed_torque_dob \
      --reference smooth --urdf $urdf --force-onset 2.5 \
      --force 3 0 0 --duration 5
  $runner $empty /work/p6_d.csv --controller adaptive_computed_torque \
      --reference smooth --urdf $urdf --payload-urdf $payload_urdf \
      --force-onset 2.5 --force 3 0 0 --duration 5
  $runner $empty /work/p6_dv.csv --controller adaptive_computed_torque \
      --reference smooth --urdf $urdf --payload-urdf $payload_urdf \
      --force-onset 2.5 --force 0 0 -3 --duration 5
  $runner $payload /work/p6_e.csv --controller adaptive_computed_torque \
      --reference smooth --urdf $urdf --payload-urdf $payload_urdf \
      --plant-payload-mass 0.20 --duration 5
  $runner $payload /work/p6_e_dob.csv \
      --controller adaptive_computed_torque_dob --reference smooth \
      --urdf $urdf --payload-urdf $payload_urdf \
      --plant-payload-mass 0.20 --freeze-rls-at 2.5 --duration 5
  $runner $payload /work/p6_f.csv --controller adaptive_computed_torque \
      --reference smooth --urdf $urdf --payload-urdf $payload_urdf \
      --plant-payload-mass 0.20 --force-onset 2.5 --force 3 0 0 \
      --freeze-rls-at 2.5 --duration 5
  $runner $payload /work/p6_g.csv --controller computed_torque_dob \
      --reference smooth --urdf $urdf --plant-payload-mass 0.20 \
      --force-onset 2.5 --force 3 0 0 --duration 5
  $runner $payload /work/p6_h.csv \
      --controller adaptive_computed_torque_dob --reference smooth \
      --urdf $urdf --payload-urdf $payload_urdf \
      --plant-payload-mass 0.20 --force-onset 2.5 --force 3 0 0 \
      --freeze-rls-at 2.5 --duration 5

  for spec in "05 0.5 13.5" "2 2 6" "12 12 5"; do
    set -- $spec
    tag=$1
    frequency=$2
    duration=$3
    $runner $empty /work/p6_bw_${tag}.csv --controller computed_torque \
        --reference smooth --urdf $urdf --force-onset 2.5 \
        --force 3 0 0 --force-frequency-hz $frequency \
        --duration $duration
    $runner $empty /work/p6_cw_${tag}.csv \
        --controller computed_torque_dob --reference smooth --urdf $urdf \
        --force-onset 2.5 --force 3 0 0 \
        --force-frequency-hz $frequency --duration $duration
  done
  $runner $payload /work/p6_hw_2.csv \
      --controller adaptive_computed_torque_dob --reference smooth \
      --urdf $urdf --payload-urdf $payload_urdf \
      --plant-payload-mass 0.20 --force-onset 2.5 --force 3 0 0 \
      --force-frequency-hz 2 --freeze-rls-at 2.5 --duration 6
  $runner $payload /work/p6_g_xfer.csv --controller computed_torque_dob \
      --reference smooth --urdf $urdf --plant-payload-mass 0.20 \
      --freeze-dob-at 2.5 --second-target-onset 2.5 \
      --second-target -0.6 -0.7 0.8 -0.5 -0.4 0 --duration 5
  $runner $payload /work/p6_h_xfer.csv \
      --controller adaptive_computed_torque_dob --reference smooth \
      --urdf $urdf --payload-urdf $payload_urdf \
      --plant-payload-mass 0.20 --freeze-rls-at 2.5 \
      --freeze-dob-at 2.5 --second-target-onset 2.5 \
      --second-target -0.6 -0.7 0.8 -0.5 -0.4 0 --duration 5

  echo "== [10/10] Phase 6 acceptance =="
  python3 /work/tools/phase6_bench.py \
      --a /work/p6_a.csv --b /work/p6_b.csv --c /work/p6_c.csv \
      --d /work/p6_d.csv --dv /work/p6_dv.csv \
      --e /work/p6_e.csv --e-dob /work/p6_e_dob.csv \
      --f /work/p6_f.csv --g /work/p6_g.csv --h /work/p6_h.csv \
      --b-omega /work/p6_bw_05.csv /work/p6_bw_2.csv /work/p6_bw_12.csv \
      --c-omega /work/p6_cw_05.csv /work/p6_cw_2.csv /work/p6_cw_12.csv \
      --h-omega /work/p6_hw_2.csv \
      --g-xfer /work/p6_g_xfer.csv --h-xfer /work/p6_h_xfer.csv \
      --frequency-output /work/phase6_frequency_response.csv
'
echo "== done =="
