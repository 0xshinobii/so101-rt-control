#!/usr/bin/env bash
# Phase 7: same Jazzy+Pinocchio image as sim, on the i7 (x86_64) host kernel.
# The container does not replace PREEMPT_RT — it uses 7.0.0-30-realtime.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=so101-dev:jazzy

docker build -t "${IMAGE}" --build-arg MUJOCO_ARCH=x86_64 \
  -f "${ROOT}/docker/Dockerfile" "${ROOT}/docker"

exec docker run --rm -it \
  -v "${ROOT}:/work" -w /work \
  --device=/dev/ttyACM0 \
  --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  "${IMAGE}" "$@"
