# Phase 2 dev/deploy image: ROS 2 Jazzy (Ubuntu 24.04) + Eigen + the MuJoCo C
# library, pinned to 3.10.0 to match the Python wheel used by the Phase 1.5
# oracle. Same-engine + same-inputs is what makes the delta~0 validation mean
# something. This is also the Linux base Phase 3's RT work will build on.
FROM ros:jazzy-ros-base

ARG MUJOCO_VERSION=3.10.0
# Container runs under Docker Desktop on Apple Silicon -> linux/arm64.
ARG MUJOCO_ARCH=aarch64

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      wget \
      ca-certificates \
      libeigen3-dev \
      python3-colcon-common-extensions \
      python3-numpy \
 && rm -rf /var/lib/apt/lists/*

# MuJoCo C library -> /opt/mujoco (headers in include/, libmujoco.so in lib/).
RUN mkdir -p /opt/mujoco \
 && wget -q "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${MUJOCO_ARCH}.tar.gz" \
      -O /tmp/mujoco.tar.gz \
 && tar -xzf /tmp/mujoco.tar.gz -C /opt/mujoco --strip-components=1 \
 && rm /tmp/mujoco.tar.gz

# Let CMake find MuJoCo and the runtime linker load libmujoco.so.
ENV MUJOCO_DIR=/opt/mujoco
ENV LD_LIBRARY_PATH=/opt/mujoco/lib:${LD_LIBRARY_PATH}

WORKDIR /work
CMD ["/bin/bash"]
