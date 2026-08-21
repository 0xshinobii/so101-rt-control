# Development image: ROS 2 Jazzy + Eigen + Pinocchio + the MuJoCo C library,
# pinned to 3.10.0 to match the Python wheel used by the Phase 1.5 oracle.
FROM ros:jazzy-ros-base

ARG MUJOCO_VERSION=3.10.0
# Apple Silicon Docker Desktop: aarch64. Bare-metal i7 (Phase 7): x86_64.
#   docker build -t so101-dev:jazzy --build-arg MUJOCO_ARCH=x86_64 \
#     -f docker/Dockerfile docker
ARG MUJOCO_ARCH=aarch64

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      wget \
      ca-certificates \
      libeigen3-dev \
      ros-jazzy-pinocchio \
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
ENV LD_LIBRARY_PATH=/opt/mujoco/lib

WORKDIR /work
CMD ["/bin/bash"]
