# Third-party components

This repository is MIT licensed (see [LICENSE](LICENSE)) **except** for the
vendored robot description below, which carries its own licence. That directory
is redistributed unmodified in its licensing metadata: the upstream `LICENSE`,
`README.md` and `CHANGELOG.md` are preserved in place.

## `models/so101/` — SO-101 robot description

| | |
|---|---|
| Upstream | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), `robotstudio_so101` |
| Original hardware | [The Robot Studio SO-ARM100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100) |
| Licence | **Apache License 2.0** — full text at [`models/so101/LICENSE`](models/so101/LICENSE) |
| Includes | MJCF, URDF and the `assets/*.stl` meshes |

### Local additions

The following files in that directory were **added by this project** and are
covered by this repository's MIT licence, not Apache-2.0. They are derivatives of
the upstream MJCF scene and are kept alongside it so that MuJoCo's relative asset
paths resolve:

- `so101_torque.xml`, `so101_torque_payload.xml` — torque-actuated variants
  (`<motor>` actuators at ±2.94 N·m, the real STS3215 envelope)
- `scene_torque.xml`, `scene_torque_payload.xml`, `scene_box.xml` — scenes
- `so101_dynamics.urdf`, `so101_dynamics_payload.urdf` — URDFs consumed by
  Pinocchio as the *controller's* model

## Build-time dependencies (not vendored)

Fetched by [`docker/Dockerfile`](docker/Dockerfile); none are redistributed here.

| Component | Licence |
|---|---|
| ROS 2 Jazzy Jalisco | Apache-2.0 |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`ros-jazzy-pinocchio`) | BSD-2-Clause |
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.10.0 C library | Apache-2.0 |
| [Eigen](https://eigen.tuxfamily.org) 3 | MPL-2.0 |
