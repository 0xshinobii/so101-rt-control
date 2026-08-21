// Gravity torque g(q) for the empty SO-101, matching so101_dynamics.urdf.
// Eigen-only (no Pinocchio) so hardware_run still builds on Ubuntu 26.04.
#pragma once

#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "arm_control/arm_types.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace arm_control {
namespace {

struct So101Link {
  Eigen::Vector3d xyz;
  Eigen::Matrix3d rpy;
  Eigen::Vector3d com;
  double mass;
};

inline Eigen::Matrix3d urdf_rpy(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// camera_mount (fixed, 0.012 kg) is lumped into gripper_link.
inline const std::array<So101Link, kDof>& so101_links() {
  static const std::array<So101Link, kDof> kLinks = {{
      {{0.0388353, 0, 0.0624},
       urdf_rpy(M_PI, 0, -M_PI),
       {-0.0307604, -0.000017, -0.0252713},
       0.100006},
      {{-0.0303992, -0.0182778, -0.0542},
       urdf_rpy(-M_PI / 2, -M_PI / 2, 0),
       {-0.089847, -0.008382, 0.018409},
       0.103},
      {{-0.11257, -0.028, 0},
       urdf_rpy(0, 0, M_PI / 2),
       {-0.098070, 0.0032438, 0.018283},
       0.104},
      {{-0.1349, 0.0052, 0},
       urdf_rpy(0, 0, -M_PI / 2),
       {-0.000103312, -0.0386143, 0.0281156},
       0.079},
      {{0, -0.0611, 0.0181},
       urdf_rpy(M_PI / 2, 0.04867951485346193, M_PI),
       {0.00007514, 0.006295, -0.026429},
       0.099},
      {{0.0202, 0.0188, -0.0234},
       urdf_rpy(M_PI / 2, 0, 0),
       {-0.001575, -0.0300244, 0.0192755},
       0.012},
  }};
  return kLinks;
}

}  // namespace

// τ = g(q) [N·m] in controller joint order. qdot = qddot = 0.
inline void so101_gravity(const Eigen::VectorXd& q, Eigen::VectorXd& tau) {
  tau.resize(kDof);
  const auto& links = so101_links();
  const Eigen::Vector3d grav(0.0, 0.0, -9.81);

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, kDof> origin{};
  std::array<Eigen::Vector3d, kDof> axis{};
  std::array<Eigen::Vector3d, kDof> com{};

  for (int i = 0; i < kDof; ++i) {
    const Eigen::Matrix3d R_joint =
        links[i].rpy * Eigen::AngleAxisd(q[i], Eigen::Vector3d::UnitZ());
    p = p + R * links[i].xyz;
    R = R * R_joint;
    origin[i] = p;
    axis[i] = R * Eigen::Vector3d::UnitZ();
    com[i] = p + R * links[i].com;
  }

  for (int i = 0; i < kDof; ++i) {
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
    for (int k = i; k < kDof; ++k) {
      m += (com[k] - origin[i]).cross(links[k].mass * grav);
    }
    tau[i] = axis[i].dot(m);
  }
}

}  // namespace arm_control
