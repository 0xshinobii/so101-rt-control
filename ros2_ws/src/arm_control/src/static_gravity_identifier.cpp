#include "arm_control/static_gravity_identifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <utility>

#include "arm_control/arm_types.hpp"
#include "arm_control/hardware_backend.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

namespace arm_control {
namespace {

constexpr std::array<std::array<double, 5>, 6> kIdentificationPoses{{
    {{-0.55, 0.50, -0.75, 0.00, 0.00}},
    {{0.55, 0.25, -1.50, 0.75, 0.00}},
    {{-0.35, -0.25, 0.25, -0.50, 0.00}},
    {{0.35, 1.00, -1.50, -0.50, 0.00}},
    {{-0.15, -0.25, -0.25, 0.75, 0.00}},
    {{0.15, -0.25, -0.50, 0.00, 0.00}},
}};

Eigen::VectorXd make_pose(const std::array<double, 5>& arm,
                          double gripper_q) {
  Eigen::VectorXd pose(kDof);
  for (int joint = 0; joint < 5; ++joint) {
    pose[joint] = arm[static_cast<size_t>(joint)];
  }
  pose[5] = gripper_q;
  return pose;
}

}  // namespace

StaticGravityIdentifier::StaticGravityIdentifier(
    const std::string& empty_urdf_path,
    const std::string& payload_urdf_path, Config config)
    : config_(config),
      empty_urdf_path_(empty_urdf_path),
      payload_urdf_path_(payload_urdf_path) {
  if (!std::isfinite(config_.move_duration_s) ||
      config_.move_duration_s <= 0.0 ||
      !std::isfinite(config_.settle_duration_s) ||
      config_.settle_duration_s <= 0.0 ||
      !std::isfinite(config_.approach_rad) || config_.approach_rad <= 0.0 ||
      config_.samples <= 0 || config_.sample_period.count() <= 0 ||
      !std::isfinite(config_.quiet_velocity_threshold_rad_s) ||
      config_.quiet_velocity_threshold_rad_s < 0.0 ||
      config_.quiet_samples <= 0 || !std::isfinite(config_.quiet_timeout_s) ||
      config_.quiet_timeout_s <= 0.0 ||
      !std::isfinite(config_.reference_payload_mass) ||
      config_.reference_payload_mass <= 0.0) {
    throw std::invalid_argument("invalid static-gravity identifier config");
  }
}

bool StaticGravityIdentifier::arm_is_quiet(
    const Eigen::VectorXd& qdot) const {
  if (qdot.size() != kDof) return false;
  for (int joint = 0; joint < 5; ++joint) {
    if (!std::isfinite(qdot[joint]) ||
        std::abs(qdot[joint]) >
            config_.quiet_velocity_threshold_rad_s) {
      return false;
    }
  }
  return true;
}

void StaticGravityIdentifier::wait_until_quiet(
    HardwareBackend& hardware) const {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(config_.quiet_timeout_s);
  Eigen::VectorXd q(kDof);
  Eigen::VectorXd qdot(kDof);
  int consecutive_quiet = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(config_.sample_period);
    hardware.read_state(q, qdot);
    consecutive_quiet =
        arm_is_quiet(qdot) ? consecutive_quiet + 1 : 0;
    if (consecutive_quiet >= config_.quiet_samples) return;
  }
  throw std::runtime_error(
      "arm did not settle before payload-current sampling");
}

Eigen::VectorXd StaticGravityIdentifier::average_current(
    HardwareBackend& hardware) const {
  Eigen::VectorXd sum = Eigen::VectorXd::Zero(kDof);
  Eigen::VectorXd amps(kDof);
  Eigen::VectorXd q(kDof);
  Eigen::VectorXd qdot(kDof);
  int accepted = 0;
  for (int sample = 0; sample < config_.samples; ++sample) {
    std::this_thread::sleep_for(config_.sample_period);
    hardware.read_state(q, qdot);
    if (!arm_is_quiet(qdot)) {
      throw std::runtime_error(
          "arm moved during payload-current sampling");
    }
    if (hardware.read_current_amps(amps)) {
      sum += amps;
      ++accepted;
    }
  }
  if (accepted < (config_.samples + 1) / 2) {
    throw std::runtime_error(
        "too few successful current samples during payload identification");
  }
  return sum / static_cast<double>(accepted);
}

Eigen::VectorXd StaticGravityIdentifier::hold_current(
    HardwareBackend& hardware, const Eigen::VectorXd& pose,
    double approach_sign) const {
  Eigen::VectorXd via = pose;
  for (int joint = 1; joint <= 3; ++joint) {
    via[joint] += approach_sign * config_.approach_rad;
  }
  hardware.move_to_operator(via, config_.move_duration_s);
  hardware.move_to(pose, config_.move_duration_s);
  std::this_thread::sleep_for(
      std::chrono::duration<double>(config_.settle_duration_s));
  wait_until_quiet(hardware);
  return average_current(hardware);
}

StaticGravityIdentifier::Result StaticGravityIdentifier::identify(
    HardwareBackend& hardware, Calibration calibration) const {
  if (!std::isfinite(calibration.raw_mass_scale) ||
      calibration.raw_mass_scale <= 0.0 ||
      !std::isfinite(calibration.raw_mass_offset) ||
      !std::isfinite(calibration.min_mass) ||
      !std::isfinite(calibration.max_mass) ||
      calibration.min_mass > calibration.max_mass) {
    throw std::invalid_argument("invalid static-gravity mass calibration");
  }

  PinocchioDynamics empty_dynamics(empty_urdf_path_);
  PinocchioDynamics payload_dynamics(payload_urdf_path_);
  Result result;
  result.observations.reserve(kIdentificationPoses.size());
  hardware.set_motion_profile(30, 0);

  double numerator = 0.0;
  double denominator = 0.0;
  for (const auto& arm_pose : kIdentificationPoses) {
    const Eigen::VectorXd pose = make_pose(arm_pose, hardware.gripper_q());
    const Eigen::VectorXd amps_positive =
        hold_current(hardware, pose, 1.0);
    const Eigen::VectorXd amps_negative =
        hold_current(hardware, pose, -1.0);

    Observation observation{
        Eigen::VectorXd(kDof), Eigen::VectorXd(kDof),
        Eigen::VectorXd(kDof), Eigen::VectorXd(kDof),
        Eigen::VectorXd(kDof)};
    Eigen::VectorXd qdot(kDof);
    Eigen::VectorXd payload_gravity(kDof);
    hardware.read_state(observation.q, qdot);
    hardware.current_to_torque(
        0.5 * (amps_positive + amps_negative),
        observation.measured_torque);
    empty_dynamics.gravity(observation.q, observation.empty_gravity);
    payload_dynamics.gravity(observation.q, payload_gravity);
    observation.payload_regressor =
        (payload_gravity - observation.empty_gravity) /
        config_.reference_payload_mass;
    observation.extra_torque =
        observation.measured_torque - observation.empty_gravity;
    observation.payload_regressor[5] = 0.0;
    observation.extra_torque[5] = 0.0;
    numerator +=
        observation.payload_regressor.dot(observation.extra_torque);
    denominator += observation.payload_regressor.squaredNorm();
    result.observations.push_back(std::move(observation));
  }

  if (denominator < 1e-8) {
    throw std::runtime_error("gravity regressor is too small");
  }
  result.raw_mass = numerator / denominator;
  result.estimated_mass = std::clamp(
      (result.raw_mass - calibration.raw_mass_offset) /
          calibration.raw_mass_scale,
      calibration.min_mass, calibration.max_mass);
  return result;
}

}  // namespace arm_control
