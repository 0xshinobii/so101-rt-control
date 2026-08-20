// Shared plain-data types for the control core. Kept dependency-light (no ROS,
// no MuJoCo) so both the headless core and the ROS 2 node use the same records.
#pragma once
#include <array>
#include <limits>

namespace arm_control {

// SO-101 controllable DOF: 5 arm joints + the held gripper (matches the 6
// motors in so101_torque.xml and the Phase 1.5 oracle's model.nu).
inline constexpr int kDof = 6;

// One control-iteration record. Logging convention mirrors the Phase 1.5 oracle
// exactly: q/qdot/tau are the PRE-step state and the torque computed from it;
// t and ee are read POST-step. Fixed-size arrays -> trivially copyable, safe to
// pass through a lock-free ring with no heap.
struct Sample {
  double t = 0.0;
  std::array<double, kDof> q{};
  std::array<double, kDof> qd{};
  std::array<double, kDof> tau{};
  std::array<double, kDof> estimated_disturbance_torque{};
  std::array<double, kDof> true_disturbance_torque{};
  std::array<double, 3> external_force{};
  std::array<double, 3> estimated_external_force{};
  double jacobian_condition = 0.0;
  std::array<double, 3> ee{};
  double estimated_payload_mass =
      std::numeric_limits<double>::quiet_NaN();
};

}  // namespace arm_control
