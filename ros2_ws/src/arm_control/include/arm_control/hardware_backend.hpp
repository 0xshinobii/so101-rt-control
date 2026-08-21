// STS3215 serial-bus plant. Torque commands are realized with a
// torque-to-position bridge (servos stay in Mode 0).
#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "arm_control/feetech_bus.hpp"
#include "arm_control/plant_interface.hpp"
#include "arm_control/so101_calib.hpp"

namespace arm_control {

class PinocchioDynamics;

class HardwareBackend : public PlantInterface {
public:
  struct Config {
    std::string port = "/dev/ttyACM0";
    int baud = 1000000;
    std::string calib_path;
    std::string urdf_path = "models/so101/so101_dynamics.urdf";
    double dt = 0.005;           // frozen f_hw = 200 Hz
    double k_servo = 50.0;       // N·m/rad; per-joint from g(q)/droop later
    double max_delta_q = 0.03;   // rad per control step
    double max_lead_q = 0.12;    // rad; Goal_Position stays near present q
    int goal_speed = 40;         // Feetech units; 0 = unlimited (unsafe)
    int64_t rx_timeout_ns = 5130000;
    int max_bus_fails = 3;
    double home_duration = 4.0;
  };

  explicit HardwareBackend(Config cfg);
  ~HardwareBackend() override;

  HardwareBackend(const HardwareBackend&) = delete;
  HardwareBackend& operator=(const HardwareBackend&) = delete;

  void read_state(Eigen::VectorXd& q, Eigen::VectorXd& qdot) override;
  void apply_torque(const Eigen::VectorXd& tau) override;
  void write_goal_q(const Eigen::VectorXd& q);
  void gravity(const Eigen::VectorXd& q, Eigen::VectorXd& tau) const;
  void step() override;
  Eigen::Vector3d ee_position() override;
  void reset() override;

  int dof() const override { return kDof; }
  double timestep() const override { return cfg_.dt; }
  double time() const override { return t_; }

private:
  void disable_torque();
  void enable_torque_at_current();
  void set_motion_profile(uint8_t acc, uint16_t speed);
  bool read_ticks(std::array<int, kDof>& ticks);
  bool write_ticks(const std::array<int, kDof>& ticks);
  void ticks_to_q(const std::array<int, kDof>& ticks, Eigen::VectorXd& q) const;
  int q_to_tick(int joint, double q) const;
  void fail_bus(const char* what);

  Config cfg_;
  std::unique_ptr<PinocchioDynamics> dynamics_;
  std::array<JointCalib, kDof> calib_{};
  std::vector<uint8_t> ids_;
  FeetechBus bus_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_cmd_;
  bool have_q_ = false;
  bool have_q_cmd_ = false;
  int bus_fails_ = 0;
  double t_ = 0.0;
  bool period_armed_ = false;
  std::chrono::steady_clock::time_point next_wakeup_{};
};

}  // namespace arm_control
