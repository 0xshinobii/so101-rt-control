// STS3215 serial-bus plant. Torque commands are realized with a
// torque-to-position bridge (servos stay in Mode 0).
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
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
    // N·m/rad, per joint. Only the elbow (index 2) is identified from
    // g(q)/droop; the rest are frozen placeholders, not a calibrated map.
    std::array<double, kDof> k_servo = {50.0, 90.0, 11.0, 50.0, 50.0, 50.0};
    double max_lead_q = 0.12;    // rad; Goal_Position stays near present q
    int goal_speed = 40;         // Feetech units; 0 = unlimited (unsafe)
    int64_t rx_timeout_ns = 2500000;  // full six-servo read; max measured 1.96 ms
    int64_t tx_timeout_ns = 750000;   // kernel-queue drain; max measured 0.29 ms
    int max_bus_fails = 3;
    double home_duration = 4.0;
    double gripper_q = 0.0;      // held during home + tracking
    bool gripper_closed = false; // true: hold calib min_ticks
    int gripper_torque_limit = 200;  // 0–1000; 200 = 20% (70 g pinch)
    double current_lsb_a = 0.0065;  // STS3215 Present_Current
    // N·m per amp, joint frame. Gripper 0: jaw current is not payload gravity.
    std::array<double, kDof> kt_nm_per_a = {1.0, 1.0, 1.0, 1.0, 1.0, 0.0};
  };

  explicit HardwareBackend(Config cfg);
  ~HardwareBackend() override;

  HardwareBackend(const HardwareBackend&) = delete;
  HardwareBackend& operator=(const HardwareBackend&) = delete;

  void read_state(Eigen::VectorXd& q, Eigen::VectorXd& qdot) override;
  void set_reference_position(const Eigen::VectorXd& q_des) override;
  void apply_torque(const Eigen::VectorXd& tau) override;
  void gravity(const Eigen::VectorXd& q, Eigen::VectorXd& tau) const;
  void move_to(const Eigen::VectorXd& q_end, double duration);
  void set_motion_profile(uint8_t acc, uint16_t speed);
  bool read_current_amps(Eigen::VectorXd& amps);
  void current_to_torque(const Eigen::VectorXd& amps,
                         Eigen::VectorXd& tau) const;
  void step() override;
  Eigen::Vector3d ee_position() override;
  void reset() override;

  // Run health. Every one of these is a silent failure without a counter:
  // a stale read duplicates the previous sample into the log, a tick clamp
  // truncates a command, and a late tick means the logged time axis (a
  // nominal counter) has drifted from real time.
  struct RunHealth {
    int stale_reads = 0;      // sync_read failures that returned the last q
    int failed_writes = 0;    // sync_write failures; prior command remained
    int tick_clamps = 0;      // goals truncated to [min_ticks, max_ticks]
    int lead_saturations = 0; // ticks where tau/K_servo hit +/- max_lead_q
    int late_ticks = 0;       // iterations that missed the period deadline
    double max_late_us = 0.0; // worst overrun
    uint64_t bus_timeouts = 0;
    uint64_t bus_checksum_errors = 0;
  };
  RunHealth health() const;
  void reset_health();

  double gripper_q() const { return cfg_.gripper_q; }
  int dof() const override { return kDof; }
  double timestep() const override { return cfg_.dt; }
  double time() const override { return t_; }

private:
  void disable_torque();
  void enable_torque_at_current();
  void set_torque_limit(uint16_t limit);
  void squeeze_gripper();
  bool read_ticks(std::array<int, kDof>& ticks);
  bool write_ticks(const std::array<int, kDof>& ticks);
  void ticks_to_q(const std::array<int, kDof>& ticks, Eigen::VectorXd& q) const;
  int q_to_tick(int joint, double q) const;
  void fail_bus(const char* what, int& consecutive_failures);
  void begin_io_cycle();
  int64_t capped_timeout_ns(int64_t budget_ns) const;

  Config cfg_;
  std::unique_ptr<PinocchioDynamics> dynamics_;
  std::array<JointCalib, kDof> calib_{};
  std::vector<uint8_t> ids_;
  FeetechBus bus_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_cmd_;
  Eigen::VectorXd q_ref_;          // last reference handed to apply_torque
  Eigen::VectorXd q_new_;          // read_state scratch; kept off the heap
  std::vector<uint8_t> rx_buf_;    // sync_read scratch
  std::vector<uint8_t> tx_buf_;    // sync_write scratch
  mutable int tick_clamps_ = 0;    // q_to_tick is const; the counter is not
  int lead_saturations_ = 0;
  int stale_reads_ = 0;
  int failed_writes_ = 0;
  int late_ticks_ = 0;
  int64_t max_late_ns_ = 0;
  bool have_q_ = false;
  bool have_q_cmd_ = false;
  int read_bus_fails_ = 0;
  int write_bus_fails_ = 0;
  int gripper_hold_tick_ = -1;
  double t_ = 0.0;
  bool period_armed_ = false;
  std::chrono::steady_clock::time_point next_wakeup_{};
  int64_t cycle_deadline_ns_ = 0;
};

}  // namespace arm_control
