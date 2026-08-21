#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "arm_control/hardware_backend.hpp"

#include "arm_control/rt_thread.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace arm_control {
namespace {

constexpr uint8_t kAddrAcceleration = 41;
constexpr uint8_t kAddrGoalSpeed = 46;
constexpr double kTicksPerRev = 4096.0;
constexpr double kRadPerTick = 2.0 * M_PI / kTicksPerRev;

double min_jerk(double s) {
  s = std::clamp(s, 0.0, 1.0);
  const double s3 = s * s * s;
  return 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s * s;
}

}  // namespace

HardwareBackend::HardwareBackend(Config cfg)
    : cfg_(std::move(cfg)),
      calib_(load_so101_calib(cfg_.calib_path)),
      q_(Eigen::VectorXd::Zero(kDof)),
      qdot_(Eigen::VectorXd::Zero(kDof)),
      q_cmd_(Eigen::VectorXd::Zero(kDof)) {
  ids_.resize(kDof);
  for (int i = 0; i < kDof; ++i) ids_[i] = static_cast<uint8_t>(calib_[i].id);
  bus_.open(cfg_.port, cfg_.baud);
  bus_.set_rx_timeout_ns(cfg_.rx_timeout_ns);
  for (int i = 0; i < kDof; ++i) {
    if (!bus_.ping(ids_[i])) {
      throw std::runtime_error("ping failed for id " +
                               std::to_string(ids_[i]));
    }
  }
}

HardwareBackend::~HardwareBackend() {
  try {
    disable_torque();
  } catch (...) {
  }
}

void HardwareBackend::fail_bus(const char* what) {
  ++bus_fails_;
  if (bus_fails_ >= cfg_.max_bus_fails) {
    disable_torque();
    throw std::runtime_error(std::string("bus failed: ") + what);
  }
}

void HardwareBackend::disable_torque() {
  const uint8_t off = 0;
  for (uint8_t id : ids_) {
    bus_.write(id, FeetechBus::kAddrTorqueEnable, &off, 1);
  }
}

void HardwareBackend::enable_torque_at_current() {
  std::array<int, kDof> ticks{};
  if (!read_ticks(ticks)) {
    throw std::runtime_error("cannot read pose before enabling torque");
  }
  write_ticks(ticks);
  // Homing uses the same profile as home_so101 (acc=30, uncapped speed).
  set_motion_profile(30, 0);
  for (uint8_t id : ids_) {
    const uint8_t on = 1;
    if (!bus_.write(id, FeetechBus::kAddrTorqueEnable, &on, 1)) {
      throw std::runtime_error("torque enable failed");
    }
  }
}

void HardwareBackend::set_motion_profile(uint8_t acc, uint16_t speed) {
  const uint8_t sp[2] = {static_cast<uint8_t>(speed & 0xFF),
                         static_cast<uint8_t>((speed >> 8) & 0xFF)};
  for (uint8_t id : ids_) {
    bus_.write(id, kAddrAcceleration, &acc, 1);
    bus_.write(id, kAddrGoalSpeed, sp, 2);
  }
}

bool HardwareBackend::read_ticks(std::array<int, kDof>& ticks) {
  std::vector<uint8_t> raw;
  if (!bus_.sync_read(ids_, FeetechBus::kAddrPresentPosition, 2, raw) ||
      raw.size() < 12) {
    return false;
  }
  for (int i = 0; i < kDof; ++i) {
    ticks[i] = static_cast<int>(raw[2 * i] | (raw[2 * i + 1] << 8));
  }
  return true;
}

bool HardwareBackend::write_ticks(const std::array<int, kDof>& ticks) {
  std::vector<uint8_t> data(12);
  for (int i = 0; i < kDof; ++i) {
    const int t = std::clamp(ticks[i], 0, 4095);
    data[2 * i] = static_cast<uint8_t>(t & 0xFF);
    data[2 * i + 1] = static_cast<uint8_t>((t >> 8) & 0xFF);
  }
  return bus_.sync_write(ids_, FeetechBus::kAddrGoalPosition, data);
}

void HardwareBackend::ticks_to_q(const std::array<int, kDof>& ticks,
                                 Eigen::VectorXd& q) const {
  q.resize(kDof);
  for (int i = 0; i < kDof; ++i) {
    q[i] = calib_[i].sign * (ticks[i] - calib_[i].zero_ticks) * kRadPerTick;
  }
}

int HardwareBackend::q_to_tick(int joint, double q) const {
  const auto& c = calib_[joint];
  int tick = c.zero_ticks +
             static_cast<int>(std::lround(c.sign * q / kRadPerTick));
  return std::clamp(tick, c.min_ticks, c.max_ticks);
}

void HardwareBackend::read_state(Eigen::VectorXd& q, Eigen::VectorXd& qdot) {
  std::array<int, kDof> ticks{};
  if (!read_ticks(ticks)) {
    fail_bus("sync_read");
    q = q_;
    qdot = qdot_;
    return;
  }
  bus_fails_ = 0;
  Eigen::VectorXd q_new(kDof);
  ticks_to_q(ticks, q_new);
  if (have_q_) {
    qdot_ = (q_new - q_) / cfg_.dt;
  } else {
    qdot_.setZero();
  }
  q_ = q_new;
  have_q_ = true;
  q = q_;
  qdot = qdot_;
}

void HardwareBackend::apply_torque(const Eigen::VectorXd& tau) {
  if (tau.size() != kDof) {
    throw std::invalid_argument("tau size");
  }
  if (!have_q_) {
    throw std::runtime_error("apply_torque before read_state");
  }
  if (!have_q_cmd_) {
    q_cmd_ = q_;
    have_q_cmd_ = true;
  }
  std::array<int, kDof> ticks{};
  for (int i = 0; i < kDof; ++i) {
    double dq = tau[i] / cfg_.k_servo;
    dq = std::clamp(dq, -cfg_.max_delta_q, cfg_.max_delta_q);
    q_cmd_[i] += dq;
    const auto& c = calib_[i];
    double q_lo = c.sign * (c.min_ticks - c.zero_ticks) * kRadPerTick;
    double q_hi = c.sign * (c.max_ticks - c.zero_ticks) * kRadPerTick;
    if (q_lo > q_hi) std::swap(q_lo, q_hi);
    q_cmd_[i] = std::clamp(q_cmd_[i], q_lo, q_hi);
    q_cmd_[i] = std::clamp(q_cmd_[i], q_[i] - cfg_.max_lead_q,
                           q_[i] + cfg_.max_lead_q);
  }
  q_cmd_[5] = 0.0;  // hold gripper at calibrated zero
  for (int i = 0; i < kDof; ++i) ticks[i] = q_to_tick(i, q_cmd_[i]);
  if (!write_ticks(ticks)) fail_bus("sync_write");
}

void HardwareBackend::write_goal_q(const Eigen::VectorXd& q) {
  if (q.size() != kDof) {
    throw std::invalid_argument("q size");
  }
  std::array<int, kDof> ticks{};
  for (int i = 0; i < kDof; ++i) {
    const double qi = (i == 5) ? 0.0 : q[i];
    ticks[i] = q_to_tick(i, qi);
  }
  if (!write_ticks(ticks)) fail_bus("sync_write");
}

void HardwareBackend::step() {
  const auto period = std::chrono::duration<double>(cfg_.dt);
  if (!period_armed_) {
    next_wakeup_ = std::chrono::steady_clock::now() +
                   std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       period);
    period_armed_ = true;
  } else {
    next_wakeup_ +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    sleep_until_monotonic(next_wakeup_);
  }
  t_ += cfg_.dt;
}

Eigen::Vector3d HardwareBackend::ee_position() {
  // Joint CSV is the hardware metric; Cartesian FK needs Pinocchio, which is
  // not packaged for Ubuntu 26.04 yet. Filled with zeros on purpose.
  return Eigen::Vector3d::Zero();
}

void HardwareBackend::reset() {
  enable_torque_at_current();
  std::array<int, kDof> start{};
  if (!read_ticks(start)) {
    throw std::runtime_error("reset: read failed");
  }
  Eigen::VectorXd q_start(kDof);
  ticks_to_q(start, q_start);
  const int rate = 50;
  const int steps =
      std::max(1, static_cast<int>(cfg_.home_duration * rate));
  const auto period = std::chrono::milliseconds(1000 / rate);
  auto next = std::chrono::steady_clock::now();
  for (int k = 0; k <= steps; ++k) {
    const double s = min_jerk(static_cast<double>(k) / steps);
    std::array<int, kDof> cmd{};
    for (int i = 0; i < kDof; ++i) {
      const double q = (1.0 - s) * q_start[i];
      cmd[i] = q_to_tick(i, q);
    }
    if (!write_ticks(cmd)) {
      throw std::runtime_error("reset: sync_write failed");
    }
    next += period;
    sleep_until_monotonic(next);
  }
  std::array<int, kDof> now{};
  if (!read_ticks(now)) {
    throw std::runtime_error("reset: read failed after home");
  }
  ticks_to_q(now, q_cmd_);
  std::printf("home q=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n", q_cmd_[0], q_cmd_[1],
              q_cmd_[2], q_cmd_[3], q_cmd_[4], q_cmd_[5]);
  have_q_cmd_ = true;
  have_q_ = false;
  qdot_.setZero();
  t_ = 0.0;
  period_armed_ = false;
  bus_fails_ = 0;
}

}  // namespace arm_control
