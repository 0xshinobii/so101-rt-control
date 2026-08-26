#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "arm_control/hardware_backend.hpp"

#include "arm_control/pinocchio_dynamics.hpp"
#include "arm_control/rt_thread.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <time.h>

namespace arm_control {
namespace {

constexpr uint8_t kAddrAcceleration = 41;
constexpr uint8_t kAddrGoalSpeed = 46;
constexpr double kTicksPerRev = 4096.0;
constexpr double kRadPerTick = 2.0 * M_PI / kTicksPerRev;

double decode_sts_present_current_a(uint8_t lo, uint8_t hi, double lsb) {
  const uint16_t raw = static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
  const int mag = static_cast<int>(raw & 0x7FFF);
  const int counts = (raw & 0x8000u) ? -mag : mag;
  return static_cast<double>(counts) * lsb;
}

double min_jerk(double s) {
  s = std::clamp(s, 0.0, 1.0);
  const double s3 = s * s * s;
  return 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s * s;
}

int64_t monotonic_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

}  // namespace

HardwareBackend::HardwareBackend(Config cfg)
    : cfg_(std::move(cfg)),
      dynamics_(std::make_unique<PinocchioDynamics>(cfg_.urdf_path)),
      calib_(load_so101_calib(cfg_.calib_path)),
      q_(Eigen::VectorXd::Zero(kDof)),
      qdot_(Eigen::VectorXd::Zero(kDof)),
      q_cmd_(Eigen::VectorXd::Zero(kDof)),
      q_ref_(Eigen::VectorXd::Zero(kDof)),
      q_new_(Eigen::VectorXd::Zero(kDof)) {
  // Size the packet scratch once so the 200 Hz path never calls the allocator.
  rx_buf_.reserve(64);
  tx_buf_.assign(2 * kDof, 0);
  ids_.resize(kDof);
  for (int i = 0; i < kDof; ++i) ids_[i] = static_cast<uint8_t>(calib_[i].id);
  bus_.open(cfg_.port, cfg_.baud);
  bus_.set_rx_timeout_ns(cfg_.rx_timeout_ns);
  bus_.set_tx_timeout_ns(cfg_.tx_timeout_ns);
  if (cfg_.gripper_closed) {
    const auto& g = calib_[5];
    gripper_hold_tick_ = g.min_ticks;
    cfg_.gripper_q =
        g.sign * (g.min_ticks - g.zero_ticks) * kRadPerTick;
  }
  std::printf("gripper hold q=%.3f rad  tick=%d  torque_limit=%d%s\n",
              cfg_.gripper_q,
              gripper_hold_tick_ >= 0 ? gripper_hold_tick_ : -1,
              cfg_.gripper_torque_limit,
              cfg_.gripper_closed ? " (min_ticks)" : "");
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

int64_t HardwareBackend::capped_timeout_ns(int64_t budget_ns) const {
  if (cycle_deadline_ns_ <= 0) return budget_ns;
  const int64_t remaining = cycle_deadline_ns_ - monotonic_ns();
  if (remaining <= 0) return 0;
  return std::min(budget_ns, remaining);
}

void HardwareBackend::begin_io_cycle() {
  cycle_deadline_ns_ =
      monotonic_ns() + static_cast<int64_t>(cfg_.dt * 1e9);
}

void HardwareBackend::fail_bus(const char* what, int& consecutive_failures) {
  ++consecutive_failures;
  if (consecutive_failures >= cfg_.max_bus_fails) {
    disable_torque();
    throw std::runtime_error(std::string("bus failed: ") + what);
  }
}

void HardwareBackend::disable_torque() {
  bus_.set_rx_timeout_ns(cfg_.rx_timeout_ns);
  bus_.set_tx_timeout_ns(cfg_.tx_timeout_ns);
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
  set_torque_limit(1000);
  for (uint8_t id : ids_) {
    const uint8_t on = 1;
    if (!bus_.write(id, FeetechBus::kAddrTorqueEnable, &on, 1)) {
      throw std::runtime_error("torque enable failed");
    }
  }
}

void HardwareBackend::set_torque_limit(uint16_t limit) {
  const uint8_t arm[2] = {static_cast<uint8_t>(limit & 0xFF),
                          static_cast<uint8_t>((limit >> 8) & 0xFF)};
  const uint16_t g_lim = static_cast<uint16_t>(
      std::clamp(cfg_.gripper_torque_limit, 1, 1000));
  const uint8_t grip[2] = {static_cast<uint8_t>(g_lim & 0xFF),
                           static_cast<uint8_t>((g_lim >> 8) & 0xFF)};
  for (int i = 0; i < kDof; ++i) {
    bus_.write(ids_[i], FeetechBus::kAddrTorqueLimit,
               (i == 5) ? grip : arm, 2);
  }
}

void HardwareBackend::squeeze_gripper() {
  if (gripper_hold_tick_ < 0) return;
  std::array<int, kDof> ticks{};
  if (!read_ticks(ticks)) {
    throw std::runtime_error("squeeze: read failed");
  }
  ticks[5] = gripper_hold_tick_;
  if (!write_ticks(ticks)) {
    throw std::runtime_error("squeeze: write failed");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
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
  if (!bus_.sync_read(ids_, FeetechBus::kAddrPresentPosition, 2, rx_buf_) ||
      rx_buf_.size() < 12) {
    return false;
  }
  for (int i = 0; i < kDof; ++i) {
    ticks[i] = static_cast<int>(rx_buf_[2 * i] | (rx_buf_[2 * i + 1] << 8));
  }
  return true;
}

bool HardwareBackend::write_ticks(const std::array<int, kDof>& ticks) {
  std::vector<uint8_t>& data = tx_buf_;
  for (int i = 0; i < kDof; ++i) {
    const int t = std::clamp(
        (i == 5 && gripper_hold_tick_ >= 0) ? gripper_hold_tick_ : ticks[i], 0,
        4095);
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
  const int tick = c.zero_ticks +
                   static_cast<int>(std::lround(c.sign * q / kRadPerTick));
  const int clamped = std::clamp(tick, c.min_ticks, c.max_ticks);
  // A truncated goal looks exactly like a tracking failure in the log, so it
  // has to be counted rather than applied silently.
  if (clamped != tick) ++tick_clamps_;
  return clamped;
}

void HardwareBackend::read_state(Eigen::VectorXd& q, Eigen::VectorXd& qdot) {
  begin_io_cycle();
  bus_.set_rx_timeout_ns(capped_timeout_ns(cfg_.rx_timeout_ns));
  std::array<int, kDof> ticks{};
  if (!read_ticks(ticks)) {
    // Returning the previous state keeps the loop alive, but the caller will
    // log it as if it were a fresh measurement. Count it so the run can say so.
    ++stale_reads_;
    fail_bus("sync_read", read_bus_fails_);
    q = q_;
    qdot = qdot_;
    return;
  }
  read_bus_fails_ = 0;
  ticks_to_q(ticks, q_new_);
  if (have_q_) {
    qdot_ = (q_new_ - q_) / cfg_.dt;
  } else {
    qdot_.setZero();
  }
  q_ = q_new_;
  have_q_ = true;
  q = q_;
  qdot = qdot_;
}

void HardwareBackend::set_reference_position(const Eigen::VectorXd& q_des) {
  if (q_des.size() != kDof) {
    throw std::invalid_argument("q_des size");
  }
  q_ref_ = q_des;
}

// The STS3215 has no closed-loop N·m mode, so this project realizes torque as a
// bounded position lead on the reference:
//
//     q_cmd = q_des + clamp(tau / K_servo, +/- max_lead_q)
//
// This is the single bridge implementation. Callers reach it through
// PlantInterface, having supplied q_des via set_reference_position().
void HardwareBackend::apply_torque(const Eigen::VectorXd& tau) {
  if (tau.size() != kDof) {
    throw std::invalid_argument("tau size");
  }
  if (!have_q_) {
    throw std::runtime_error("apply_torque before read_state");
  }
  std::array<int, kDof> ticks{};
  for (int i = 0; i < kDof; ++i) {
    const double raw_lead = tau[i] / cfg_.k_servo[i];
    const double lead =
        std::clamp(raw_lead, -cfg_.max_lead_q, cfg_.max_lead_q);
    // Saturating here means the controller asked for more authority than the
    // bridge can express -- the clip that actually shapes the tracking result.
    if (lead != raw_lead) ++lead_saturations_;
    q_cmd_[i] = q_ref_[i] + lead;
  }
  q_cmd_[5] = cfg_.gripper_q;  // the jaw is held, never driven by tau
  have_q_cmd_ = true;
  bus_.set_tx_timeout_ns(capped_timeout_ns(cfg_.tx_timeout_ns));
  for (int i = 0; i < kDof; ++i) ticks[i] = q_to_tick(i, q_cmd_[i]);
  if (!write_ticks(ticks)) {
    ++failed_writes_;
    fail_bus("sync_write", write_bus_fails_);
  } else {
    write_bus_fails_ = 0;
  }
}

HardwareBackend::RunHealth HardwareBackend::health() const {
  RunHealth h;
  h.stale_reads = stale_reads_;
  h.failed_writes = failed_writes_;
  h.tick_clamps = tick_clamps_;
  h.lead_saturations = lead_saturations_;
  h.late_ticks = late_ticks_;
  h.max_late_us = static_cast<double>(max_late_ns_) / 1000.0;
  h.bus_timeouts = bus_.stats().timeouts;
  h.bus_checksum_errors = bus_.stats().checksum_errors;
  return h;
}

void HardwareBackend::reset_health() {
  stale_reads_ = 0;
  failed_writes_ = 0;
  tick_clamps_ = 0;
  lead_saturations_ = 0;
  late_ticks_ = 0;
  max_late_ns_ = 0;
  read_bus_fails_ = 0;
  write_bus_fails_ = 0;
  bus_.reset_stats();
}

void HardwareBackend::gravity(const Eigen::VectorXd& q,
                              Eigen::VectorXd& tau) const {
  dynamics_->gravity(q, tau);
}

void HardwareBackend::step() {
  const auto period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(cfg_.dt));
  // Arm on the first call WITHOUT consuming a period, then sleep on every call
  // including the first. Arming to now+period and returning early made the
  // first interval ~0 ms and the second ~10 ms, and no counter could see it.
  if (!period_armed_) {
    next_wakeup_ = std::chrono::steady_clock::now();
    period_armed_ = true;
  }
  next_wakeup_ += period;
  // t_ advances by exactly dt whether or not the deadline was met, so the
  // logged time axis is nominal. Count overruns rather than let the log
  // quietly claim 200 Hz that the bus did not deliver.
  const auto now = std::chrono::steady_clock::now();
  if (now > next_wakeup_) {
    ++late_ticks_;
    const int64_t late = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now - next_wakeup_).count();
    if (late > max_late_ns_) max_late_ns_ = late;
  }
  sleep_until_monotonic(next_wakeup_);
  t_ += cfg_.dt;
}

Eigen::Vector3d HardwareBackend::ee_position() {
  if (!have_q_) return Eigen::Vector3d::Zero();
  return dynamics_->ee_position(q_);
}

void HardwareBackend::reset() {
  enable_torque_at_current();
  squeeze_gripper();
  Eigen::VectorXd home = Eigen::VectorXd::Zero(kDof);
  home[5] = cfg_.gripper_q;
  move_to(home, cfg_.home_duration);
  std::array<int, kDof> now{};
  if (!read_ticks(now)) {
    throw std::runtime_error("reset: read failed after home");
  }
  ticks_to_q(now, q_cmd_);
  std::printf("home q=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n", q_cmd_[0], q_cmd_[1],
              q_cmd_[2], q_cmd_[3], q_cmd_[4], q_cmd_[5]);
  // Stream a smooth min-jerk Goal_Position. acc>0 restarts the ramp every
  // 5 ms rewrite and crawls; speed=40 was ~0.06 rad/s and missed kTarget.
  set_motion_profile(0, 0);
  set_torque_limit(1000);
  squeeze_gripper();
  have_q_cmd_ = true;
  have_q_ = false;
  qdot_.setZero();
  t_ = 0.0;
  period_armed_ = false;
  read_bus_fails_ = 0;
  write_bus_fails_ = 0;
}

void HardwareBackend::move_to(const Eigen::VectorXd& q_end_in, double duration) {
  if (q_end_in.size() != kDof) {
    throw std::invalid_argument("q size");
  }
  Eigen::VectorXd q_end = q_end_in;
  q_end[5] = cfg_.gripper_q;
  std::array<int, kDof> start{};
  if (!read_ticks(start)) {
    throw std::runtime_error("move_to: read failed");
  }
  Eigen::VectorXd q_start(kDof);
  ticks_to_q(start, q_start);
  const int rate = 50;
  const int steps = std::max(1, static_cast<int>(duration * rate));
  const auto period = std::chrono::milliseconds(1000 / rate);
  auto next = std::chrono::steady_clock::now();
  for (int k = 0; k <= steps; ++k) {
    const double s = min_jerk(static_cast<double>(k) / steps);
    std::array<int, kDof> cmd{};
    for (int i = 0; i < kDof; ++i) {
      const double q = (1.0 - s) * q_start[i] + s * q_end[i];
      cmd[i] = q_to_tick(i, q);
    }
    if (!write_ticks(cmd)) {
      throw std::runtime_error("move_to: sync_write failed");
    }
    next += period;
    sleep_until_monotonic(next);
  }
  have_q_ = false;
  have_q_cmd_ = true;
  q_cmd_ = q_end;
  qdot_.setZero();
  period_armed_ = false;
}

bool HardwareBackend::read_current_amps(Eigen::VectorXd& amps) {
  amps.resize(kDof);
  std::vector<uint8_t> raw;
  if (!bus_.sync_read(ids_, FeetechBus::kAddrPresentCurrent, 2, raw) ||
      raw.size() < 12) {
    return false;
  }
  for (int i = 0; i < kDof; ++i) {
    amps[i] = decode_sts_present_current_a(raw[2 * i], raw[2 * i + 1],
                                           cfg_.current_lsb_a);
  }
  return true;
}

void HardwareBackend::current_to_torque(const Eigen::VectorXd& amps,
                                        Eigen::VectorXd& tau) const {
  tau.resize(kDof);
  for (int i = 0; i < kDof; ++i) {
    tau[i] = static_cast<double>(calib_[i].sign) * cfg_.kt_nm_per_a[i] * amps[i];
  }
}

}  // namespace arm_control
