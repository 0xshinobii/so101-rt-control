#include "arm_control/experiment_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace arm_control {

void ExperimentRecorder::prepare(Config config) {
  if (!std::isfinite(config.duration_s) ||
      !std::isfinite(config.settle_time_s) || config.duration_s <= 0.0 ||
      config.settle_time_s < 0.0 ||
      config.settle_time_s >= config.duration_s ||
      !std::isfinite(config.rate_hz) || config.rate_hz <= 0.0) {
    throw std::invalid_argument(
        "experiment requires duration_s > settle_time_s >= 0 and rate_hz > 0");
  }
  if (config.output_directory.empty()) {
    throw std::invalid_argument(
        "experiment.output_directory must not be empty");
  }
  if (!std::isfinite(config.payload_g) || config.payload_g < -1.0) {
    throw std::invalid_argument(
        "experiment.payload_g must be -1 (unknown) or nonnegative");
  }
  std::filesystem::create_directories(config.output_directory);
  config_ = std::move(config);
  samples_.resize(
      static_cast<size_t>(std::ceil(config_.duration_s * config_.rate_hz)));
  sample_count_.store(0, std::memory_order_relaxed);
  complete_.store(false, std::memory_order_relaxed);
}

bool ExperimentRecorder::record(const Sample& sample) noexcept {
  const size_t index = sample_count_.load(std::memory_order_relaxed);
  if (index >= samples_.size()) {
    return false;
  }
  samples_[index] = sample;
  sample_count_.store(index + 1, std::memory_order_release);
  if (index + 1 == samples_.size()) {
    complete_.store(true, std::memory_order_release);
    return true;
  }
  return false;
}

ExperimentRecorder::SaveResult ExperimentRecorder::save(
    const std::string& controller, uint64_t activation,
    int64_t started_unix_ms, double compensated_mass,
    const ExperimentHealth& health) const {
  const size_t count = std::min(
      sample_count_.load(std::memory_order_acquire), samples_.size());
  const std::string run_id =
      controller + "_activation_" + std::to_string(activation) + "_" +
      std::to_string(started_unix_ms);
  const std::filesystem::path output_dir(config_.output_directory);
  const std::filesystem::path csv_path = output_dir / (run_id + ".csv");

  std::array<double, 4> joint_squared_error{};
  size_t settled_count = 0;
  for (size_t index = 0; index < count; ++index) {
    if (samples_[index].t < config_.settle_time_s) continue;
    for (int joint = 0; joint < 4; ++joint) {
      const double error =
          samples_[index].q[joint] - config_.target[joint];
      joint_squared_error[joint] += error * error;
    }
    ++settled_count;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 4> joint_rms{};
  double arm_rms = nan;
  if (settled_count > 0) {
    double squared_sum = 0.0;
    for (int joint = 0; joint < 4; ++joint) {
      joint_rms[joint] = std::sqrt(
          joint_squared_error[joint] / static_cast<double>(settled_count));
      squared_sum += joint_rms[joint] * joint_rms[joint];
    }
    arm_rms = std::sqrt(squared_sum / 4.0);
  } else {
    joint_rms.fill(nan);
  }

  double ee_miss = nan;
  double estimated_mass = nan;
  double actual_duration = 0.0;
  if (count > 0) {
    const auto& last = samples_[count - 1];
    double squared_miss = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double error = last.ee[axis] - config_.target_ee[axis];
      squared_miss += error * error;
    }
    ee_miss = std::sqrt(squared_miss);
    estimated_mass = last.estimated_payload_mass;
    actual_duration = last.t;
  }

  const bool clean =
      health.stale_reads == 0 && health.failed_writes == 0 &&
      health.tick_clamps == 0 && health.lead_saturations == 0 &&
      health.late_ticks == 0 && health.bus_timeouts == 0 &&
      health.bus_checksum_errors == 0 && health.torque_off_failures == 0;

  std::ofstream out(csv_path);
  if (!out) throw std::runtime_error("cannot open " + csv_path.string());
  out.precision(17);
  out << "# run_id=" << run_id << " controller=" << controller
      << " payload_g=" << config_.payload_g
      << " planned_duration_s=" << config_.duration_s
      << " settle_time_s=" << config_.settle_time_s
      << " completed=" << (complete() ? 1 : 0)
      << " rms_no_roll_rad=" << arm_rms << " ee_miss_m=" << ee_miss
      << " estimated_mass_raw_kg=" << estimated_mass
      << " compensated_mass_kg=" << compensated_mass
      << " clean=" << (clean ? 1 : 0) << '\n';
  out << "t";
  for (int joint = 0; joint < kDof; ++joint) out << ",q" << joint;
  for (int joint = 0; joint < kDof; ++joint) out << ",qd" << joint;
  for (int joint = 0; joint < kDof; ++joint) out << ",tau" << joint;
  out << ",ee_x,ee_y,ee_z,estimated_payload_mass\n";
  for (size_t index = 0; index < count; ++index) {
    const auto& sample = samples_[index];
    out << sample.t;
    for (double value : sample.q) out << ',' << value;
    for (double value : sample.qd) out << ',' << value;
    for (double value : sample.tau) out << ',' << value;
    out << ',' << sample.ee[0] << ',' << sample.ee[1] << ','
        << sample.ee[2] << ',' << sample.estimated_payload_mass << '\n';
  }

  const std::filesystem::path summary_path = output_dir / "summary.csv";
  const bool write_header =
      !std::filesystem::exists(summary_path) ||
      std::filesystem::file_size(summary_path) == 0;
  std::ofstream summary(summary_path, std::ios::app);
  if (!summary) {
    throw std::runtime_error("cannot open " + summary_path.string());
  }
  summary.precision(17);
  if (write_header) {
    summary
        << "run_id,controller,payload_g,completed,samples,actual_duration_s,"
           "rms_no_roll_rad,joint0_rms_rad,joint1_rms_rad,joint2_rms_rad,"
           "joint3_rms_rad,ee_miss_m,estimated_mass_raw_kg,"
           "compensated_mass_kg,stale_reads,failed_writes,tick_clamps,"
           "lead_saturations,late_ticks,max_late_us,bus_timeouts,"
           "bus_checksum_errors,clean,csv_path\n";
  }
  summary << run_id << ',' << controller << ',' << config_.payload_g << ','
          << (complete() ? 1 : 0) << ',' << count << ',' << actual_duration
          << ',' << arm_rms;
  for (double value : joint_rms) summary << ',' << value;
  summary << ',' << ee_miss << ',' << estimated_mass << ','
          << compensated_mass << ',' << health.stale_reads << ','
          << health.failed_writes << ',' << health.tick_clamps << ','
          << health.lead_saturations << ',' << health.late_ticks << ','
          << health.max_late_us << ',' << health.bus_timeouts << ','
          << health.bus_checksum_errors << ',' << (clean ? 1 : 0) << ','
          << csv_path.string() << '\n';
  return {csv_path.string(), arm_rms, ee_miss, compensated_mass};
}

void ExperimentRecorder::clear() {
  samples_.clear();
  sample_count_.store(0, std::memory_order_relaxed);
  complete_.store(false, std::memory_order_relaxed);
}

}  // namespace arm_control
