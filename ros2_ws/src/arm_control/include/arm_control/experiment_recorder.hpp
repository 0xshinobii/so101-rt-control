#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "arm_control/arm_types.hpp"

namespace arm_control {

struct ExperimentHealth {
  int stale_reads = 0;
  int failed_writes = 0;
  int tick_clamps = 0;
  int lead_saturations = 0;
  int late_ticks = 0;
  double max_late_us = 0.0;
  uint64_t bus_timeouts = 0;
  uint64_t bus_checksum_errors = 0;
  uint64_t torque_off_failures = 0;
};

class ExperimentRecorder {
public:
  struct Config {
    double duration_s = 4.0;
    double settle_time_s = 1.5;
    double payload_g = -1.0;
    double rate_hz = 200.0;
    std::string output_directory = "experiment_logs";
    std::array<double, kDof> target{};
    std::array<double, 3> target_ee{};
  };

  struct SaveResult {
    std::string csv_path;
    double arm_rms = 0.0;
    double ee_miss = 0.0;
    double compensated_mass = 0.0;
  };

  void prepare(Config config);
  // RT-thread operation: fixed-size copy only. True means the run is complete.
  bool record(const Sample& sample) noexcept;
  bool complete() const noexcept {
    return complete_.load(std::memory_order_acquire);
  }
  SaveResult save(const std::string& controller, uint64_t activation,
                  int64_t started_unix_ms, double compensated_mass,
                  const ExperimentHealth& health) const;
  void clear();

private:
  Config config_;
  std::vector<Sample> samples_;
  std::atomic<size_t> sample_count_{0};
  std::atomic<bool> complete_{false};
};

}  // namespace arm_control
