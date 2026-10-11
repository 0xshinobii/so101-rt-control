#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "arm_control/feetech_bus.hpp"

namespace arm_control {

// Deterministic test bus. It behaves like an instant position servo until a
// fault is armed, then consumes the configured transaction deadline before
// returning failure.
class ScriptedFeetechBus final : public FeetechBusInterface {
public:
  enum class FaultMode { kNone, kReadTimeout, kReadWriteTimeout };

  struct FaultStats {
    uint64_t failed_reads = 0;
    uint64_t failed_sync_writes = 0;
    uint64_t failed_individual_writes = 0;
    uint64_t successful_reads = 0;
    uint64_t successful_sync_writes = 0;
  };

  void open(const std::string&, int) override { open_ = true; }
  void close() override { open_ = false; }
  bool is_open() const override { return open_; }
  bool ping(uint8_t) override;
  bool read(uint8_t id, uint8_t addr, uint8_t length, uint8_t* out) override;
  bool write(uint8_t id, uint8_t addr, const uint8_t* data,
             uint8_t length) override;
  bool sync_read(const std::vector<uint8_t>& ids, uint8_t addr,
                 uint8_t length, std::vector<uint8_t>& out) override;
  bool sync_write(const std::vector<uint8_t>& ids, uint8_t addr,
                  const std::vector<uint8_t>& data) override;

  FeetechStats stats() const override;
  void reset_stats() override;
  void set_rx_timeout_ns(int64_t ns) override { rx_timeout_ns_ = ns; }
  void set_tx_timeout_ns(int64_t ns) override { tx_timeout_ns_ = ns; }

  // failure_limit == 0 means remain faulty until explicitly cleared.
  void inject(FaultMode mode, uint64_t failure_limit = 0);
  FaultStats fault_stats() const;

private:
  bool fail_read();
  bool fail_write(int64_t timeout_ns, std::atomic<uint64_t>& counter);
  void consume_failure();
  static void wait_for_budget(int64_t timeout_ns);

  bool open_ = false;
  std::array<uint16_t, 256> positions_{};
  std::atomic<FaultMode> mode_{FaultMode::kNone};
  std::atomic<uint64_t> failures_left_{0};
  std::atomic<uint64_t> tx_packets_{0};
  std::atomic<uint64_t> rx_packets_{0};
  std::atomic<uint64_t> timeouts_{0};
  std::atomic<uint64_t> failed_reads_{0};
  std::atomic<uint64_t> failed_sync_writes_{0};
  std::atomic<uint64_t> failed_individual_writes_{0};
  std::atomic<uint64_t> successful_reads_{0};
  std::atomic<uint64_t> successful_sync_writes_{0};
  int64_t rx_timeout_ns_ = 2500000;
  int64_t tx_timeout_ns_ = 750000;
};

}  // namespace arm_control
