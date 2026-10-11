// Feetech SMS/STS protocol 1 (Dynamixel-v1 shaped) over POSIX serial.
// Used by Phase 7 bus timing and the hardware PlantInterface backend.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace arm_control {

struct FeetechStats {
  uint64_t tx_packets = 0;
  uint64_t rx_packets = 0;
  uint64_t checksum_errors = 0;
  uint64_t timeouts = 0;
};

class FeetechBusInterface {
public:
  virtual ~FeetechBusInterface() = default;

  virtual void open(const std::string& device, int baud) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual bool ping(uint8_t id) = 0;
  virtual bool read(uint8_t id, uint8_t addr, uint8_t length,
                    uint8_t* out) = 0;
  virtual bool write(uint8_t id, uint8_t addr, const uint8_t* data,
                     uint8_t length) = 0;
  virtual bool sync_read(const std::vector<uint8_t>& ids, uint8_t addr,
                         uint8_t length, std::vector<uint8_t>& out) = 0;
  virtual bool sync_write(const std::vector<uint8_t>& ids, uint8_t addr,
                          const std::vector<uint8_t>& data) = 0;
  virtual FeetechStats stats() const = 0;
  virtual void reset_stats() = 0;
  virtual void set_rx_timeout_ns(int64_t ns) = 0;
  virtual void set_tx_timeout_ns(int64_t ns) = 0;
};

class FeetechBus : public FeetechBusInterface {
public:
  static constexpr uint8_t kInstPing = 1;
  static constexpr uint8_t kInstRead = 2;
  static constexpr uint8_t kInstWrite = 3;
  static constexpr uint8_t kInstSyncWrite = 0x83;
  static constexpr uint8_t kInstSyncRead = 0x82;
  static constexpr uint8_t kBroadcastId = 0xFE;
  static constexpr uint8_t kAddrTorqueEnable = 40;
  static constexpr uint8_t kAddrGoalPosition = 42;
  static constexpr uint8_t kAddrTorqueLimit = 48;  // 2 B, 1000 = 100%
  static constexpr uint8_t kAddrPresentPosition = 56;
  static constexpr uint8_t kAddrPresentSpeed = 58;
  static constexpr uint8_t kAddrPresentLoad = 60;     // 2 B, PWM duty; not amps
  static constexpr uint8_t kAddrPresentCurrent = 69;  // 2 B, sign-magnitude, 6.5 mA/LSB

  FeetechBus() = default;
  ~FeetechBus() override;
  FeetechBus(const FeetechBus&) = delete;
  FeetechBus& operator=(const FeetechBus&) = delete;

  // Termios rates this bus can program. Not every rate the servo accepts.
  static bool baud_supported(int baud);

  void open(const std::string& device, int baud) override;
  void close() override;
  bool is_open() const override { return fd_ >= 0; }

  bool ping(uint8_t id) override;
  bool read(uint8_t id, uint8_t addr, uint8_t length, uint8_t* out) override;
  bool write(uint8_t id, uint8_t addr, const uint8_t* data,
             uint8_t length) override;
  bool sync_read(const std::vector<uint8_t>& ids, uint8_t addr, uint8_t length,
                 std::vector<uint8_t>& out) override;
  bool sync_write(const std::vector<uint8_t>& ids, uint8_t addr,
                  const std::vector<uint8_t>& data) override;

  FeetechStats stats() const override { return stats_; }
  void reset_stats() override { stats_ = {}; }

  // Deadline budget for one complete request/response transaction [ns].
  void set_rx_timeout_ns(int64_t ns) override { rx_timeout_ns_ = ns; }
  // Deadline budget for a broadcast write with no status response [ns].
  void set_tx_timeout_ns(int64_t ns) override { tx_timeout_ns_ = ns; }

private:
  bool wait_ready(short events, int64_t deadline_ns);
  bool tx(const uint8_t* data, size_t n, int64_t deadline_ns);
  bool drain_until(int64_t deadline_ns);
  bool rx_byte(uint8_t& b, int64_t deadline_ns);
  bool rx_status(uint8_t expected_id, uint8_t* data, uint8_t data_len,
                 int64_t deadline_ns);
  static uint8_t checksum(const uint8_t* id_through_params, size_t n);

  // Reused packet scratch. resize() on an already-large-enough vector does
  // not allocate, so the steady-state hot path is allocation-free.
  std::vector<uint8_t> pkt_;
  int fd_ = -1;
  int64_t rx_timeout_ns_ = 2500000;  // measured max 1.96 ms
  int64_t tx_timeout_ns_ = 750000;   // measured max 0.29 ms
  FeetechStats stats_;
};

}  // namespace arm_control
