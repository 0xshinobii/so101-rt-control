#include "arm_control/scripted_feetech_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <time.h>

namespace arm_control {
namespace {

int64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

}  // namespace

bool ScriptedFeetechBus::ping(uint8_t) {
  tx_packets_.fetch_add(1, std::memory_order_relaxed);
  rx_packets_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool ScriptedFeetechBus::read(uint8_t id, uint8_t addr, uint8_t length,
                              uint8_t* out) {
  if (fail_read()) return false;
  tx_packets_.fetch_add(1, std::memory_order_relaxed);
  rx_packets_.fetch_add(1, std::memory_order_relaxed);
  std::fill(out, out + length, 0);
  if (addr == FeetechBus::kAddrPresentPosition && length >= 2) {
    out[0] = static_cast<uint8_t>(positions_[id] & 0xFF);
    out[1] = static_cast<uint8_t>((positions_[id] >> 8) & 0xFF);
  }
  return true;
}

bool ScriptedFeetechBus::write(uint8_t id, uint8_t addr,
                               const uint8_t* data, uint8_t length) {
  if (mode_.load(std::memory_order_acquire) ==
      FaultMode::kReadWriteTimeout) {
    return fail_write(rx_timeout_ns_, failed_individual_writes_);
  }
  tx_packets_.fetch_add(1, std::memory_order_relaxed);
  rx_packets_.fetch_add(1, std::memory_order_relaxed);
  if (addr == FeetechBus::kAddrGoalPosition && length >= 2) {
    positions_[id] =
        static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
  }
  return true;
}

bool ScriptedFeetechBus::sync_read(const std::vector<uint8_t>& ids,
                                   uint8_t addr, uint8_t length,
                                   std::vector<uint8_t>& out) {
  if (fail_read()) return false;
  tx_packets_.fetch_add(1, std::memory_order_relaxed);
  rx_packets_.fetch_add(ids.size(), std::memory_order_relaxed);
  successful_reads_.fetch_add(1, std::memory_order_relaxed);
  out.assign(ids.size() * length, 0);
  if (addr == FeetechBus::kAddrPresentPosition && length >= 2) {
    for (size_t i = 0; i < ids.size(); ++i) {
      out[i * length] = static_cast<uint8_t>(positions_[ids[i]] & 0xFF);
      out[i * length + 1] =
          static_cast<uint8_t>((positions_[ids[i]] >> 8) & 0xFF);
    }
  }
  return true;
}

bool ScriptedFeetechBus::sync_write(const std::vector<uint8_t>& ids,
                                    uint8_t addr,
                                    const std::vector<uint8_t>& data) {
  if (mode_.load(std::memory_order_acquire) ==
      FaultMode::kReadWriteTimeout) {
    return fail_write(tx_timeout_ns_, failed_sync_writes_);
  }
  const size_t per = ids.empty() ? 0 : data.size() / ids.size();
  if (per == 0 || data.size() != per * ids.size()) return false;
  tx_packets_.fetch_add(1, std::memory_order_relaxed);
  successful_sync_writes_.fetch_add(1, std::memory_order_relaxed);
  if (addr == FeetechBus::kAddrGoalPosition && per >= 2) {
    for (size_t i = 0; i < ids.size(); ++i) {
      positions_[ids[i]] = static_cast<uint16_t>(
          data[i * per] | (static_cast<uint16_t>(data[i * per + 1]) << 8));
    }
  }
  return true;
}

FeetechStats ScriptedFeetechBus::stats() const {
  FeetechStats result;
  result.tx_packets = tx_packets_.load(std::memory_order_relaxed);
  result.rx_packets = rx_packets_.load(std::memory_order_relaxed);
  result.timeouts = timeouts_.load(std::memory_order_relaxed);
  return result;
}

void ScriptedFeetechBus::reset_stats() {
  tx_packets_.store(0, std::memory_order_relaxed);
  rx_packets_.store(0, std::memory_order_relaxed);
  timeouts_.store(0, std::memory_order_relaxed);
}

void ScriptedFeetechBus::inject(FaultMode mode, uint64_t failure_limit) {
  failures_left_.store(failure_limit, std::memory_order_relaxed);
  mode_.store(mode, std::memory_order_release);
  std::fprintf(stderr, "fault_injection mode=%d monotonic_ns=%lld\n",
               static_cast<int>(mode),
               static_cast<long long>(monotonic_ns()));
}

ScriptedFeetechBus::FaultStats ScriptedFeetechBus::fault_stats() const {
  FaultStats result;
  result.failed_reads = failed_reads_.load(std::memory_order_relaxed);
  result.failed_sync_writes =
      failed_sync_writes_.load(std::memory_order_relaxed);
  result.failed_individual_writes =
      failed_individual_writes_.load(std::memory_order_relaxed);
  result.successful_reads = successful_reads_.load(std::memory_order_relaxed);
  result.successful_sync_writes =
      successful_sync_writes_.load(std::memory_order_relaxed);
  return result;
}

bool ScriptedFeetechBus::fail_read() {
  if (mode_.load(std::memory_order_acquire) == FaultMode::kNone) return false;
  tx_packets_.fetch_add(1, std::memory_order_relaxed);
  wait_for_budget(rx_timeout_ns_);
  timeouts_.fetch_add(1, std::memory_order_relaxed);
  failed_reads_.fetch_add(1, std::memory_order_relaxed);
  std::fprintf(stderr, "fault_injection read_timeout monotonic_ns=%lld\n",
               static_cast<long long>(monotonic_ns()));
  consume_failure();
  return true;
}

bool ScriptedFeetechBus::fail_write(int64_t timeout_ns,
                                    std::atomic<uint64_t>& counter) {
  wait_for_budget(timeout_ns);
  timeouts_.fetch_add(1, std::memory_order_relaxed);
  counter.fetch_add(1, std::memory_order_relaxed);
  std::fprintf(stderr, "fault_injection write_timeout monotonic_ns=%lld\n",
               static_cast<long long>(monotonic_ns()));
  consume_failure();
  return false;
}

void ScriptedFeetechBus::consume_failure() {
  uint64_t left = failures_left_.load(std::memory_order_relaxed);
  if (left == 0) return;
  left = failures_left_.fetch_sub(1, std::memory_order_acq_rel);
  if (left == 1) mode_.store(FaultMode::kNone, std::memory_order_release);
}

void ScriptedFeetechBus::wait_for_budget(int64_t timeout_ns) {
  timespec remaining{timeout_ns / 1000000000LL, timeout_ns % 1000000000LL};
  while (nanosleep(&remaining, &remaining) != 0 && errno == EINTR) {
  }
}

}  // namespace arm_control
