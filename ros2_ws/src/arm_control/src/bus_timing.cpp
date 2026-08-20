// Phase 7: measure Feetech 6-servo sync-read / sync-write round trips.
// Freeze f_hw from p99.9; bus timeout from the same loss/RTT distribution.
//
// Build (Linux, arm plugged in):
//   g++ -O2 -std=c++17 -I ros2_ws/src/arm_control/include \
//     ros2_ws/src/arm_control/src/feetech_bus.cpp \
//     ros2_ws/src/arm_control/src/bus_timing.cpp -o bus_timing
// Run:
//   ./bus_timing --port /dev/ttyACM0 --loops 2000 --csv bus_rtt.csv
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
#include <time.h>
#include <vector>

#include "arm_control/feetech_bus.hpp"

namespace {

struct Args {
  std::string port = "/dev/ttyACM0";
  int baud = 1000000;
  int loops = 2000;
  std::string csv;
  std::vector<uint8_t> ids{1, 2, 3, 4, 5, 6};
};

void usage() {
  std::fprintf(stderr,
               "usage: bus_timing [--port PATH] [--baud N] [--loops N] "
               "[--csv path]\n");
}

bool parse_args(int argc, char** argv, Args& args) {
  for (int i = 1; i < argc; ++i) {
    const std::string opt = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", name);
        return nullptr;
      }
      return argv[++i];
    };
    if (opt == "--port") {
      const char* v = need("--port");
      if (!v) return false;
      args.port = v;
    } else if (opt == "--baud") {
      const char* v = need("--baud");
      if (!v) return false;
      args.baud = std::atoi(v);
    } else if (opt == "--loops") {
      const char* v = need("--loops");
      if (!v) return false;
      args.loops = std::atoi(v);
    } else if (opt == "--csv") {
      const char* v = need("--csv");
      if (!v) return false;
      args.csv = v;
    } else if (opt == "-h" || opt == "--help") {
      usage();
      std::exit(0);
    } else {
      std::fprintf(stderr, "unknown option: %s\n", opt.c_str());
      return false;
    }
  }
  return args.loops > 0;
}

double percentile_us(std::vector<int64_t> ns, double p) {
  if (ns.empty()) return 0.0;
  std::sort(ns.begin(), ns.end());
  const double idx = (p / 100.0) * static_cast<double>(ns.size() - 1);
  const size_t lo = static_cast<size_t>(idx);
  const size_t hi = std::min(lo + 1, ns.size() - 1);
  const double frac = idx - static_cast<double>(lo);
  return (ns[lo] * (1.0 - frac) + ns[hi] * frac) / 1000.0;
}

void print_rtt(const char* name, const std::vector<int64_t>& ns) {
  if (ns.empty()) {
    std::printf("%s: no samples\n", name);
    return;
  }
  int64_t min_ns = ns[0];
  int64_t max_ns = ns[0];
  long double sum = 0;
  for (int64_t v : ns) {
    if (v < min_ns) min_ns = v;
    if (v > max_ns) max_ns = v;
    sum += v;
  }
  std::printf(
      "%s n=%zu Min: %.1f Avg: %.1f p99: %.1f p99.9: %.1f Max: %.1f  (us)\n",
      name, ns.size(), min_ns / 1000.0,
      static_cast<double>(sum / ns.size()) / 1000.0, percentile_us(ns, 99.0),
      percentile_us(ns, 99.9), max_ns / 1000.0);
}

int64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 2;

  arm_control::FeetechBus bus;
  try {
    bus.open(args.port, args.baud);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "open failed: %s\n", e.what());
    return 1;
  }
  bus.set_rx_timeout_ns(10000000);

  for (uint8_t id : args.ids) {
    if (!bus.ping(id)) {
      std::fprintf(stderr, "ping failed for id=%u (port=%s)\n", id,
                   args.port.c_str());
      return 1;
    }
    std::printf("ping id=%u ok\n", id);
  }

  constexpr uint8_t kReadLen = 4;
  std::vector<uint8_t> state;
  std::vector<int64_t> read_ns;
  std::vector<int64_t> write_ns;
  read_ns.reserve(static_cast<size_t>(args.loops));
  write_ns.reserve(static_cast<size_t>(args.loops));

  bus.reset_stats();
  for (int i = 0; i < args.loops; ++i) {
    const int64_t t0 = now_ns();
    const bool rok = bus.sync_read(
        args.ids, arm_control::FeetechBus::kAddrPresentPosition, kReadLen,
        state);
    const int64_t t1 = now_ns();
    if (rok) read_ns.push_back(t1 - t0);
    if (!rok || state.size() < args.ids.size() * kReadLen) continue;

    std::vector<uint8_t> goal(args.ids.size() * 2);
    for (size_t j = 0; j < args.ids.size(); ++j) {
      goal[2 * j] = state[j * kReadLen];
      goal[2 * j + 1] = state[j * kReadLen + 1];
    }
    const int64_t t2 = now_ns();
    const bool wok = bus.sync_write(
        args.ids, arm_control::FeetechBus::kAddrGoalPosition, goal);
    const int64_t t3 = now_ns();
    if (wok) write_ns.push_back(t3 - t2);
  }

  const auto& st = bus.stats();
  const uint64_t attempts = static_cast<uint64_t>(args.loops) * args.ids.size();
  std::printf("checksum_errors=%llu timeouts=%llu rx=%llu tx=%llu\n",
              static_cast<unsigned long long>(st.checksum_errors),
              static_cast<unsigned long long>(st.timeouts),
              static_cast<unsigned long long>(st.rx_packets),
              static_cast<unsigned long long>(st.tx_packets));
  if (attempts > 0) {
    std::printf("packet_loss_frac=%.6f  (timeouts+checksum)/loops\n",
                static_cast<double>(st.timeouts + st.checksum_errors) /
                    static_cast<double>(args.loops));
  }
  print_rtt("sync_read", read_ns);
  print_rtt("sync_write", write_ns);

  if (!read_ns.empty()) {
    const double p999 = percentile_us(read_ns, 99.9);
    const double f_hw = 0.8 * 1e6 / p999;
    std::printf("suggested f_hw <= %.1f Hz  (0.8 / p99.9_read)\n", f_hw);
    std::printf("suggested bus_timeout_ns = %.0f  (3 * p99.9_read)\n",
                3.0 * p999 * 1000.0);
  }

  if (!args.csv.empty()) {
    std::ofstream out(args.csv);
    out << "i,op,rtt_us\n";
    out.precision(9);
    for (size_t i = 0; i < read_ns.size(); ++i)
      out << i << ",read," << read_ns[i] / 1000.0 << '\n';
    for (size_t i = 0; i < write_ns.size(); ++i)
      out << i << ",write," << write_ns[i] / 1000.0 << '\n';
    std::printf("wrote %s\n", args.csv.c_str());
  }
  return 0;
}
