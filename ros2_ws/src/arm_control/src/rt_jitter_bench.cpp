// Wall-clock wakeup-jitter bench for Phase 3. No MuJoCo, no ROS: the loop is
// sleep + timestamp, matching cyclictest's quantity, with this stack's RT
// setup (mlockall, SCHED_FIFO, optional CPU pin).
//
// Build (no ROS needed):
//   g++ -O2 -std=c++17 -I ros2_ws/src/arm_control/include \
//     ros2_ws/src/arm_control/src/rt_jitter_bench.cpp -o rt_jitter_bench -lpthread
// Run:
//   sudo ./rt_jitter_bench --period-us 1000 --loops 60000 --csv jitter.csv
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "arm_control/rt_thread.hpp"

namespace {

struct Args {
  int period_us = 1000;
  int loops = 20000;
  int priority = 80;
  int cpu = -1;
  std::string csv;
};

void usage() {
  std::fprintf(stderr,
               "usage: rt_jitter_bench [--period-us N] [--loops N] "
               "[--priority N] [--cpu N] [--csv path]\n");
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
    if (opt == "--period-us") {
      const char* v = need("--period-us");
      if (!v) return false;
      args.period_us = std::atoi(v);
    } else if (opt == "--loops") {
      const char* v = need("--loops");
      if (!v) return false;
      args.loops = std::atoi(v);
    } else if (opt == "--priority") {
      const char* v = need("--priority");
      if (!v) return false;
      args.priority = std::atoi(v);
    } else if (opt == "--cpu") {
      const char* v = need("--cpu");
      if (!v) return false;
      args.cpu = std::atoi(v);
    } else if (opt == "--csv") {
      const char* v = need("--csv");
      if (!v) return false;
      args.csv = v;
    } else if (opt == "-h" || opt == "--help") {
      usage();
      std::exit(0);
    } else {
      std::fprintf(stderr, "unknown option: %s\n", opt.c_str());
      usage();
      return false;
    }
  }
  if (args.period_us <= 0 || args.loops <= 0 || args.priority < 1 ||
      args.priority > 99) {
    std::fprintf(stderr, "invalid period/loops/priority\n");
    return false;
  }
  return true;
}

double percentile_us(std::vector<int64_t> ns, double p) {
  if (ns.empty()) return 0.0;
  std::sort(ns.begin(), ns.end());
  const double idx = (p / 100.0) * static_cast<double>(ns.size() - 1);
  const size_t lo = static_cast<size_t>(idx);
  const size_t hi = std::min(lo + 1, ns.size() - 1);
  const double frac = idx - static_cast<double>(lo);
  const double v =
      static_cast<double>(ns[lo]) * (1.0 - frac) +
      static_cast<double>(ns[hi]) * frac;
  return v / 1000.0;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 2;

  arm_control::RtConfig cfg;
  cfg.fifo_priority = args.priority;
  cfg.cpu_affinity = args.cpu;
  const arm_control::RtStatus rt = arm_control::configure_rt_thread(cfg);
  std::printf("rt: mlockall=%d fifo=%d affinity=%d cstates=%d\n",
              rt.memory_locked, rt.fifo_set, rt.affinity_set,
              rt.cstates_suppressed);
  if (!rt.error.empty()) {
    std::fprintf(stderr, "rt warnings: %s\n", rt.error.c_str());
  }
  if (!rt.fifo_set) {
    std::fprintf(stderr, "error: SCHED_FIFO not set; re-run with sudo\n");
    return 1;
  }

  std::vector<int64_t> late_ns(static_cast<size_t>(args.loops));
  const auto period = std::chrono::microseconds(args.period_us);
  auto next = std::chrono::steady_clock::now() + period;

  for (int i = 0; i < args.loops; ++i) {
    arm_control::sleep_until_monotonic(next);
    const auto now = std::chrono::steady_clock::now();
    late_ns[static_cast<size_t>(i)] =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - next)
            .count();
    next += period;
  }

  int64_t min_ns = late_ns[0];
  int64_t max_ns = late_ns[0];
  long double sum = 0;
  for (int64_t v : late_ns) {
    if (v < min_ns) min_ns = v;
    if (v > max_ns) max_ns = v;
    sum += static_cast<long double>(v);
  }
  const double avg_us =
      static_cast<double>(sum / late_ns.size()) / 1000.0;
  std::printf(
      "period_us=%d loops=%d Min: %.3f Avg: %.3f p99: %.3f p99.9: %.3f "
      "Max: %.3f  (us)\n",
      args.period_us, args.loops, min_ns / 1000.0, avg_us,
      percentile_us(late_ns, 99.0), percentile_us(late_ns, 99.9),
      max_ns / 1000.0);

  if (!args.csv.empty()) {
    std::ofstream out(args.csv);
    if (!out) {
      std::fprintf(stderr, "failed to write %s\n", args.csv.c_str());
      return 1;
    }
    out << "# period_us=" << args.period_us << "\n";
    out << "# loops=" << args.loops << "\n";
    out << "i,late_us\n";
    out.precision(9);
    for (size_t i = 0; i < late_ns.size(); ++i) {
      out << i << ',' << late_ns[i] / 1000.0 << '\n';
    }
    std::printf("wrote %s\n", args.csv.c_str());
  }
  return 0;
}
