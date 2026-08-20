// POSIX RT setup for the control thread: mlockall, SCHED_FIFO, optional CPU
// pin, and /dev/cpu_dma_latency. Call from the thread that will run the loop
// (FIFO applies to the caller). Setup may fail without CAP_SYS_NICE / root;
// callers should log RtStatus and keep running rather than abort.
//
// Translation units that include this header must define _GNU_SOURCE before
// any system headers (needed for pthread_setaffinity_np).
#pragma once

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <sys/mman.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>

namespace arm_control {

struct RtConfig {
  int fifo_priority = 80;  // 1..99; 80 matches the cyclictest baseline
  int cpu_affinity = -1;   // -1 = do not pin
  bool lock_memory = true;
  bool suppress_cstates = true;
};

struct RtStatus {
  bool memory_locked = false;
  bool fifo_set = false;
  bool affinity_set = false;
  bool cstates_suppressed = false;
  std::string error;
};

inline void append_error(RtStatus& st, const char* what) {
  if (!st.error.empty()) st.error += "; ";
  st.error += what;
  st.error += ": ";
  st.error += std::strerror(errno);
}

inline RtStatus configure_rt_thread(const RtConfig& cfg) {
  RtStatus st;

  if (cfg.lock_memory) {
    struct rlimit lim;
    lim.rlim_cur = RLIM_INFINITY;
    lim.rlim_max = RLIM_INFINITY;
    if (setrlimit(RLIMIT_MEMLOCK, &lim) != 0) {
      append_error(st, "setrlimit(RLIMIT_MEMLOCK)");
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      append_error(st, "mlockall");
    } else {
      st.memory_locked = true;
    }
  }

  if (cfg.fifo_priority > 0) {
    struct sched_param sp;
    sp.sched_priority = cfg.fifo_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      append_error(st, "pthread_setschedparam(SCHED_FIFO)");
    } else {
      st.fifo_set = true;
    }
  }

  if (cfg.cpu_affinity >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cfg.cpu_affinity, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
      append_error(st, "pthread_setaffinity_np");
    } else {
      st.affinity_set = true;
    }
  }

  if (cfg.suppress_cstates) {
    // Keep the fd open for the process lifetime; closing it restores C-states.
    static int dma_fd = -1;
    if (dma_fd < 0) {
      dma_fd = open("/dev/cpu_dma_latency", O_RDWR | O_CLOEXEC);
      if (dma_fd < 0) {
        append_error(st, "open(/dev/cpu_dma_latency)");
      } else {
        int32_t target = 0;
        if (write(dma_fd, &target, sizeof(target)) != sizeof(target)) {
          append_error(st, "write(/dev/cpu_dma_latency)");
          close(dma_fd);
          dma_fd = -1;
        } else {
          st.cstates_suppressed = true;
        }
      }
    } else {
      st.cstates_suppressed = true;
    }
  }

  return st;
}

// Absolute wait on CLOCK_MONOTONIC (same clock as std::steady_clock on
// Linux). Retries EINTR. Relative nanosleep / sleep_until can pick up timer
// slack and inflate Max vs cyclictest.
inline void sleep_until_monotonic(
    std::chrono::steady_clock::time_point deadline) {
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      deadline.time_since_epoch())
                      .count();
  timespec ts;
  ts.tv_sec = static_cast<time_t>(ns / 1000000000);
  ts.tv_nsec = static_cast<long>(ns % 1000000000);
  while (true) {
    const int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    if (rc == 0 || rc != EINTR) return;
  }
}

}  // namespace arm_control
