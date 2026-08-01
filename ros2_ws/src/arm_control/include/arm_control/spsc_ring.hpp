// Single-producer / single-consumer lock-free ring buffer. The RT control
// thread is the sole producer; the non-RT ROS 2 publisher is the sole consumer.
// The producer never blocks or allocates -- push() drops the sample if the
// consumer has fallen behind (telemetry is best-effort; control is not).
//
// Correctness rests on: exactly one producer thread, exactly one consumer
// thread, capacity a power of two, and acquire/release ordering on head/tail.
#pragma once
#include <atomic>
#include <cstddef>
#include <vector>

namespace arm_control {

template <typename T>
class SpscRing {
public:
  explicit SpscRing(size_t capacity_pow2) : mask_(capacity_pow2 - 1),
                                            buf_(capacity_pow2) {
    // capacity must be a power of two so index wrap is a cheap mask.
    // (head_/tail_ are free-running counters; slot = counter & mask_.)
  }

  // Producer side (RT thread). Returns false and drops if the ring is full.
  bool push(const T& item) {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail > mask_) {
      return false;  // full: drop, never block the RT thread
    }
    buf_[head & mask_] = item;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Consumer side (non-RT thread). Returns false if empty.
  bool pop(T& out) {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    const size_t head = head_.load(std::memory_order_acquire);
    if (tail == head) {
      return false;  // empty
    }
    out = buf_[tail & mask_];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

private:
  const size_t mask_;
  std::vector<T> buf_;
  std::atomic<size_t> head_{0};  // written by producer
  std::atomic<size_t> tail_{0};  // written by consumer
};

}  // namespace arm_control
