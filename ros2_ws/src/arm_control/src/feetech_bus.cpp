#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "arm_control/feetech_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace arm_control {
namespace {

int64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

speed_t baud_flag(int baud) {
  switch (baud) {
    case 115200:
      return B115200;
    case 1000000:
      return B1000000;
    default:
      throw std::invalid_argument("unsupported baud");
  }
}

}  // namespace

FeetechBus::~FeetechBus() { close(); }

void FeetechBus::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void FeetechBus::open(const std::string& device, int baud) {
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error(std::string("open ") + device + ": " +
                             std::strerror(errno));
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    close();
    throw std::runtime_error("tcgetattr failed");
  }
  cfmakeraw(&tio);
  const speed_t spd = baud_flag(baud);
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    close();
    throw std::runtime_error("tcsetattr failed");
  }
  tcflush(fd_, TCIOFLUSH);
}

uint8_t FeetechBus::checksum(const uint8_t* p, size_t n) {
  unsigned sum = 0;
  for (size_t i = 0; i < n; ++i) sum += p[i];
  return static_cast<uint8_t>(~sum);
}

bool FeetechBus::wait_ready(short events, int64_t deadline_ns) {
  while (true) {
    const int64_t remaining_ns = deadline_ns - now_ns();
    if (remaining_ns <= 0) {
      ++stats_.timeouts;
      return false;
    }
    pollfd pfd{fd_, events, 0};
    timespec ts{remaining_ns / 1000000000LL, remaining_ns % 1000000000LL};
    const int ready = ::ppoll(&pfd, 1, &ts, nullptr);
    if (ready < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(std::string("serial ppoll: ") +
                               std::strerror(errno));
    }
    if (ready == 0) {
      ++stats_.timeouts;
      return false;
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return false;
    if (pfd.revents & events) return true;
  }
}

bool FeetechBus::tx(const uint8_t* data, size_t n, int64_t deadline_ns) {
  size_t off = 0;
  while (off < n) {
    if (now_ns() >= deadline_ns) {
      ++stats_.timeouts;
      return false;
    }
    const ssize_t w = ::write(fd_, data + off, n - off);
    if (w < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN) {
        if (!wait_ready(POLLOUT, deadline_ns)) return false;
        continue;
      }
      throw std::runtime_error(std::string("serial write: ") +
                               std::strerror(errno));
    }
    if (w == 0) {
      if (!wait_ready(POLLOUT, deadline_ns)) return false;
      continue;
    }
    off += static_cast<size_t>(w);
  }
  if (!drain_until(deadline_ns)) return false;
  ++stats_.tx_packets;
  return true;
}

// TIOCOUTQ == 0 means the kernel TTY queue is empty: bytes were handed to
// the USB stack, not that the UART clocked them onto the wire.
bool FeetechBus::drain_until(int64_t deadline_ns) {
  while (true) {
    const int64_t now = now_ns();
    if (now >= deadline_ns) {
      ++stats_.timeouts;
      return false;
    }
    int queued = 0;
    if (ioctl(fd_, TIOCOUTQ, &queued) != 0) {
      throw std::runtime_error(std::string("serial TIOCOUTQ: ") +
                               std::strerror(errno));
    }
    if (queued == 0) return true;

    // int64_t is long on x86_64, so an LL literal breaks min()'s deduction.
    const int64_t wake = std::min<int64_t>(deadline_ns, now + 50000);
    timespec ts{wake / 1000000000LL, wake % 1000000000LL};
    int rc = 0;
    do {
      rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    } while (rc == EINTR);
    if (rc != 0) {
      ++stats_.timeouts;
      return false;
    }
  }
}

bool FeetechBus::rx_byte(uint8_t& b, int64_t deadline_ns) {
  while (true) {
    if (!wait_ready(POLLIN, deadline_ns)) return false;
    if (now_ns() >= deadline_ns) {
      ++stats_.timeouts;
      return false;
    }
    const ssize_t n = ::read(fd_, &b, 1);
    if (n == 1) return true;
    if (n == 0) return false;
    if (n < 0 && errno != EAGAIN && errno != EINTR) return false;
  }
}

bool FeetechBus::rx_status(uint8_t expected_id, uint8_t* data,
                           uint8_t data_len, int64_t deadline_ns) {
  uint8_t b = 0;
  int ff = 0;
  while (ff < 2) {
    if (!rx_byte(b, deadline_ns)) return false;
    if (b == 0xFF)
      ++ff;
    else
      ff = 0;
  }
  uint8_t hdr[3];
  for (int i = 0; i < 3; ++i) {
    if (!rx_byte(hdr[i], deadline_ns)) return false;
  }
  const uint8_t id = hdr[0];
  const uint8_t length = hdr[1];
  const uint8_t error = hdr[2];
  if (length < 2) {
    ++stats_.checksum_errors;
    return false;
  }
  const uint8_t payload_len = static_cast<uint8_t>(length - 2);
  unsigned sum = id + length + error;
  for (uint8_t i = 0; i < payload_len; ++i) {
    uint8_t value = 0;
    if (!rx_byte(value, deadline_ns)) return false;
    sum += value;
    if (data != nullptr && i < data_len) data[i] = value;
  }
  uint8_t csum = 0;
  if (!rx_byte(csum, deadline_ns)) return false;
  const uint8_t expect = static_cast<uint8_t>(~sum);
  if (csum != expect || id != expected_id) {
    ++stats_.checksum_errors;
    return false;
  }
  ++stats_.rx_packets;
  return true;
}

bool FeetechBus::ping(uint8_t id) {
  uint8_t pkt[6] = {0xFF, 0xFF, id, 2, kInstPing, 0};
  pkt[5] = checksum(pkt + 2, 3);
  tcflush(fd_, TCIFLUSH);
  const int64_t deadline = now_ns() + rx_timeout_ns_;
  return tx(pkt, sizeof(pkt), deadline) &&
         rx_status(id, nullptr, 0, deadline);
}

bool FeetechBus::read(uint8_t id, uint8_t addr, uint8_t length, uint8_t* out) {
  uint8_t pkt[8] = {0xFF, 0xFF, id, 4, kInstRead, addr, length, 0};
  pkt[7] = checksum(pkt + 2, 5);
  tcflush(fd_, TCIFLUSH);
  const int64_t deadline = now_ns() + rx_timeout_ns_;
  return tx(pkt, sizeof(pkt), deadline) &&
         rx_status(id, out, length, deadline);
}

bool FeetechBus::write(uint8_t id, uint8_t addr, const uint8_t* data,
                      uint8_t length) {
  std::vector<uint8_t>& pkt = pkt_;
  pkt.assign(7 + length, 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xFF;
  pkt[2] = id;
  pkt[3] = static_cast<uint8_t>(3 + length);
  pkt[4] = kInstWrite;
  pkt[5] = addr;
  for (uint8_t i = 0; i < length; ++i) pkt[6 + i] = data[i];
  pkt.back() = checksum(pkt.data() + 2, 4 + length);
  tcflush(fd_, TCIFLUSH);
  const int64_t deadline = now_ns() + rx_timeout_ns_;
  return tx(pkt.data(), pkt.size(), deadline) &&
         rx_status(id, nullptr, 0, deadline);
}

bool FeetechBus::sync_read(const std::vector<uint8_t>& ids, uint8_t addr,
                           uint8_t length, std::vector<uint8_t>& out) {
  out.assign(ids.size() * length, 0);
  std::vector<uint8_t>& pkt = pkt_;
  pkt.assign(8 + ids.size(), 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xFF;
  pkt[2] = kBroadcastId;
  pkt[3] = static_cast<uint8_t>(4 + ids.size());
  pkt[4] = kInstSyncRead;
  pkt[5] = addr;
  pkt[6] = length;
  for (size_t i = 0; i < ids.size(); ++i) pkt[7 + i] = ids[i];
  pkt.back() = checksum(pkt.data() + 2, 5 + ids.size());
  tcflush(fd_, TCIFLUSH);
  const int64_t deadline = now_ns() + rx_timeout_ns_;
  if (!tx(pkt.data(), pkt.size(), deadline)) return false;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (!rx_status(ids[i], out.data() + i * length, length, deadline)) {
      return false;
    }
  }
  return true;
}

bool FeetechBus::sync_write(const std::vector<uint8_t>& ids, uint8_t addr,
                            const std::vector<uint8_t>& data) {
  if (ids.empty()) return false;
  const size_t per = data.size() / ids.size();
  if (per == 0 || data.size() != per * ids.size()) {
    throw std::invalid_argument("sync_write data size mismatch");
  }
  std::vector<uint8_t>& pkt = pkt_;
  pkt.assign(8 + ids.size() * (1 + per), 0);
  pkt[0] = 0xFF;
  pkt[1] = 0xFF;
  pkt[2] = kBroadcastId;
  pkt[3] = static_cast<uint8_t>(4 + ids.size() * (1 + per));
  pkt[4] = kInstSyncWrite;
  pkt[5] = addr;
  pkt[6] = static_cast<uint8_t>(per);
  size_t o = 7;
  for (size_t i = 0; i < ids.size(); ++i) {
    pkt[o++] = ids[i];
    for (size_t j = 0; j < per; ++j) pkt[o++] = data[i * per + j];
  }
  pkt.back() = checksum(pkt.data() + 2, pkt.size() - 3);
  tcflush(fd_, TCIFLUSH);
  const int64_t deadline = now_ns() + tx_timeout_ns_;
  return tx(pkt.data(), pkt.size(), deadline);
}

}  // namespace arm_control
