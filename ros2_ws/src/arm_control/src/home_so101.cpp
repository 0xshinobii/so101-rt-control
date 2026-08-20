// Move the SO-101 follower from the current pose to calibrated zero with a
// minimum-jerk interpolation (zero vel/acc at start and end — no step).
// Enables torque at the *current* position first so the arm does not jump.
//
//   g++ -O2 -std=c++17 -I ros2_ws/src/arm_control/include \
//     ros2_ws/src/arm_control/src/feetech_bus.cpp \
//     ros2_ws/src/arm_control/src/home_so101.cpp -o home_so101
//   ./home_so101 --port /dev/ttyACM0 --calib so101_follower_calib.json
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "arm_control/feetech_bus.hpp"

namespace {

const std::array<const char*, 6> kNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper"};
const std::vector<uint8_t> kIds = {1, 2, 3, 4, 5, 6};
constexpr uint8_t kAddrAcceleration = 41;
constexpr uint8_t kAddrGoalSpeed = 46;
constexpr int kRateHz = 50;

struct JointCalib {
  int zero = 0;
  int min_ticks = 0;
  int max_ticks = 4095;
};

int json_int(const std::string& block, const char* key) {
  const std::string pat = std::string("\"") + key + "\":";
  const auto pos = block.find(pat);
  if (pos == std::string::npos) {
    throw std::runtime_error(std::string("missing ") + key);
  }
  return std::atoi(block.c_str() + pos + pat.size());
}

std::array<JointCalib, 6> load_calib(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::stringstream buf;
  buf << in.rdbuf();
  const std::string text = buf.str();
  std::array<JointCalib, 6> out;
  for (int i = 0; i < 6; ++i) {
    const std::string key = std::string("\"") + kNames[i] + "\"";
    const auto start = text.find(key);
    if (start == std::string::npos) {
      throw std::runtime_error(std::string("missing joint ") + kNames[i]);
    }
    const auto brace = text.find('{', start);
    const auto end = text.find('}', brace);
    const std::string block = text.substr(brace, end - brace + 1);
    out[i].zero = json_int(block, "zero_ticks");
    out[i].min_ticks = json_int(block, "min_ticks");
    out[i].max_ticks = json_int(block, "max_ticks");
  }
  return out;
}

bool read_positions(arm_control::FeetechBus& bus, std::array<int, 6>& ticks) {
  std::vector<uint8_t> raw;
  if (!bus.sync_read(kIds, arm_control::FeetechBus::kAddrPresentPosition, 2,
                     raw) ||
      raw.size() < 12) {
    return false;
  }
  for (int i = 0; i < 6; ++i) {
    ticks[i] = static_cast<int>(raw[2 * i] | (raw[2 * i + 1] << 8));
  }
  return true;
}

bool write_positions(arm_control::FeetechBus& bus,
                     const std::array<int, 6>& ticks) {
  std::vector<uint8_t> data(12);
  for (int i = 0; i < 6; ++i) {
    const int t = std::clamp(ticks[i], 0, 4095);
    data[2 * i] = static_cast<uint8_t>(t & 0xFF);
    data[2 * i + 1] = static_cast<uint8_t>((t >> 8) & 0xFF);
  }
  return bus.sync_write(kIds, arm_control::FeetechBus::kAddrGoalPosition, data);
}

double min_jerk(double s) {
  s = std::clamp(s, 0.0, 1.0);
  const double s3 = s * s * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  return 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
}

}  // namespace

int main(int argc, char** argv) {
  std::string port = "/dev/ttyACM0";
  std::string calib_path = "so101_follower_calib.json";
  double duration = 4.0;
  for (int i = 1; i < argc; ++i) {
    const std::string opt = argv[i];
    if (opt == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (opt == "--calib" && i + 1 < argc) {
      calib_path = argv[++i];
    } else if (opt == "--duration" && i + 1 < argc) {
      duration = std::atof(argv[++i]);
    } else {
      std::fprintf(stderr,
                   "usage: home_so101 [--port PATH] [--calib FILE] "
                   "[--duration SEC]\n");
      return 2;
    }
  }
  if (!(duration >= 1.0 && duration <= 30.0)) {
    std::fprintf(stderr, "duration must be in [1, 30] s\n");
    return 2;
  }

  const auto calib = load_calib(calib_path);
  arm_control::FeetechBus bus;
  bus.open(port, 1000000);
  bus.set_rx_timeout_ns(5130000);

  std::array<int, 6> start{};
  if (!read_positions(bus, start)) {
    std::fprintf(stderr, "failed to read current pose\n");
    return 1;
  }

  // Hold here before enabling torque so the last Goal_Position is not a jump.
  if (!write_positions(bus, start)) {
    std::fprintf(stderr, "failed to set goal = current\n");
    return 1;
  }
  const uint8_t acc = 30;
  const uint8_t speed[2] = {0, 0};  // 0 = no extra speed cap; profile is ours
  for (uint8_t id : kIds) {
    bus.write(id, kAddrAcceleration, &acc, 1);
    bus.write(id, kAddrGoalSpeed, speed, 2);
    const uint8_t on = 1;
    if (!bus.write(id, arm_control::FeetechBus::kAddrTorqueEnable, &on, 1)) {
      std::fprintf(stderr, "torque on failed id=%u\n", id);
      return 1;
    }
  }

  int max_delta = 0;
  for (int i = 0; i < 6; ++i) {
    max_delta = std::max(max_delta, std::abs(calib[i].zero - start[i]));
    std::printf("%s  now=%d  zero=%d  delta=%d\n", kNames[i], start[i],
                calib[i].zero, calib[i].zero - start[i]);
  }
  const double t_move =
      std::max(duration, static_cast<double>(max_delta) / 250.0);
  std::printf("min-jerk home over %.2f s at %d Hz\n", t_move, kRateHz);

  const int steps = static_cast<int>(t_move * kRateHz);
  const auto period = std::chrono::milliseconds(1000 / kRateHz);
  auto next = std::chrono::steady_clock::now();
  for (int k = 0; k <= steps; ++k) {
    const double s = min_jerk(static_cast<double>(k) / steps);
    std::array<int, 6> cmd{};
    for (int i = 0; i < 6; ++i) {
      const double v = start[i] + s * (calib[i].zero - start[i]);
      int tick = static_cast<int>(std::lround(v));
      tick = std::clamp(tick, calib[i].min_ticks, calib[i].max_ticks);
      cmd[i] = tick;
    }
    if (!write_positions(bus, cmd)) {
      std::fprintf(stderr, "sync_write failed at step %d\n", k);
      return 1;
    }
    next += period;
    std::this_thread::sleep_until(next);
  }
  std::printf("holding calibrated zero (torque on)\n");
  return 0;
}
