// SO-101 follower calibration without LeRobot. Disables torque, records
// mid-range as zero ticks, then samples min/max while you backdrive joints.
// Writes JSON for the hardware backend (q=0 == this mid-range pose).
//
//   g++ -O2 -std=c++17 -I ros2_ws/src/arm_control/include \
//     ros2_ws/src/arm_control/src/feetech_bus.cpp \
//     ros2_ws/src/arm_control/src/calibrate_so101.cpp -o calibrate_so101
//   ./calibrate_so101 --port /dev/ttyACM0 --out so101_follower_calib.json
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/select.h>
#include <unistd.h>
#include <vector>

#include "arm_control/feetech_bus.hpp"

namespace {

const std::array<const char*, 6> kNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper"};
const std::vector<uint8_t> kIds = {1, 2, 3, 4, 5, 6};

void wait_enter(const char* prompt) {
  std::printf("%s", prompt);
  std::fflush(stdout);
  int c;
  while ((c = std::getchar()) != '\n' && c != EOF) {
  }
}

bool stdin_ready() {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  timeval tv{0, 0};
  return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0;
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

}  // namespace

int main(int argc, char** argv) {
  std::string port = "/dev/ttyACM0";
  std::string out_path = "so101_follower_calib.json";
  for (int i = 1; i < argc; ++i) {
    const std::string opt = argv[i];
    if (opt == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (opt == "--out" && i + 1 < argc) {
      out_path = argv[++i];
    } else {
      std::fprintf(stderr, "usage: calibrate_so101 [--port PATH] [--out FILE]\n");
      return 2;
    }
  }

  arm_control::FeetechBus bus;
  bus.open(port, 1000000);
  bus.set_rx_timeout_ns(5130000);
  for (uint8_t id : kIds) {
    if (!bus.ping(id)) {
      std::fprintf(stderr, "ping failed id=%u\n", id);
      return 1;
    }
  }

  const uint8_t off = 0;
  for (uint8_t id : kIds) {
    if (!bus.write(id, arm_control::FeetechBus::kAddrTorqueEnable, &off, 1)) {
      std::fprintf(stderr, "torque off failed id=%u\n", id);
      return 1;
    }
  }
  std::printf("torque disabled. arm should be backdrivable.\n");

  wait_enter(
      "Move EVERY joint to mid-range (LeRobot home). Gripper half-open.\n"
      "Press Enter to capture zero...\n");

  std::array<int, 6> zero{};
  if (!read_positions(bus, zero)) {
    std::fprintf(stderr, "failed to read mid-range pose\n");
    return 1;
  }
  for (int i = 0; i < 6; ++i) {
    std::printf("  %s zero_ticks=%d\n", kNames[i], zero[i]);
  }

  std::array<int, 6> tmin = zero;
  std::array<int, 6> tmax = zero;
  wait_enter(
      "Now move each joint through its FULL mechanical range.\n"
      "Sampling until you press Enter...\n");

  std::array<int, 6> ticks{};
  while (!stdin_ready()) {
    if (!read_positions(bus, ticks)) continue;
    for (int i = 0; i < 6; ++i) {
      if (ticks[i] < tmin[i]) tmin[i] = ticks[i];
      if (ticks[i] > tmax[i]) tmax[i] = ticks[i];
    }
    std::printf("\r");
    for (int i = 0; i < 6; ++i) {
      std::printf("%s %4d [%4d,%4d]  ", kNames[i], ticks[i], tmin[i], tmax[i]);
    }
    std::fflush(stdout);
  }
  wait_enter("");
  std::printf("\n");

  std::ofstream out(out_path);
  if (!out) {
    std::fprintf(stderr, "cannot write %s\n", out_path.c_str());
    return 1;
  }
  out << "{\n";
  for (int i = 0; i < 6; ++i) {
    out << "  \"" << kNames[i] << "\": {"
        << "\"id\": " << static_cast<int>(kIds[i]) << ", "
        << "\"zero_ticks\": " << zero[i] << ", "
        << "\"min_ticks\": " << tmin[i] << ", "
        << "\"max_ticks\": " << tmax[i] << ", "
        << "\"sign\": 1}";
    if (i < 5) out << ",";
    out << "\n";
  }
  out << "}\n";
  std::printf("wrote %s\n", out_path.c_str());
  return 0;
}
