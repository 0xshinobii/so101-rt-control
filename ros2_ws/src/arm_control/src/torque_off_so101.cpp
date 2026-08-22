// Disable torque on all six follower STS3215s. Arm goes limp — hold it.
//
//   g++ -O2 -std=c++17 -I ros2_ws/src/arm_control/include \
//     ros2_ws/src/arm_control/src/feetech_bus.cpp \
//     ros2_ws/src/arm_control/src/torque_off_so101.cpp -o torque_off_so101
//   ./torque_off_so101 --port /dev/ttyACM0
#include <cstdio>
#include <string>
#include <vector>

#include "arm_control/feetech_bus.hpp"

int main(int argc, char** argv) {
  std::string port = "/dev/ttyACM0";
  for (int i = 1; i < argc; ++i) {
    const std::string opt = argv[i];
    if (opt == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else {
      std::fprintf(stderr, "usage: torque_off_so101 [--port PATH]\n");
      return 2;
    }
  }

  const std::vector<uint8_t> ids = {1, 2, 3, 4, 5, 6};
  arm_control::FeetechBus bus;
  bus.open(port, 1000000);
  bus.set_rx_timeout_ns(5130000);

  const uint8_t off = 0;
  int failed = 0;
  for (uint8_t id : ids) {
    if (!bus.write(id, arm_control::FeetechBus::kAddrTorqueEnable, &off, 1)) {
      std::fprintf(stderr, "torque off failed id=%u\n", id);
      ++failed;
    }
  }
  if (failed) return 1;
  std::printf("torque off  ids 1-6  port=%s  hold the arm\n", port.c_str());
  return 0;
}
