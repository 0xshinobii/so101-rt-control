// K_servo ≈ g(q) / (q_des - q) from a settled hardware CSV.
// Skip joints with |offset| < 0.01 rad (noise / backlash).
//
//   g++ -O2 -std=c++17 -I ros2_ws/src/arm_control/include \
//     $(pkg-config --cflags eigen3) \
//     tools/kservo_from_csv.cpp \
//     ros2_ws/src/arm_control/src/pinocchio_dynamics.cpp \
//     -o kservo_from_csv $(pkg-config --libs pinocchio) -lpthread
//   ./kservo_from_csv hw_pd.csv
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

using arm_control::kDof;
using arm_control::PinocchioDynamics;

namespace {
const Eigen::VectorXd kTarget =
    (Eigen::VectorXd(kDof) << 0.6, 0.7, -0.8, 0.5, 0.4, 0.0).finished();
const char* kNames[kDof] = {"pan", "lift", "elbow", "wrist_flex",
                            "wrist_roll", "gripper"};
constexpr double kMinAbsOffset = 0.01;
}  // namespace

int main(int argc, char** argv) {
  const std::string csv = (argc > 1) ? argv[1] : "hw_pd.csv";
  const std::string urdf =
      (argc > 2) ? argv[2] : "models/so101/so101_dynamics.urdf";
  std::ifstream in(csv);
  if (!in) throw std::runtime_error("cannot open " + csv);
  std::string line, last;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("t,", 0) == 0) continue;
    last = line;
  }
  if (last.empty()) throw std::runtime_error("no samples");
  std::stringstream ss(last);
  std::string cell;
  std::getline(ss, cell, ',');  // t
  Eigen::VectorXd q(kDof);
  for (int i = 0; i < kDof; ++i) {
    std::getline(ss, cell, ',');
    q[i] = std::stod(cell);
  }

  PinocchioDynamics dyn(urdf);
  Eigen::VectorXd g(kDof);
  dyn.gravity(q, g);
  const Eigen::Vector3d ee = dyn.ee_position(q);
  std::printf("q    =");
  for (int i = 0; i < kDof; ++i) std::printf(" % .4f", q[i]);
  std::printf("\ng(q) =");
  for (int i = 0; i < kDof; ++i) std::printf(" % .4f", g[i]);
  std::printf(" N.m\nee   = %.4f %.4f %.4f m\n", ee.x(), ee.y(), ee.z());
  std::printf("\n%-12s offset     g [N.m]   K_servo\n", "joint");
  for (int i = 0; i < 5; ++i) {
    const double off = kTarget[i] - q[i];
    std::printf("%-12s %+8.4f  %+8.4f   ", kNames[i], off, g[i]);
    if (std::abs(off) < kMinAbsOffset) {
      std::printf("(skip |offset|<%.2f)\n", kMinAbsOffset);
    } else {
      std::printf("%.1f\n", g[i] / off);
    }
  }
  return 0;
}
