// Real-arm runner. Streams the same min-jerk q_des that homing uses
// (STS3215 position mode). Joint tracking is the metric; EE columns are 0.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/hardware_backend.hpp"

using arm_control::HardwareBackend;
using arm_control::kDof;
using arm_control::Sample;

namespace {

const Eigen::VectorXd kTarget =
    (Eigen::VectorXd(kDof) << 0.6, 0.7, -0.8, 0.5, 0.4, 0.0).finished();
constexpr double kDuration = 4.0;
constexpr double kReferenceDuration = 1.0;

void minimum_jerk_reference(double time, Eigen::VectorXd& q,
                            Eigen::VectorXd& qdot, Eigen::VectorXd& qddot) {
  const double s = std::clamp(time / kReferenceDuration, 0.0, 1.0);
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  q = (10.0 * s3 - 15.0 * s4 + 6.0 * s5) * kTarget;
  qdot = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) * kTarget /
         kReferenceDuration;
  qddot = (60.0 * s - 180.0 * s2 + 120.0 * s3) * kTarget /
          (kReferenceDuration * kReferenceDuration);
}

}  // namespace

int main(int argc, char** argv) {
  HardwareBackend::Config cfg;
  cfg.calib_path = "so101_follower_calib.json";
  std::string csv_path = "hw_pd_so101.csv";
  for (int i = 1; i < argc; ++i) {
    const std::string opt = argv[i];
    auto need = [&]() -> const char* {
      if (i + 1 >= argc) throw std::invalid_argument("missing value");
      return argv[++i];
    };
    if (opt == "--port") {
      cfg.port = need();
    } else if (opt == "--calib") {
      cfg.calib_path = need();
    } else if (opt == "--csv") {
      csv_path = need();
    } else if (opt == "--k-servo") {
      cfg.k_servo = std::stod(need());
    } else {
      std::fprintf(stderr,
                   "usage: hardware_run [--port PATH] [--calib FILE] "
                   "[--k-servo N] [--csv FILE]\n");
      return 2;
    }
  }

  try {
    HardwareBackend plant(cfg);
    std::printf(
        "hardware stream  port=%s dt=%.4f\n"
        "clear workspace — homes, then min-jerk to kTarget\n",
        cfg.port.c_str(), plant.timestep());
    plant.reset();

    const double dt = plant.timestep();
    const int steps = static_cast<int>(kDuration / dt);
    std::vector<Sample> log(steps);
    Eigen::VectorXd q(kDof), qdot(kDof);
    Eigen::VectorXd q_ref(kDof), qdot_ref(kDof), qddot_ref(kDof);
    for (int i = 0; i < steps; ++i) {
      minimum_jerk_reference(i * dt, q_ref, qdot_ref, qddot_ref);
      plant.write_goal_q(q_ref);
      plant.step();
      plant.read_state(q, qdot);
      Sample& s = log[i];
      s.t = plant.time();
      for (int j = 0; j < kDof; ++j) {
        s.q[j] = q[j];
        s.qd[j] = qdot[j];
        s.tau[j] = 0.0;
      }
    }

    std::ofstream out(csv_path);
    out.precision(17);
    out << "# plant=hardware controller=position_stream\n";
    out << "t";
    for (int i = 0; i < kDof; ++i) out << ",q" << i;
    for (int i = 0; i < kDof; ++i) out << ",qd" << i;
    for (int i = 0; i < kDof; ++i) out << ",tau" << i;
    out << ",ee_x,ee_y,ee_z\n";
    for (const auto& s : log) {
      out << s.t;
      for (int i = 0; i < kDof; ++i) out << ',' << s.q[i];
      for (int i = 0; i < kDof; ++i) out << ',' << s.qd[i];
      for (int i = 0; i < kDof; ++i) out << ',' << s.tau[i];
      out << ",0,0,0\n";
    }
    const Sample& last = log.back();
    std::printf("wrote %s  final q=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n",
                csv_path.c_str(), last.q[0], last.q[1], last.q[2], last.q[3],
                last.q[4], last.q[5]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
