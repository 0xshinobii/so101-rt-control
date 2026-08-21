// Home, min-jerk stream to kTarget, optional PD hold via q_cmd = kTarget + τ/K.
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
#include "arm_control/pd_controller.hpp"

using arm_control::HardwareBackend;
using arm_control::kDof;
using arm_control::PdController;
using arm_control::Sample;

namespace {

const Eigen::VectorXd kTarget =
    (Eigen::VectorXd(kDof) << 0.6, 0.7, -0.8, 0.5, 0.4, 0.0).finished();
const Eigen::VectorXd kKp =
    (Eigen::VectorXd(kDof) << 40.0, 40.0, 25.0, 15.0, 8.0, 5.0).finished();
const Eigen::VectorXd kKd =
    (Eigen::VectorXd(kDof) << 3.0, 3.0, 2.0, 1.0, 0.6, 0.4).finished();
// pan/lift/flex not identified; elbow from g(q)/offset; wrist_roll K ignored.
const Eigen::VectorXd kKservo =
    (Eigen::VectorXd(kDof) << 50.0, 90.0, 11.0, 50.0, 50.0, 50.0).finished();
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

void record(Sample& s, double t, const Eigen::VectorXd& q,
            const Eigen::VectorXd& qdot, const Eigen::VectorXd& tau,
            const Eigen::Vector3d& ee) {
  s.t = t;
  for (int j = 0; j < kDof; ++j) {
    s.q[j] = q[j];
    s.qd[j] = qdot[j];
    s.tau[j] = tau[j];
  }
  s.ee[0] = ee.x();
  s.ee[1] = ee.y();
  s.ee[2] = ee.z();
}

}  // namespace

int main(int argc, char** argv) {
  HardwareBackend::Config cfg;
  cfg.calib_path = "so101_follower_calib.json";
  std::string csv_path = "hw_hold.csv";
  bool gravity_ff = false;
  double hold_s = 0.0;
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
    } else if (opt == "--urdf") {
      cfg.urdf_path = need();
    } else if (opt == "--gravity-ff") {
      gravity_ff = true;
    } else if (opt == "--hold-s") {
      hold_s = std::stod(need());
    } else {
      std::fprintf(stderr,
                   "usage: hardware_run [--port PATH] [--calib FILE] "
                   "[--urdf FILE] [--hold-s SEC] [--gravity-ff] [--csv FILE]\n");
      return 2;
    }
  }

  try {
    HardwareBackend plant(cfg);
    PdController pd(kKp, kKd);
    const double dt = plant.timestep();
    const int stream_steps = static_cast<int>(kDuration / dt);
    const int hold_steps =
        hold_s > 0.0 ? std::max(1, static_cast<int>(hold_s / dt)) : 0;
    std::printf(
        "hardware  port=%s dt=%.4f stream=%.1fs hold=%.1fs\n"
        "K_servo=(%.0f,%.0f,%.0f,%.0f,%.0f,%.0f)  clear workspace\n",
        cfg.port.c_str(), dt, kDuration, hold_s, kKservo[0], kKservo[1],
        kKservo[2], kKservo[3], kKservo[4], kKservo[5]);
    plant.reset();

    std::vector<Sample> log(stream_steps + hold_steps);
    Eigen::VectorXd q(kDof), qdot(kDof), g(kDof), tau(kDof), q_goal(kDof);
    Eigen::VectorXd q_ref(kDof), qdot_ref(kDof), qddot_ref(kDof);
    Eigen::VectorXd qdot_des = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(kDof);
    plant.read_state(q, qdot);

    for (int i = 0; i < stream_steps; ++i) {
      minimum_jerk_reference(i * dt, q_ref, qdot_ref, qddot_ref);
      q_goal = q_ref;
      plant.gravity(q, g);
      if (gravity_ff) {
        for (int j = 0; j < kDof; ++j) {
          q_goal[j] += std::clamp(g[j] / kKservo[j], -cfg.max_lead_q,
                                  cfg.max_lead_q);
        }
        q_goal[5] = 0.0;
      }
      plant.write_goal_q(q_goal);
      plant.step();
      plant.read_state(q, qdot);
      record(log[i], plant.time(), q, qdot, g, plant.ee_position());
    }

    Eigen::VectorXd q_hold0 = q;
    double max_drift[kDof] = {};
    for (int i = 0; i < hold_steps; ++i) {
      pd.compute(q, qdot, kTarget, qdot_des, qddot_des, tau);
      q_goal = kTarget;
      for (int j = 0; j < kDof; ++j) {
        q_goal[j] += std::clamp(tau[j] / kKservo[j], -cfg.max_lead_q,
                                cfg.max_lead_q);
      }
      q_goal[5] = 0.0;
      plant.write_goal_q(q_goal);
      plant.step();
      plant.read_state(q, qdot);
      record(log[stream_steps + i], plant.time(), q, qdot, tau,
             plant.ee_position());
      for (int j = 0; j < kDof; ++j) {
        max_drift[j] = std::max(max_drift[j], std::abs(q[j] - q_hold0[j]));
      }
    }

    std::ofstream out(csv_path);
    out.precision(17);
    out << "# plant=hardware controller=stream"
        << (hold_steps ? "+pd_hold" : "") << "\n";
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
      out << ',' << s.ee[0] << ',' << s.ee[1] << ',' << s.ee[2] << '\n';
    }
    const Sample& last = log.back();
    std::printf(
        "wrote %s  final q=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n",
        csv_path.c_str(), last.q[0], last.q[1], last.q[2], last.q[3],
        last.q[4], last.q[5]);
    if (hold_steps > 0) {
      std::printf(
          "hold start q=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n"
          "max |drift| (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f) rad\n",
          q_hold0[0], q_hold0[1], q_hold0[2], q_hold0[3], q_hold0[4],
          q_hold0[5], max_drift[0], max_drift[1], max_drift[2], max_drift[3],
          max_drift[4], max_drift[5]);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
