// Static gravity identification: Present_Current as τ, bidirectional holds,
// least-squares m from τ − g_empty(q) = Φ_g(q) · m.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/hardware_backend.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

using arm_control::HardwareBackend;
using arm_control::kDof;
using arm_control::PinocchioDynamics;

namespace {

constexpr double kMoveS = 2.0;
constexpr double kSettleS = 1.2;
constexpr double kApproach = 0.07;
constexpr int kSamples = 25;
constexpr auto kSampleDt = std::chrono::milliseconds(20);
constexpr double kMref = 0.20;

// Hold poses keep gripper-frame z ≳ 0.22 m (9 cm hanging cup + margin)
// even with ±kApproach on lift/elbow/flex. Old set dropped to z≈0.06.
const double kPoses[][5] = {
    {0.10, 0.15, -0.35, 0.20, 0.10},
    {0.00, 0.20, -0.80, 0.15, 0.00},
    {0.45, 0.10, -1.10, 0.40, 0.20},
    {0.55, 0.40, -0.70, 0.20, 0.25},
    {0.30, 0.25, -0.95, 0.10, 0.35},
    {0.20, 0.35, -0.85, 0.30, 0.10},
};
constexpr int kNposes = 6;

Eigen::VectorXd pose_q(int i, double gripper_q) {
  Eigen::VectorXd q(kDof);
  for (int j = 0; j < 5; ++j) q[j] = kPoses[i][j];
  q[5] = gripper_q;
  return q;
}

void sleep_s(double s) {
  std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(s)));
}

Eigen::VectorXd average_amps(HardwareBackend& plant) {
  Eigen::VectorXd sum = Eigen::VectorXd::Zero(kDof);
  Eigen::VectorXd a(kDof);
  int n = 0;
  for (int i = 0; i < kSamples; ++i) {
    std::this_thread::sleep_for(kSampleDt);
    if (plant.read_current_amps(a)) {
      sum += a;
      ++n;
    }
  }
  if (n < kSamples / 2) {
    throw std::runtime_error("current read failed during hold");
  }
  return sum / static_cast<double>(n);
}

Eigen::VectorXd hold_current(HardwareBackend& plant, const Eigen::VectorXd& q,
                             double approach_sign) {
  Eigen::VectorXd q_via = q;
  q_via[1] += approach_sign * kApproach;
  q_via[2] += approach_sign * kApproach;
  q_via[3] += approach_sign * kApproach;
  plant.move_to(q_via, kMoveS);
  plant.move_to(q, kMoveS);
  sleep_s(kSettleS);
  return average_amps(plant);
}

void zero_gripper(Eigen::VectorXd& v) { v[5] = 0.0; }

}  // namespace

int main(int argc, char** argv) {
  HardwareBackend::Config cfg;
  cfg.calib_path = "so101_follower_calib.json";
  std::string payload_urdf = "models/so101/so101_dynamics_payload.urdf";
  bool empty = false;
  bool have_kt = false;
  double kt = 1.0;
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
    } else if (opt == "--urdf") {
      cfg.urdf_path = need();
    } else if (opt == "--payload-urdf") {
      payload_urdf = need();
    } else if (opt == "--gripper-closed") {
      cfg.gripper_closed = true;
    } else if (opt == "--gripper-torque") {
      cfg.gripper_torque_limit = std::stoi(need());
    } else if (opt == "--empty") {
      empty = true;
    } else if (opt == "--kt") {
      kt = std::stod(need());
      have_kt = true;
    } else {
      std::fprintf(stderr,
                   "usage: gravity_id [--port PATH] [--calib FILE] [--urdf FILE] "
                   "[--payload-urdf FILE] [--gripper-closed] "
                   "[--gripper-torque 1-1000] [--empty] [--kt N.m/A]\n"
                   "  --empty  fit kt from g(q) vs Present_Current (no mass)\n"
                   "  default  estimate payload mass (pass --kt from --empty)\n");
      return 2;
    }
  }

  try {
    if (!empty) {
      if (!have_kt) {
        std::fprintf(stderr,
                     "warning: --kt omitted, using 1.0 N.m/A (run --empty first)\n");
      }
      for (int j = 0; j < 5; ++j) cfg.kt_nm_per_a[j] = kt;
    } else {
      for (int j = 0; j < 5; ++j) cfg.kt_nm_per_a[j] = 1.0;
    }
    cfg.kt_nm_per_a[5] = 0.0;

    HardwareBackend plant(cfg);
    PinocchioDynamics empty_dyn(cfg.urdf_path);
    PinocchioDynamics payload_dyn(payload_urdf);
    plant.reset();
    plant.set_motion_profile(30, 0);

    std::printf(
        "gravity_id  %s  %d poses, approach ±%.2f rad, settle %.1f s\n",
        empty ? "fit kt (empty arm)" : "estimate mass", kNposes, kApproach,
        kSettleS);

    double num = 0.0;
    double den = 0.0;
    double kt_num[5] = {};
    double kt_den[5] = {};

    Eigen::VectorXd q(kDof), qdot(kDof), g(kDof), g_pay(kDof), tau(kDof),
        phi(kDof), extra(kDof), amps_up(kDof), amps_dn(kDof), amps(kDof);

    for (int p = 0; p < kNposes; ++p) {
      const Eigen::VectorXd q_des = pose_q(p, plant.gripper_q());
      amps_up = hold_current(plant, q_des, +1.0);
      amps_dn = hold_current(plant, q_des, -1.0);
      amps = 0.5 * (amps_up + amps_dn);

      plant.read_state(q, qdot);
      empty_dyn.gravity(q, g);
      payload_dyn.gravity(q, g_pay);
      plant.current_to_torque(amps, tau);
      phi = (g_pay - g) / kMref;
      extra = tau - g;
      zero_gripper(phi);
      zero_gripper(extra);

      std::printf(
          "pose %d  q=(%.3f,%.3f,%.3f,%.3f,%.3f)  "
          "tau=(%.3f,%.3f,%.3f)  g=(%.3f,%.3f,%.3f)\n",
          p, q[0], q[1], q[2], q[3], q[4], tau[1], tau[2], tau[3], g[1], g[2],
          g[3]);

      if (empty) {
        for (int j = 1; j <= 3; ++j) {
          kt_num[j] += g[j] * tau[j];
          kt_den[j] += tau[j] * tau[j];
        }
      } else {
        num += phi.dot(extra);
        den += phi.dot(phi);
      }
    }

    if (empty) {
      double pooled_n = 0.0;
      double pooled_d = 0.0;
      const char* names[] = {"pan", "lift", "elbow", "flex", "roll"};
      std::printf("fitted kt [N.m/A] from g / (sign·I) at kt=1 proxy:\n");
      for (int j = 1; j <= 3; ++j) {
        const double ktj = (kt_den[j] > 1e-8) ? kt_num[j] / kt_den[j] : 0.0;
        pooled_n += kt_num[j];
        pooled_d += kt_den[j];
        std::printf("  %s  %.4f\n", names[j], ktj);
      }
      const double pooled = (pooled_d > 1e-8) ? pooled_n / pooled_d : 0.0;
      std::printf("pooled (lift+elbow+flex) kt=%.4f\n"
                  "rerun loaded: gravity_id --gripper-closed --kt %.4f\n",
                  pooled, pooled);
    } else {
      if (den < 1e-8) throw std::runtime_error("gravity regressor is ~0");
      const double m = num / den;
      std::printf("static-gravity LS mass=%.4f kg  (template 0.20 kg, kt=%.4f)\n"
                  "freeze for tracking: hardware_run ... --mass %.4f\n",
                  m, kt, m);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
