// Hardware tracking: τ from PD / CT / adaptive CT, realized as
// q_cmd = q_des + clamp(τ / K_servo). Hang the payload before start.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "arm_control/adaptive_computed_torque_controller.hpp"
#include "arm_control/arm_types.hpp"
#include "arm_control/computed_torque_controller.hpp"
#include "arm_control/controller.hpp"
#include "arm_control/hardware_backend.hpp"
#include "arm_control/payload_mass_rls.hpp"
#include "arm_control/pd_controller.hpp"

using arm_control::AdaptiveComputedTorqueController;
using arm_control::ComputedTorqueController;
using arm_control::Controller;
using arm_control::HardwareBackend;
using arm_control::kDof;
using arm_control::PdController;
using arm_control::Sample;

namespace {

const Eigen::VectorXd kTarget =
    (Eigen::VectorXd(kDof) << 0.6, 0.7, -0.8, 0.5, 0.4, 0.0).finished();
// Outer PD sits on the STS3215 position loop. Keep Kp/K_servo < ~0.2 or
// q_cmd = q_des + (Kp/K) e oscillates (elbow 25/11 slammed; lift hunted).
// Kd = 0: qdot is a 5 ms tick difference and chatters Goal_Position.
const Eigen::VectorXd kKp =
    (Eigen::VectorXd(kDof) << 8.0, 12.0, 2.0, 8.0, 8.0, 0.0).finished();
const Eigen::VectorXd kKd = Eigen::VectorXd::Zero(kDof);
const Eigen::VectorXd kComputedKp = Eigen::VectorXd::Constant(kDof, 40.0);
const Eigen::VectorXd kComputedKd = Eigen::VectorXd::Constant(kDof, 4.0);
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

void apply_bridge(const Eigen::VectorXd& q_des, const Eigen::VectorXd& tau,
                  double max_lead, double gripper_q, Eigen::VectorXd& q_goal) {
  q_goal = q_des;
  for (int j = 0; j < kDof; ++j) {
    q_goal[j] += std::clamp(tau[j] / kKservo[j], -max_lead, max_lead);
  }
  q_goal[5] = gripper_q;
}

// Home Φ_g ≈ 0 (links stacked), so m = (τ − g)/Φ is friction/LSB noise.
// One flexed pose (same as kTarget) + two-way current cancels Coulomb;
// extra poses mostly add wait. gravity_id still has the 6-pose LS.
constexpr double kIdGotoS = 1.5;
constexpr double kIdWiggleS = 0.45;
constexpr double kIdSettleS = 0.35;
constexpr double kIdHomeS = 2.5;
constexpr double kIdApproach = 0.07;
constexpr int kIdSamples = 12;

Eigen::VectorXd average_amps(HardwareBackend& plant) {
  Eigen::VectorXd sum = Eigen::VectorXd::Zero(kDof);
  Eigen::VectorXd a(kDof);
  int n = 0;
  for (int i = 0; i < kIdSamples; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (plant.read_current_amps(a)) {
      sum += a;
      ++n;
    }
  }
  if (n < kIdSamples / 2) throw std::runtime_error("current read failed");
  return sum / static_cast<double>(n);
}

Eigen::VectorXd hold_current(HardwareBackend& plant, const Eigen::VectorXd& q,
                             double approach_sign) {
  Eigen::VectorXd q_via = q;
  q_via[1] += approach_sign * kIdApproach;
  q_via[2] += approach_sign * kIdApproach;
  q_via[3] += approach_sign * kIdApproach;
  plant.move_to(q_via, kIdWiggleS);
  plant.move_to(q, kIdWiggleS);
  std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(kIdSettleS * 1000)));
  return average_amps(plant);
}

double identify_mass_this_run(HardwareBackend& plant,
                              AdaptiveComputedTorqueController& adaptive) {
  plant.set_motion_profile(30, 0);
  Eigen::VectorXd q_des = kTarget;
  q_des[5] = plant.gripper_q();
  plant.move_to(q_des, kIdGotoS);
  const Eigen::VectorXd amps_up = hold_current(plant, q_des, +1.0);
  const Eigen::VectorXd amps_dn = hold_current(plant, q_des, -1.0);
  Eigen::VectorXd q(kDof), qdot(kDof), tau(kDof);
  plant.read_state(q, qdot);
  plant.current_to_torque(0.5 * (amps_up + amps_dn), tau);
  double num = 0.0;
  double den = 0.0;
  adaptive.add_static_observation(q, tau, num, den);
  if (den < 1e-8) throw std::runtime_error("gravity regressor ~0");
  std::printf(
      "id  q=(%.3f,%.3f,%.3f,%.3f,%.3f)  tau=(%.3f,%.3f,%.3f)  m=%.4f kg\n",
      q[0], q[1], q[2], q[3], q[4], tau[1], tau[2], tau[3], num / den);
  Eigen::VectorXd home = Eigen::VectorXd::Zero(kDof);
  home[5] = plant.gripper_q();
  plant.move_to(home, kIdHomeS);
  plant.set_motion_profile(0, 0);
  return num / den;
}

}  // namespace

int main(int argc, char** argv) {
  HardwareBackend::Config cfg;
  cfg.calib_path = "so101_follower_calib.json";
  std::string csv_path = "hw_payload.csv";
  std::string controller_name = "pd";
  std::string payload_urdf = "models/so101/so101_dynamics_payload.urdf";
  double reference_payload_mass = 0.20;
  double payload_g = 70.0;
  double rls_lambda = 0.99;
  double mass_cal_scale = 1.0;
  double mass_cal_offset = 0.0;
  bool have_kt = false;
  bool have_mass = false;
  double freeze_mass = 0.0;
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
    } else if (opt == "--urdf") {
      cfg.urdf_path = need();
    } else if (opt == "--payload-urdf") {
      payload_urdf = need();
    } else if (opt == "--controller") {
      controller_name = need();
    } else if (opt == "--payload-g") {
      payload_g = std::stod(need());
    } else if (opt == "--gripper-closed") {
      cfg.gripper_closed = true;
    } else if (opt == "--gripper-q") {
      cfg.gripper_q = std::stod(need());
    } else if (opt == "--gripper-torque") {
      cfg.gripper_torque_limit = std::stoi(need());
    } else if (opt == "--kt") {
      const double kt = std::stod(need());
      for (int j = 0; j < 5; ++j) cfg.kt_nm_per_a[j] = kt;
      have_kt = true;
    } else if (opt == "--lambda") {
      rls_lambda = std::stod(need());
    } else if (opt == "--mass-cal-scale") {
      mass_cal_scale = std::stod(need());
    } else if (opt == "--mass-cal-offset") {
      mass_cal_offset = std::stod(need());
    } else if (opt == "--mass") {
      freeze_mass = std::stod(need());
      have_mass = true;
    } else {
      std::fprintf(stderr,
                   "usage: hardware_run [--port PATH] [--calib FILE] "
                   "[--urdf FILE] [--payload-urdf FILE] "
                   "[--controller pd|computed_torque|adaptive_computed_torque] "
                   "[--payload-g G] [--gripper-closed] [--gripper-q RAD] "
                   "[--gripper-torque 1-1000] "
                   "[--kt N.m/A] [--mass KG] [--lambda L] "
                   "[--mass-cal-scale S] [--mass-cal-offset KG] "
                   "[--csv FILE]\n");
      return 2;
    }
  }

  try {
    HardwareBackend plant(cfg);
    std::unique_ptr<Controller> controller;
    AdaptiveComputedTorqueController* adaptive = nullptr;
    if (controller_name == "pd") {
      controller = std::make_unique<PdController>(kKp, kKd);
    } else if (controller_name == "computed_torque") {
      controller = std::make_unique<ComputedTorqueController>(
          cfg.urdf_path, kComputedKp, kComputedKd);
    } else if (controller_name == "adaptive_computed_torque") {
      arm_control::PayloadMassRlsEstimator::Config est;
      est.forgetting_factor = rls_lambda;
      est.raw_mass_scale = mass_cal_scale;
      est.raw_mass_offset = mass_cal_offset;
      auto instance = std::make_unique<AdaptiveComputedTorqueController>(
          cfg.urdf_path, payload_urdf, reference_payload_mass, kComputedKp,
          kComputedKd, plant.timestep(), est);
      instance->set_use_acceleration_rls(false);
      adaptive = instance.get();
      controller = std::move(instance);
      if (!have_kt) {
        std::fprintf(stderr,
                     "warning: adaptive using default kt=1.0; run gravity_id "
                     "--empty and pass --kt\n");
      }
    } else {
      throw std::invalid_argument("controller");
    }

    std::printf(
        "hardware  controller=%s  payload=%.0f g (physical)  "
        "K=(%.0f,%.0f,%.0f,%.0f,%.0f,%.0f)\n"
        "hold %.0f g in the jaws — squeeze, home, 1-pose ID, then 4 s min-jerk\n",
        controller_name.c_str(), payload_g, kKservo[0], kKservo[1], kKservo[2],
        kKservo[3], kKservo[4], kKservo[5], payload_g);
    if (adaptive) {
      if (have_mass) {
        std::printf("static-gravity  kt=%.3f N.m/A  freeze m=%.4f kg from gravity_id\n",
                    cfg.kt_nm_per_a[1], freeze_mass);
      } else {
        std::printf("static-gravity  kt=%.3f N.m/A  1-pose two-way ID this run, then freeze\n",
                    cfg.kt_nm_per_a[1]);
        std::printf(
            "mass calibration  physical=(raw%+.5f)/%.5f, then clamp [0,0.5]\n",
            -mass_cal_offset, mass_cal_scale);
      }
    }
    plant.reset();
    controller->reset();

    const double dt = plant.timestep();
    const int steps = static_cast<int>(kDuration / dt);
    std::vector<Sample> log(steps);
    Eigen::VectorXd q(kDof), qdot(kDof), tau(kDof), q_goal(kDof);
    Eigen::VectorXd q_ref(kDof), qdot_ref(kDof), qddot_ref(kDof);
    Eigen::VectorXd amps(kDof), tau_meas(kDof);

    if (adaptive && have_mass) {
      adaptive->set_payload_mass(freeze_mass);
      std::printf("frozen mass=%.4f kg  compensated=%.4f kg\n",
                  adaptive->estimated_payload_mass(),
                  adaptive->compensated_payload_mass());
    } else if (adaptive) {
      std::printf("1-pose static ID (two-way current at kTarget) — ~8 s\n");
      const double m = identify_mass_this_run(plant, *adaptive);
      adaptive->set_identified_payload_mass(m);
      std::printf("this-run LS raw_mass=%.4f kg  calibrated=%.4f kg  (frozen)\n",
                  adaptive->estimated_payload_mass(),
                  adaptive->compensated_payload_mass());
    }

    plant.read_state(q, qdot);

    for (int i = 0; i < steps; ++i) {
      minimum_jerk_reference(i * dt, q_ref, qdot_ref, qddot_ref);
      controller->compute(q, qdot, q_ref, qdot_ref, qddot_ref, tau);
      apply_bridge(q_ref, tau, cfg.max_lead_q, plant.gripper_q(), q_goal);
      plant.write_goal_q(q_goal);
      plant.step();
      plant.read_state(q, qdot);
      Sample& s = log[i];
      s.t = plant.time();
      for (int j = 0; j < kDof; ++j) {
        s.q[j] = q[j];
        s.qd[j] = qdot[j];
        s.tau[j] = tau[j];
      }
      const Eigen::Vector3d ee = plant.ee_position();
      s.ee[0] = ee.x();
      s.ee[1] = ee.y();
      s.ee[2] = ee.z();
      s.estimated_payload_mass = controller->estimated_payload_mass();
    }

    std::ofstream out(csv_path);
    out.precision(17);
    out << "# plant=hardware controller=" << controller_name
        << " payload_g=" << payload_g << "\n";
    out << "t";
    for (int i = 0; i < kDof; ++i) out << ",q" << i;
    for (int i = 0; i < kDof; ++i) out << ",qd" << i;
    for (int i = 0; i < kDof; ++i) out << ",tau" << i;
    out << ",ee_x,ee_y,ee_z,estimated_payload_mass\n";
    for (const auto& s : log) {
      out << s.t;
      for (int i = 0; i < kDof; ++i) out << ',' << s.q[i];
      for (int i = 0; i < kDof; ++i) out << ',' << s.qd[i];
      for (int i = 0; i < kDof; ++i) out << ',' << s.tau[i];
      out << ',' << s.ee[0] << ',' << s.ee[1] << ',' << s.ee[2] << ','
          << s.estimated_payload_mass << '\n';
    }
    const Sample& last = log.back();
    std::printf(
        "wrote %s  final q=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f) "
        "ee=(%.4f,%.4f,%.4f)\n",
        csv_path.c_str(), last.q[0], last.q[1], last.q[2], last.q[3],
        last.q[4], last.q[5], last.ee[0], last.ee[1], last.ee[2]);
    if (adaptive) {
      std::printf("RLS raw_mass=%.4f kg  compensated=%.4f kg  updates=%zu\n",
                  adaptive->estimated_payload_mass(),
                  adaptive->compensated_payload_mass(),
                  adaptive->estimator_updates());
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
