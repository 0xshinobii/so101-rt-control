// Hardware tracking: τ from PD / CT / adaptive CT, driven by the same
// ControlLoop the simulation node uses. HardwareBackend realizes τ as
// q_cmd = q_des + clamp(τ / K_servo); see hardware_backend.cpp.
// Hang the payload before start.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "arm_control/adaptive_computed_torque_controller.hpp"
#include "arm_control/arm_types.hpp"
#include "arm_control/computed_torque_controller.hpp"
#include "arm_control/control_loop.hpp"
#include "arm_control/controller.hpp"
#include "arm_control/hardware_backend.hpp"
#include "arm_control/payload_mass_rls.hpp"
#include "arm_control/pd_controller.hpp"
#include "arm_control/rt_thread.hpp"

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

// One encoder LSB. Jaw drift below this is unresolvable, not absence of drift.
constexpr double kTickRad = 2.0 * M_PI / 4096.0;

double identify_mass_this_run(HardwareBackend& plant,
                              AdaptiveComputedTorqueController& adaptive) {
  plant.set_motion_profile(30, 0);
  Eigen::VectorXd q_des = kTarget;
  q_des[5] = plant.gripper_q();
  // The affine calibration was fitted with no grasp change between
  // identification and tracking, so the grasp must not be re-established here.
  // Instead, record whether the two-way approach moved the jaw at all.
  Eigen::VectorXd q_probe(kDof), qdot_probe(kDof);
  plant.read_state(q_probe, qdot_probe);
  const double jaw_before = q_probe[5];
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
  plant.read_state(q_probe, qdot_probe);
  const double jaw_drift = q_probe[5] - jaw_before;
  std::printf("id  jaw q %+.4f -> %+.4f rad  drift %+.4f (%.1f LSB)\n",
              jaw_before, q_probe[5], jaw_drift,
              std::abs(jaw_drift) / kTickRad);
  if (std::abs(jaw_drift) > 2.0 * kTickRad) {
    std::fprintf(stderr,
                 "warning: jaw moved %.1f LSB during identification — the "
                 "payload may sit differently than when its mass was measured, "
                 "and the affine calibration assumes it does not\n",
                 std::abs(jaw_drift) / kTickRad);
  }
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
  bool motion_rls = false;
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
    } else if (opt == "--motion-rls") {
      motion_rls = true;
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
                   "[--motion-rls] "
                   "[--csv FILE]\n");
      return 2;
    }
  }

  try {
    if (motion_rls && controller_name != "adaptive_computed_torque") {
      throw std::invalid_argument("--motion-rls requires adaptive controller");
    }
    if (motion_rls && have_mass) {
      throw std::invalid_argument("--motion-rls cannot be combined with --mass");
    }
    if (motion_rls &&
        (mass_cal_scale != 1.0 || mass_cal_offset != 0.0)) {
      throw std::invalid_argument(
          "--motion-rls must use identity mass calibration");
    }

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
      instance->set_use_acceleration_rls(motion_rls);
      adaptive = instance.get();
      controller = std::move(instance);
      if (!motion_rls && !have_kt) {
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
        "hold %.0f g in the jaws — squeeze, home, then 4 s min-jerk\n",
        controller_name.c_str(), payload_g, cfg.k_servo[0], cfg.k_servo[1],
        cfg.k_servo[2], cfg.k_servo[3], cfg.k_servo[4], cfg.k_servo[5],
        payload_g);
    if (adaptive) {
      if (motion_rls) {
        std::printf(
            "motion RLS  qdd from encoder ticks, no pre-run static ID\n");
      } else if (have_mass) {
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
    std::vector<double> compensated_mass_log(
        steps, std::numeric_limits<double>::quiet_NaN());
    Eigen::VectorXd q_ref(kDof), qdot_ref(kDof), qddot_ref(kDof);

    if (adaptive && motion_rls) {
      std::printf("motion RLS starts at m=%.4f kg and updates during tracking\n",
                  adaptive->estimated_payload_mass());
    } else if (adaptive && have_mass) {
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

    // Same ControlLoop the simulation node runs; only the backend differs.
    // Logging therefore follows the shared convention: q/qdot/tau are the
    // PRE-step state and the torque computed from it, t is POST-step.
    arm_control::ControlLoop loop(plant, *controller, kTarget);

    // Enter RT policy only after construction, homing, ID and log allocation.
    // The tracking loop runs on this thread; setup does not need FIFO priority.
    // docker/run_i7.sh supplies CAP_SYS_NICE and rtprio/memlock limits.
    const arm_control::RtStatus rt =
        arm_control::configure_rt_thread(arm_control::RtConfig{});
    std::printf("rt  mlockall=%d fifo=%d affinity=%d cstates=%d\n",
                rt.memory_locked, rt.fifo_set, rt.affinity_set,
                rt.cstates_suppressed);
    if (!rt.error.empty()) {
      std::fprintf(stderr, "rt warnings: %s\n", rt.error.c_str());
    }
    if (!rt.fifo_set || !rt.memory_locked) {
      std::fprintf(stderr,
                   "warning: not running SCHED_FIFO with locked memory; this "
                   "run is not a real-time measurement\n");
    }
    plant.reset_health();

    for (int i = 0; i < steps; ++i) {
      minimum_jerk_reference(i * dt, q_ref, qdot_ref, qddot_ref);
      loop.set_reference(q_ref, qdot_ref, qddot_ref);
      loop.step_once(log[i]);
      if (adaptive) {
        compensated_mass_log[i] = adaptive->compensated_payload_mass();
      }
    }

    const auto health = plant.health();
    // File output and teardown are not control work; leave FIFO policy first.
    if (rt.fifo_set) {
      sched_param normal{};
      if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal) != 0) {
        std::fprintf(stderr, "warning: failed to restore SCHED_OTHER\n");
      }
    }
    if (rt.memory_locked && munlockall() != 0) {
      std::fprintf(stderr, "warning: failed to unlock process memory\n");
    }

    std::ofstream out(csv_path);
    out.precision(17);
    out << "# plant=hardware controller=" << controller_name
        << " payload_g=" << payload_g;
    if (adaptive) {
      out << " estimator=" << (motion_rls ? "motion_rls" : "static_id")
          << " mass_cal_scale=" << mass_cal_scale
          << " mass_cal_offset=" << mass_cal_offset;
    }
    out << " rt_fifo=" << (rt.fifo_set ? 1 : 0)
        << " rt_mlockall=" << (rt.memory_locked ? 1 : 0)
        << " stale_reads=" << health.stale_reads
        << " failed_writes=" << health.failed_writes
        << " tick_clamps=" << health.tick_clamps
        << " lead_saturations=" << health.lead_saturations
        << " late_ticks=" << health.late_ticks
        << " max_late_us=" << health.max_late_us
        << " bus_timeouts=" << health.bus_timeouts
        << " bus_checksum_errors=" << health.bus_checksum_errors << "\n";
    out << "t";
    for (int i = 0; i < kDof; ++i) out << ",q" << i;
    for (int i = 0; i < kDof; ++i) out << ",qd" << i;
    for (int i = 0; i < kDof; ++i) out << ",tau" << i;
    out << ",ee_x,ee_y,ee_z,estimated_payload_mass,"
           "compensated_payload_mass\n";
    for (int sample_index = 0; sample_index < steps; ++sample_index) {
      const auto& s = log[sample_index];
      out << s.t;
      for (int i = 0; i < kDof; ++i) out << ',' << s.q[i];
      for (int i = 0; i < kDof; ++i) out << ',' << s.qd[i];
      for (int i = 0; i < kDof; ++i) out << ',' << s.tau[i];
      out << ',' << s.ee[0] << ',' << s.ee[1] << ',' << s.ee[2] << ','
          << s.estimated_payload_mass << ','
          << compensated_mass_log[sample_index] << '\n';
    }
    const Sample& last = log.back();
    std::printf(
        "wrote %s  final q=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f) "
        "ee=(%.4f,%.4f,%.4f)\n",
        csv_path.c_str(), last.q[0], last.q[1], last.q[2], last.q[3],
        last.q[4], last.q[5], last.ee[0], last.ee[1], last.ee[2]);
    std::printf(
        "run health  stale_reads=%d  failed_writes=%d  tick_clamps=%d  "
        "lead_saturations=%d  late_ticks=%d (max %.1f us)  bus_timeouts=%llu  "
        "checksum_errors=%llu\n",
        health.stale_reads, health.failed_writes, health.tick_clamps,
        health.lead_saturations, health.late_ticks, health.max_late_us,
        static_cast<unsigned long long>(health.bus_timeouts),
        static_cast<unsigned long long>(health.bus_checksum_errors));
    if (health.stale_reads || health.failed_writes || health.tick_clamps ||
        health.lead_saturations || health.late_ticks || health.bus_timeouts ||
        health.bus_checksum_errors) {
      std::fprintf(stderr,
                   "warning: this run is not clean — a stale read duplicates "
                   "a sample, a failed write leaves the previous command in "
                   "place, a tick clamp truncates a command, a lead "
                   "saturation means the controller wanted more authority than "
                   "the bridge can express, and a late tick means the logged "
                   "time axis drifted from real time\n");
    }
    if (adaptive) {
      std::printf("mass raw=%.4f kg  compensated=%.4f kg  estimator updates=%zu\n",
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
