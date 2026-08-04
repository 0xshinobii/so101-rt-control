// Headless control-core runner. Drives the SO-101 through the shared
// PlantInterface / Controller abstractions with selectable PD or Pinocchio
// computed torque and step or bounded minimum-jerk reference.
//
// The hot loop is RT-clean: samples are collected into a pre-reserved buffer
// (no per-step allocation, no per-step I/O); the CSV is written once at the end.
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/computed_torque_controller.hpp"
#include "arm_control/control_loop.hpp"
#include "arm_control/controller.hpp"
#include "arm_control/mujoco_backend.hpp"
#include "arm_control/pd_controller.hpp"

using arm_control::ComputedTorqueController;
using arm_control::ControlLoop;
using arm_control::Controller;
using arm_control::kDof;
using arm_control::MujocoBackend;
using arm_control::PdController;
using arm_control::Sample;

namespace {

// --- Constants mirrored exactly from the Phase 1.5 oracle (run_baseline_so101.py) ---
// Desired joint angles [rad]; gripper held at home (index 5).
const Eigen::VectorXd kTarget =
    (Eigen::VectorXd(kDof) << 0.6, 0.7, -0.8, 0.5, 0.4, 0.0).finished();
// Naive PD gains (ki = 0), per joint.
const Eigen::VectorXd kKp =
    (Eigen::VectorXd(kDof) << 40.0, 40.0, 25.0, 15.0, 8.0, 5.0).finished();
const Eigen::VectorXd kKd =
    (Eigen::VectorXd(kDof) << 3.0, 3.0, 2.0, 1.0, 0.6, 0.4).finished();
// Critically damped acceleration-domain gains: wn=20 rad/s, zeta=1. The
// bounded reference keeps the corresponding transient torque inside limits.
const Eigen::VectorXd kComputedKp =
    Eigen::VectorXd::Constant(kDof, 400.0);
const Eigen::VectorXd kComputedKd =
    Eigen::VectorXd::Constant(kDof, 40.0);

constexpr double kDuration = 4.0;  // seconds
constexpr double kReferenceDuration = 1.0;

void minimum_jerk_reference(double time, Eigen::VectorXd& q,
                            Eigen::VectorXd& qdot,
                            Eigen::VectorXd& qddot) {
  const double s = std::clamp(time / kReferenceDuration, 0.0, 1.0);
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  const double position_scale = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  const double velocity_scale =
      (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / kReferenceDuration;
  const double acceleration_scale =
      (60.0 * s - 180.0 * s2 + 120.0 * s3) /
      (kReferenceDuration * kReferenceDuration);
  q = position_scale * kTarget;
  qdot = velocity_scale * kTarget;
  qddot = acceleration_scale * kTarget;
}

void write_csv(const std::string& path, const std::vector<Sample>& log,
               const std::string& controller_name,
               const std::string& reference_name,
               const ComputedTorqueController* computed, double dt) {
  std::ofstream out(path);
  out.precision(17);  // full double precision -> the ~1e-6 trajectory diff is meaningful
  out << "# controller=" << controller_name << '\n';
  out << "# reference=" << reference_name << '\n';
  if (computed) {
    out << "# raw_peak_tau=";
    for (int i = 0; i < kDof; ++i) {
      if (i) out << ',';
      out << computed->raw_peak_torque()[i];
    }
    out << "\n# applied_peak_tau=";
    for (int i = 0; i < kDof; ++i) {
      if (i) out << ',';
      out << computed->applied_peak_torque()[i];
    }
    out << "\n# saturated_samples=" << computed->saturated_samples()
        << "\n# saturation_duration="
        << computed->saturated_samples() * dt << '\n';
  }
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
}

}  // namespace

int main(int argc, char** argv) {
  const std::string model_path =
      (argc > 1) ? argv[1] : "models/so101/scene_torque.xml";
  const std::string csv_path = (argc > 2) ? argv[2] : "cpp_baseline_so101.csv";
  std::string controller_name = "pd";
  std::string reference_name = "step";
  std::string urdf_path = "models/so101/so101_dynamics.urdf";
  for (int i = 3; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--controller" && i + 1 < argc) {
      controller_name = argv[++i];
    } else if (option == "--urdf" && i + 1 < argc) {
      urdf_path = argv[++i];
    } else if (option == "--reference" && i + 1 < argc) {
      reference_name = argv[++i];
    } else {
      throw std::invalid_argument("unknown/incomplete option: " + option);
    }
  }

  try {
    MujocoBackend plant(model_path);
    std::unique_ptr<Controller> controller;
    ComputedTorqueController* computed = nullptr;
    if (controller_name == "pd") {
      controller = std::make_unique<PdController>(kKp, kKd);
    } else if (controller_name == "computed_torque") {
      auto instance = std::make_unique<ComputedTorqueController>(
          urdf_path, kComputedKp, kComputedKd);
      computed = instance.get();
      controller = std::move(instance);
    } else {
      throw std::invalid_argument("controller must be pd or computed_torque");
    }
    ControlLoop loop(plant, *controller, kTarget);
    if (reference_name != "step" && reference_name != "smooth") {
      throw std::invalid_argument("reference must be step or smooth");
    }

    const double dt = loop.timestep();
    const int steps = static_cast<int>(kDuration / dt);

    std::printf(
        "model=%s controller=%s reference=%s dof=%d dt=%.4f steps=%d\n",
        model_path.c_str(), controller_name.c_str(), reference_name.c_str(),
        loop.dof(), dt, steps);

    loop.reset();

    std::vector<Sample> log;
    log.resize(steps);  // pre-allocated: the loop body allocates nothing
    Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd qdot_ref = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd qddot_ref = Eigen::VectorXd::Zero(kDof);

    for (int i = 0; i < steps; ++i) {
      if (reference_name == "smooth") {
        minimum_jerk_reference(i * dt, q_ref, qdot_ref, qddot_ref);
        loop.set_reference(q_ref, qdot_ref, qddot_ref);
      }
      loop.step_once(log[i]);
    }

    write_csv(csv_path, log, controller_name, reference_name, computed, dt);
    const Sample& last = log.back();
    std::printf("done. wrote %s (%d samples). final t=%.4f q0=%.6f ee=(%.4f,%.4f,%.4f)\n",
                csv_path.c_str(), static_cast<int>(log.size()), last.t,
                last.q[0], last.ee[0], last.ee[1], last.ee[2]);
    if (computed) {
      std::printf("computed torque: saturated_samples=%zu/%zu (%.4f s)\n",
                  computed->saturated_samples(), computed->sample_count(),
                  computed->saturated_samples() * dt);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
