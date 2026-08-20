// Headless control-core runner. Drives the SO-101 through the shared
// PlantInterface / Controller abstractions with selectable PD or Pinocchio
// computed torque and step or bounded minimum-jerk reference.
//
// The hot loop is RT-clean: samples are collected into a pre-reserved buffer
// (no per-step allocation, no per-step I/O); the CSV is written once at the end.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "arm_control/adaptive_computed_torque_controller.hpp"
#include "arm_control/adaptive_computed_torque_dob_controller.hpp"
#include "arm_control/arm_types.hpp"
#include "arm_control/computed_torque_controller.hpp"
#include "arm_control/computed_torque_dob_controller.hpp"
#include "arm_control/control_loop.hpp"
#include "arm_control/controller.hpp"
#include "arm_control/mujoco_backend.hpp"
#include "arm_control/pd_controller.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

using arm_control::AdaptiveComputedTorqueController;
using arm_control::AdaptiveComputedTorqueDobController;
using arm_control::ComputedTorqueController;
using arm_control::ComputedTorqueDobController;
using arm_control::ControlLoop;
using arm_control::Controller;
using arm_control::kDof;
using arm_control::MujocoBackend;
using arm_control::PdController;
using arm_control::PinocchioDynamics;
using arm_control::Sample;

namespace {

// --- Constants mirrored exactly from the Phase 1.5 oracle (run_baseline_so101.py) ---
// Desired joint angles [rad]; gripper held at home (index 5).
const Eigen::VectorXd kTarget =
    (Eigen::VectorXd(kDof) << 0.6, 0.7, -0.8, 0.5, 0.4, 0.0).finished();
const Eigen::VectorXd kHome = Eigen::VectorXd::Zero(kDof);
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

constexpr double kDefaultDuration = 4.0;  // seconds
constexpr double kReferenceDuration = 1.0;

void minimum_jerk_segment(double time, const Eigen::VectorXd& start,
                          const Eigen::VectorXd& target,
                          Eigen::VectorXd& q, Eigen::VectorXd& qdot,
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
  q.noalias() = start + position_scale * (target - start);
  qdot.noalias() = velocity_scale * (target - start);
  qddot.noalias() = acceleration_scale * (target - start);
}

void minimum_jerk_reference(double time, Eigen::VectorXd& q,
                            Eigen::VectorXd& qdot,
                            Eigen::VectorXd& qddot) {
  minimum_jerk_segment(time, kHome, kTarget,
                       q, qdot, qddot);
}

void write_csv(const std::string& path, const std::vector<Sample>& log,
               const std::string& controller_name,
               const std::string& reference_name,
               const ComputedTorqueController* computed,
               const AdaptiveComputedTorqueController* adaptive,
               const ComputedTorqueDobController* computed_dob,
               const AdaptiveComputedTorqueDobController* adaptive_dob,
               double dt) {
  std::ofstream out(path);
  out.precision(17);  // full double precision -> the ~1e-6 trajectory diff is meaningful
  out << "# controller=" << controller_name << '\n';
  out << "# reference=" << reference_name << '\n';
  const Eigen::VectorXd* raw_peak = nullptr;
  const Eigen::VectorXd* applied_peak = nullptr;
  std::size_t saturated_samples = 0;
  if (computed) {
    raw_peak = &computed->raw_peak_torque();
    applied_peak = &computed->applied_peak_torque();
    saturated_samples = computed->saturated_samples();
  } else if (adaptive) {
    raw_peak = &adaptive->raw_peak_torque();
    applied_peak = &adaptive->applied_peak_torque();
    saturated_samples = adaptive->saturated_samples();
  } else if (computed_dob) {
    raw_peak = &computed_dob->raw_peak_torque();
    applied_peak = &computed_dob->applied_peak_torque();
    saturated_samples = computed_dob->saturated_samples();
  } else if (adaptive_dob) {
    raw_peak = &adaptive_dob->raw_peak_torque();
    applied_peak = &adaptive_dob->applied_peak_torque();
    saturated_samples = adaptive_dob->saturated_samples();
  }
  if (raw_peak) {
    out << "# raw_peak_tau=";
    for (int i = 0; i < kDof; ++i) {
      if (i) out << ',';
      out << (*raw_peak)[i];
    }
    out << "\n# applied_peak_tau=";
    for (int i = 0; i < kDof; ++i) {
      if (i) out << ',';
      out << (*applied_peak)[i];
    }
    out << "\n# saturated_samples=" << saturated_samples
        << "\n# saturation_duration="
        << saturated_samples * dt << '\n';
  }
  if (adaptive) {
    out << "# final_payload_mass_estimate="
        << adaptive->estimated_payload_mass()
        << "\n# final_compensated_payload_mass="
        << adaptive->compensated_payload_mass()
        << "\n# estimator_updates=" << adaptive->estimator_updates()
        << "\n# estimator_covariance=" << adaptive->estimator_covariance() << '\n';
  } else if (adaptive_dob) {
    out << "# final_payload_mass_estimate="
        << adaptive_dob->estimated_payload_mass()
        << "\n# final_compensated_payload_mass="
        << adaptive_dob->compensated_payload_mass()
        << "\n# estimator_updates=" << adaptive_dob->estimator_updates()
        << "\n# estimator_covariance="
        << adaptive_dob->estimator_covariance() << '\n';
  }
  out << "t";
  for (int i = 0; i < kDof; ++i) out << ",q" << i;
  for (int i = 0; i < kDof; ++i) out << ",qd" << i;
  for (int i = 0; i < kDof; ++i) out << ",tau" << i;
  out << ",ee_x,ee_y,ee_z";
  const bool has_mass = adaptive || adaptive_dob;
  const bool has_dob = computed_dob || adaptive_dob;
  if (has_mass) out << ",estimated_payload_mass";
  if (has_dob) {
    for (int i = 0; i < kDof; ++i) out << ",tau_ext_hat" << i;
    for (int i = 0; i < kDof; ++i) out << ",tau_ext_true" << i;
    out << ",force_x,force_y,force_z,force_hat_x,force_hat_y,force_hat_z"
        << ",jacobian_condition";
  }
  out << '\n';
  for (const auto& s : log) {
    out << s.t;
    for (int i = 0; i < kDof; ++i) out << ',' << s.q[i];
    for (int i = 0; i < kDof; ++i) out << ',' << s.qd[i];
    for (int i = 0; i < kDof; ++i) out << ',' << s.tau[i];
    out << ',' << s.ee[0] << ',' << s.ee[1] << ',' << s.ee[2];
    if (has_mass) out << ',' << s.estimated_payload_mass;
    if (has_dob) {
      for (double value : s.estimated_disturbance_torque) {
        out << ',' << value;
      }
      for (double value : s.true_disturbance_torque) out << ',' << value;
      for (double value : s.external_force) out << ',' << value;
      for (double value : s.estimated_external_force) out << ',' << value;
      out << ',' << s.jacobian_condition;
    }
    out << '\n';
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
  std::string payload_urdf_path =
      "models/so101/so101_dynamics_payload.urdf";
  double reference_payload_mass = 0.20;
  double plant_payload_mass = std::numeric_limits<double>::quiet_NaN();
  double duration = kDefaultDuration;
  double dob_bandwidth_hz = 8.0;
  double force_onset = std::numeric_limits<double>::infinity();
  double force_frequency_hz = 0.0;
  double freeze_rls_at = std::numeric_limits<double>::infinity();
  double freeze_dob_at = std::numeric_limits<double>::infinity();
  double second_target_onset = std::numeric_limits<double>::infinity();
  Eigen::Vector3d force_amplitude = Eigen::Vector3d::Zero();
  Eigen::VectorXd second_target = kTarget;
  arm_control::PayloadMassRlsEstimator::Config estimator_config;
  for (int i = 3; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--controller" && i + 1 < argc) {
      controller_name = argv[++i];
    } else if (option == "--urdf" && i + 1 < argc) {
      urdf_path = argv[++i];
    } else if (option == "--reference" && i + 1 < argc) {
      reference_name = argv[++i];
    } else if (option == "--payload-urdf" && i + 1 < argc) {
      payload_urdf_path = argv[++i];
    } else if (option == "--reference-payload-mass" && i + 1 < argc) {
      reference_payload_mass = std::stod(argv[++i]);
    } else if (option == "--plant-payload-mass" && i + 1 < argc) {
      plant_payload_mass = std::stod(argv[++i]);
    } else if (option == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (option == "--dob-bandwidth-hz" && i + 1 < argc) {
      dob_bandwidth_hz = std::stod(argv[++i]);
    } else if (option == "--force-onset" && i + 1 < argc) {
      force_onset = std::stod(argv[++i]);
    } else if (option == "--force" && i + 3 < argc) {
      for (int axis = 0; axis < 3; ++axis) {
        force_amplitude[axis] = std::stod(argv[++i]);
      }
    } else if (option == "--force-frequency-hz" && i + 1 < argc) {
      force_frequency_hz = std::stod(argv[++i]);
    } else if (option == "--freeze-rls-at" && i + 1 < argc) {
      freeze_rls_at = std::stod(argv[++i]);
    } else if (option == "--freeze-dob-at" && i + 1 < argc) {
      freeze_dob_at = std::stod(argv[++i]);
    } else if (option == "--second-target-onset" && i + 1 < argc) {
      second_target_onset = std::stod(argv[++i]);
    } else if (option == "--second-target" && i + kDof < argc) {
      for (int joint = 0; joint < kDof; ++joint) {
        second_target[joint] = std::stod(argv[++i]);
      }
    } else if (option == "--rls-initial-mass" && i + 1 < argc) {
      estimator_config.initial_mass = std::stod(argv[++i]);
    } else if (option == "--rls-initial-covariance" && i + 1 < argc) {
      estimator_config.initial_covariance = std::stod(argv[++i]);
    } else if (option == "--rls-forgetting-factor" && i + 1 < argc) {
      estimator_config.forgetting_factor = std::stod(argv[++i]);
    } else if (option == "--rls-max-mass" && i + 1 < argc) {
      estimator_config.max_mass = std::stod(argv[++i]);
    } else if (option == "--rls-excitation-threshold" && i + 1 < argc) {
      estimator_config.excitation_threshold = std::stod(argv[++i]);
    } else {
      throw std::invalid_argument("unknown/incomplete option: " + option);
    }
  }
  if (!(duration > 0.0) || !(dob_bandwidth_hz > 0.0) ||
      force_frequency_hz < 0.0) {
    throw std::invalid_argument(
        "duration and DOB bandwidth must be positive; force frequency nonnegative");
  }

  try {
    MujocoBackend plant(model_path);
    if (std::isfinite(plant_payload_mass)) {
      plant.set_body_mass("known_payload", plant_payload_mass);
    }
    std::unique_ptr<Controller> controller;
    ComputedTorqueController* computed = nullptr;
    AdaptiveComputedTorqueController* adaptive = nullptr;
    ComputedTorqueDobController* computed_dob = nullptr;
    AdaptiveComputedTorqueDobController* adaptive_dob = nullptr;
    if (controller_name == "pd") {
      controller = std::make_unique<PdController>(kKp, kKd);
    } else if (controller_name == "computed_torque") {
      auto instance = std::make_unique<ComputedTorqueController>(
          urdf_path, kComputedKp, kComputedKd);
      computed = instance.get();
      controller = std::move(instance);
    } else if (controller_name == "adaptive_computed_torque") {
      auto instance = std::make_unique<AdaptiveComputedTorqueController>(
          urdf_path, payload_urdf_path, reference_payload_mass, kComputedKp,
          kComputedKd, plant.timestep(), estimator_config);
      adaptive = instance.get();
      controller = std::move(instance);
    } else if (controller_name == "computed_torque_dob") {
      auto instance = std::make_unique<ComputedTorqueDobController>(
          urdf_path, kComputedKp, kComputedKd, plant.timestep(),
          dob_bandwidth_hz);
      computed_dob = instance.get();
      controller = std::move(instance);
    } else if (controller_name == "adaptive_computed_torque_dob") {
      auto instance =
          std::make_unique<AdaptiveComputedTorqueDobController>(
              urdf_path, payload_urdf_path, reference_payload_mass,
              kComputedKp, kComputedKd, plant.timestep(), estimator_config,
              dob_bandwidth_hz);
      adaptive_dob = instance.get();
      controller = std::move(instance);
    } else {
      throw std::invalid_argument(
          "controller must be pd, computed_torque, "
          "adaptive_computed_torque, computed_torque_dob, or "
          "adaptive_computed_torque_dob");
    }
    ControlLoop loop(plant, *controller, kTarget);
    if (reference_name != "step" && reference_name != "smooth") {
      throw std::invalid_argument("reference must be step or smooth");
    }

    const double dt = loop.timestep();
    const int steps = static_cast<int>(duration / dt);

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
    Eigen::VectorXd q_for_jacobian = Eigen::VectorXd::Zero(kDof);
    Eigen::Matrix<double, 3, kDof> translation_jacobian;
    Eigen::VectorXd true_disturbance = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd estimated_disturbance = Eigen::VectorXd::Zero(kDof);
    Eigen::Vector3d estimated_force = Eigen::Vector3d::Zero();
    std::unique_ptr<PinocchioDynamics> truth_dynamics;
    if (computed_dob || adaptive_dob) {
      truth_dynamics = std::make_unique<PinocchioDynamics>(urdf_path);
    }
    bool rls_frozen = false;
    bool dob_frozen = false;

    for (int i = 0; i < steps; ++i) {
      const double time = i * dt;
      if (reference_name == "smooth") {
        if (time < second_target_onset) {
          minimum_jerk_reference(time, q_ref, qdot_ref, qddot_ref);
        } else {
          minimum_jerk_segment(time - second_target_onset, kTarget,
                               second_target, q_ref, qdot_ref, qddot_ref);
        }
        loop.set_reference(q_ref, qdot_ref, qddot_ref);
      }

      if (!rls_frozen && time >= freeze_rls_at) {
        if (adaptive) adaptive->set_estimator_frozen(true);
        if (adaptive_dob) adaptive_dob->set_estimator_frozen(true);
        rls_frozen = true;
      }
      if (!dob_frozen && time >= freeze_dob_at) {
        if (computed_dob) computed_dob->set_observer_frozen(true);
        if (adaptive_dob) adaptive_dob->set_observer_frozen(true);
        dob_frozen = true;
      }

      Eigen::Vector3d applied_force = Eigen::Vector3d::Zero();
      if (time >= force_onset) {
        const double scale =
            force_frequency_hz > 0.0
                ? std::sin(2.0 * std::acos(-1.0) * force_frequency_hz *
                           (time - force_onset))
                : 1.0;
        applied_force = scale * force_amplitude;
      }
      plant.set_ee_force_world(applied_force);
      loop.step_once(log[i]);

      for (int joint = 0; joint < kDof; ++joint) {
        q_for_jacobian[joint] = log[i].q[joint];
      }
      for (int axis = 0; axis < 3; ++axis) {
        log[i].external_force[axis] = applied_force[axis];
      }
      if (truth_dynamics) {
        truth_dynamics->frame_translation_jacobian(
            q_for_jacobian, "gripper_frame_joint", translation_jacobian);
        true_disturbance.noalias() =
            translation_jacobian.transpose() * applied_force;
        for (int joint = 0; joint < kDof; ++joint) {
          log[i].true_disturbance_torque[joint] =
              true_disturbance[joint];
        }
        for (int joint = 0; joint < kDof; ++joint) {
          estimated_disturbance[joint] =
              log[i].estimated_disturbance_torque[joint];
        }
        constexpr double damping = 1e-4;
        Eigen::Matrix3d normal =
            translation_jacobian * translation_jacobian.transpose();
        normal.diagonal().array() += damping * damping;
        estimated_force =
            normal.ldlt().solve(translation_jacobian *
                                estimated_disturbance);
        Eigen::JacobiSVD<Eigen::Matrix<double, 3, kDof>> svd(
            translation_jacobian);
        const auto singular = svd.singularValues();
        log[i].jacobian_condition =
            singular[2] > 1e-12
                ? singular[0] / singular[2]
                : std::numeric_limits<double>::infinity();
        for (int axis = 0; axis < 3; ++axis) {
          log[i].estimated_external_force[axis] =
              estimated_force[axis];
        }
      }
    }

    write_csv(csv_path, log, controller_name, reference_name, computed,
              adaptive, computed_dob, adaptive_dob, dt);
    const Sample& last = log.back();
    std::printf("done. wrote %s (%d samples). final t=%.4f q0=%.6f ee=(%.4f,%.4f,%.4f)\n",
                csv_path.c_str(), static_cast<int>(log.size()), last.t,
                last.q[0], last.ee[0], last.ee[1], last.ee[2]);
    if (computed) {
      std::printf("computed torque: saturated_samples=%zu/%zu (%.4f s)\n",
                  computed->saturated_samples(), computed->sample_count(),
                  computed->saturated_samples() * dt);
    } else if (adaptive) {
      std::printf(
          "adaptive computed torque: raw_mass=%.6f kg compensated_mass=%.6f "
          "kg updates=%zu "
          "saturated_samples=%zu/%zu (%.4f s)\n",
          adaptive->estimated_payload_mass(),
          adaptive->compensated_payload_mass(), adaptive->estimator_updates(),
          adaptive->saturated_samples(), adaptive->sample_count(),
          adaptive->saturated_samples() * dt);
    } else if (computed_dob) {
      std::printf("computed torque DOB: saturated_samples=%zu/%zu (%.4f s)\n",
                  computed_dob->saturated_samples(),
                  computed_dob->sample_count(),
                  computed_dob->saturated_samples() * dt);
    } else if (adaptive_dob) {
      std::printf(
          "adaptive computed torque DOB: raw_mass=%.6f kg "
          "compensated_mass=%.6f kg updates=%zu "
          "saturated_samples=%zu/%zu (%.4f s)\n",
          adaptive_dob->estimated_payload_mass(),
          adaptive_dob->compensated_payload_mass(),
          adaptive_dob->estimator_updates(),
          adaptive_dob->saturated_samples(), adaptive_dob->sample_count(),
          adaptive_dob->saturated_samples() * dt);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
