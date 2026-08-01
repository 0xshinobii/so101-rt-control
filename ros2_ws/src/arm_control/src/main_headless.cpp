// Headless control-core runner (steps 1-3). Drives the SO-101 from its home
// pose to the Phase 1.5 target under the naive PD law, through the
// PlantInterface / Controller abstractions, and writes a trajectory CSV for
// validation against the Python oracle (tools/arm_bench.py).
//
// The hot loop is RT-clean: samples are collected into a pre-reserved buffer
// (no per-step allocation, no per-step I/O); the CSV is written once at the end.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/control_loop.hpp"
#include "arm_control/mujoco_backend.hpp"
#include "arm_control/pd_controller.hpp"

using arm_control::ControlLoop;
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

constexpr double kDuration = 4.0;  // seconds

void write_csv(const std::string& path, const std::vector<Sample>& log) {
  std::ofstream out(path);
  out.precision(17);  // full double precision -> the ~1e-6 trajectory diff is meaningful
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

  try {
    MujocoBackend plant(model_path);
    PdController controller(kKp, kKd);
    ControlLoop loop(plant, controller, kTarget);

    const double dt = loop.timestep();
    const int steps = static_cast<int>(kDuration / dt);

    std::printf("model=%s dof=%d dt=%.4f steps=%d\n", model_path.c_str(),
                loop.dof(), dt, steps);

    loop.reset();

    std::vector<Sample> log;
    log.resize(steps);  // pre-allocated: the loop body allocates nothing

    for (int i = 0; i < steps; ++i) {
      loop.step_once(log[i]);
    }

    write_csv(csv_path, log);
    const Sample& last = log.back();
    std::printf("done. wrote %s (%d samples). final t=%.4f q0=%.6f ee=(%.4f,%.4f,%.4f)\n",
                csv_path.c_str(), static_cast<int>(log.size()), last.t,
                last.q[0], last.ee[0], last.ee[1], last.ee[2]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
