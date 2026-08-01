// The control iteration, backend- and ROS-agnostic. Owns pre-allocated Eigen
// buffers so a single step allocates nothing and does no I/O -- the RT-clean
// discipline that lets Phase 3 add OS-level RT config without a rewrite.
//
// The loop deliberately reproduces the Phase 1.5 logging convention: q/qdot are
// read PRE-step and tau is computed from them; t and ee are read POST-step.
#pragma once
#include <Eigen/Dense>

#include "arm_control/arm_types.hpp"
#include "arm_control/controller.hpp"
#include "arm_control/plant_interface.hpp"

namespace arm_control {

class ControlLoop {
public:
  ControlLoop(PlantInterface& plant, Controller& controller,
              Eigen::VectorXd q_des);

  // Reset plant to home and clear controller state.
  void reset();

  // One control iteration. RT-clean: no heap allocation, no I/O. Fills `out`
  // with the pre-step state, the applied torque, and the post-step t/ee.
  void step_once(Sample& out);

  int dof() const { return plant_.dof(); }
  double timestep() const { return plant_.timestep(); }

private:
  PlantInterface& plant_;
  Controller& controller_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd tau_;
};

}  // namespace arm_control
