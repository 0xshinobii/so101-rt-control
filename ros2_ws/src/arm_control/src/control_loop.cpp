#include "arm_control/control_loop.hpp"

namespace arm_control {

ControlLoop::ControlLoop(PlantInterface& plant, Controller& controller,
                         Eigen::VectorXd q_des)
    : plant_(plant),
      controller_(controller),
      q_des_(std::move(q_des)),
      qdot_des_(Eigen::VectorXd::Zero(plant.dof())),
      qddot_des_(Eigen::VectorXd::Zero(plant.dof())),
      q_(Eigen::VectorXd::Zero(plant.dof())),
      qdot_(Eigen::VectorXd::Zero(plant.dof())),
      tau_(Eigen::VectorXd::Zero(plant.dof())) {}

void ControlLoop::reset() {
  plant_.reset();
  controller_.reset();
}

void ControlLoop::set_reference(const Eigen::VectorXd& q_des,
                                const Eigen::VectorXd& qdot_des,
                                const Eigen::VectorXd& qddot_des) {
  q_des_ = q_des;
  qdot_des_ = qdot_des;
  qddot_des_ = qddot_des;
}

void ControlLoop::step_once(Sample& out) {
  // PRE-step read: q, qdot -> the torque is computed from these.
  plant_.read_state(q_, qdot_);
  controller_.compute(q_, qdot_, q_des_, qdot_des_, qddot_des_, tau_);
  plant_.apply_torque(tau_);

  // Advance the sim by one control period.
  plant_.step();

  // Record: q/qdot/tau are pre-step; t/ee are post-step (Phase 1.5 convention).
  const int n = plant_.dof();
  out.t = plant_.time();
  for (int i = 0; i < n && i < kDof; ++i) {
    out.q[i] = q_[i];
    out.qd[i] = qdot_[i];
    out.tau[i] = tau_[i];
  }
  out.estimated_payload_mass = controller_.estimated_payload_mass();
  const Eigen::Vector3d ee = plant_.ee_position();
  out.ee[0] = ee.x();
  out.ee[1] = ee.y();
  out.ee[2] = ee.z();
}

}  // namespace arm_control
