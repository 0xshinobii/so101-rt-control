// The controller abstraction: same signature for the naive PD now and for
// computed-torque / adaptive+DOB controllers. Always outputs torque, so it is
// 100%
// backend-agnostic (see PlantInterface).
#pragma once
#include <limits>

#include <Eigen/Dense>

namespace arm_control {

class Controller {
public:
  virtual ~Controller() = default;

  // Compute joint torques that drive q toward q_des. tau_out is caller-owned
  // and pre-sized, so a correct implementation allocates nothing here.
  virtual void compute(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                       const Eigen::VectorXd& q_des,
                       const Eigen::VectorXd& qdot_des,
                       const Eigen::VectorXd& qddot_des,
                       Eigen::VectorXd& tau_out) = 0;

  // Clear any internal state (e.g. an integral accumulator) between runs.
  virtual void reset() = 0;

  // Optional fixed-size telemetry. Non-estimating controllers retain NaN.
  virtual double estimated_payload_mass() const {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Optional six-joint external-disturbance estimate [N.m]. Null means that
  // the controller has no disturbance observer.
  virtual const Eigen::VectorXd* estimated_disturbance_torque() const {
    return nullptr;
  }
};

}  // namespace arm_control
