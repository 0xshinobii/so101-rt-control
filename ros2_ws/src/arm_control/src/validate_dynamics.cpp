#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include "arm_control/arm_types.hpp"
#include "arm_control/pinocchio_dynamics.hpp"

namespace {

using arm_control::kDof;
using arm_control::PinocchioDynamics;

constexpr double kTorqueTolerance = 1e-6;
constexpr double kMassAbsoluteTolerance = 1e-9;
constexpr double kMassRelativeTolerance = 1e-5;

const std::array<Eigen::Matrix<double, kDof, 1>, 5> kPoses = {
    (Eigen::Matrix<double, kDof, 1>() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
    (Eigen::Matrix<double, kDof, 1>() << 0.6, 0.7, -0.8, 0.5, 0.4, 0.0).finished(),
    (Eigen::Matrix<double, kDof, 1>() << 0.3, 0.35, -0.4, 0.25, 0.2, 0.0).finished(),
    // Extended, gravity-heavy pose.
    (Eigen::Matrix<double, kDof, 1>() << 0.0, 1.2, -0.2, 0.0, 0.0, 0.0).finished(),
    (Eigen::Matrix<double, kDof, 1>() << -0.5, -0.8, 0.9, -0.6, 0.3, 0.2).finished()};

const std::array<Eigen::Matrix<double, kDof, 1>, 2> kVelocities = {
    (Eigen::Matrix<double, kDof, 1>() << 0.2, -0.3, 0.25, -0.15, 0.1, -0.05).finished(),
    (Eigen::Matrix<double, kDof, 1>() << -0.4, 0.2, -0.1, 0.3, -0.25, 0.15).finished()};

class MujocoModel {
public:
  explicit MujocoModel(const std::string& xml_path) {
    char error[1024] = "";
    model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      throw std::runtime_error("mj_loadXML failed for '" + xml_path + "': " + error);
    }
    data_ = mj_makeData(model_);
    if (!data_) throw std::runtime_error("mj_makeData failed");
    if (model_->nq != kDof || model_->nv != kDof) {
      throw std::runtime_error("MuJoCo validation model must have six DOF");
    }

    for (int i = 0; i < kDof; ++i) {
      const int joint_id =
          mj_name2id(model_, mjOBJ_JOINT, PinocchioDynamics::kJointNames[i].c_str());
      if (joint_id < 0) {
        throw std::runtime_error("MuJoCo joint not found: " +
                                 PinocchioDynamics::kJointNames[i]);
      }
      q_index_[i] = model_->jnt_qposadr[joint_id];
      v_index_[i] = model_->jnt_dofadr[joint_id];
    }

  }

  ~MujocoModel() {
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
  }

  void evaluate(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
                Eigen::VectorXd& bias, Eigen::MatrixXd& mass) {
    mj_resetData(model_, data_);
    for (int i = 0; i < kDof; ++i) {
      data_->qpos[q_index_[i]] = q[i];
      data_->qvel[v_index_[i]] = qdot[i];
    }
    mj_forward(model_, data_);

    bias.resize(kDof);
    for (int i = 0; i < kDof; ++i) bias[i] = data_->qfrc_bias[v_index_[i]];

    std::vector<mjtNum> dense(model_->nv * model_->nv);
    mj_fullM(model_, data_, dense.data());
    mass.resize(kDof, kDof);
    for (int row = 0; row < kDof; ++row) {
      for (int col = 0; col < kDof; ++col) {
        mass(row, col) =
            dense[v_index_[row] * model_->nv + v_index_[col]];
      }
    }
  }

private:
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::array<int, kDof> q_index_{};
  std::array<int, kDof> v_index_{};
};

bool mass_is_close(const Eigen::MatrixXd& pin, const Eigen::MatrixXd& mj,
                   double& max_abs, double& max_rel) {
  bool ok = true;
  max_abs = 0.0;
  max_rel = 0.0;
  for (int row = 0; row < kDof; ++row) {
    for (int col = 0; col < kDof; ++col) {
      const double delta = std::abs(pin(row, col) - mj(row, col));
      const double scale = std::max(std::abs(pin(row, col)), std::abs(mj(row, col)));
      const double relative = delta / std::max(scale, kMassAbsoluteTolerance);
      max_abs = std::max(max_abs, delta);
      max_rel = std::max(max_rel, relative);
      if (delta > kMassAbsoluteTolerance + kMassRelativeTolerance * scale) ok = false;
    }
  }
  return ok;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s MODEL.xml MODEL.urdf\n", argv[0]);
    return 2;
  }

  try {
    MujocoModel mujoco(argv[1]);
    PinocchioDynamics pinocchio(argv[2]);
    bool all_ok = true;
    double worst_gravity = 0.0;
    double worst_bias = 0.0;
    double worst_mass_abs = 0.0;
    double worst_mass_rel = 0.0;
    double worst_holding_torque = 0.0;

    Eigen::VectorXd pin_value(kDof), mj_value(kDof);
    Eigen::MatrixXd pin_mass(kDof, kDof), mj_mass(kDof, kDof);
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(kDof);

    for (size_t pose_index = 0; pose_index < kPoses.size(); ++pose_index) {
      const Eigen::VectorXd q = kPoses[pose_index];
      mujoco.evaluate(q, zero, mj_value, mj_mass);
      pinocchio.gravity(q, pin_value);
      worst_holding_torque =
          std::max(worst_holding_torque, mj_value.cwiseAbs().maxCoeff());
      const double gravity_error = (pin_value - mj_value).cwiseAbs().maxCoeff();
      worst_gravity = std::max(worst_gravity, gravity_error);
      all_ok &= gravity_error <= kTorqueTolerance;

      pinocchio.mass_matrix(q, pin_mass);
      double mass_abs = 0.0;
      double mass_rel = 0.0;
      const bool mass_ok = mass_is_close(pin_mass, mj_mass, mass_abs, mass_rel);
      worst_mass_abs = std::max(worst_mass_abs, mass_abs);
      worst_mass_rel = std::max(worst_mass_rel, mass_rel);
      all_ok &= mass_ok;

      std::printf("pose[%zu] gravity max|d|=%.3e, M max|d|=%.3e max_rel=%.3e\n",
                  pose_index, gravity_error, mass_abs, mass_rel);

      for (size_t velocity_index = 0; velocity_index < kVelocities.size();
           ++velocity_index) {
        mujoco.evaluate(q, kVelocities[velocity_index], mj_value, mj_mass);
        pinocchio.nonlinear_effects(q, kVelocities[velocity_index], pin_value);
        const double bias_error = (pin_value - mj_value).cwiseAbs().maxCoeff();
        worst_bias = std::max(worst_bias, bias_error);
        all_ok &= bias_error <= kTorqueTolerance;
        std::printf("  velocity[%zu] bias max|d|=%.3e\n", velocity_index,
                    bias_error);
      }
    }

    std::printf(
        "worst: gravity_error=%.3e N.m, bias_error=%.3e N.m, "
        "M_abs=%.3e, M_rel=%.3e, holding_tau=%.3f N.m (%.1f%% of limit)\n",
        worst_gravity, worst_bias, worst_mass_abs, worst_mass_rel,
        worst_holding_torque, 100.0 * worst_holding_torque / 2.94);
    std::printf("RIGID-BODY EQUIVALENCE: %s\n", all_ok ? "PASS" : "FAIL");
    return all_ok ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "error: %s\n", error.what());
    return 2;
  }
}
