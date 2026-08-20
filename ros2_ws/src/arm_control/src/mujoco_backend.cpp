#include "arm_control/mujoco_backend.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <stdexcept>

namespace arm_control {

MujocoBackend::MujocoBackend(const std::string& xml_path,
                             const std::string& ee_site,
                             const std::string& keyframe) {
  char error[1000] = "";
  m_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  if (!m_) {
    throw std::runtime_error("mj_loadXML failed for '" + xml_path +
                             "': " + error);
  }
  d_ = mj_makeData(m_);

  ee_site_id_ = mj_name2id(m_, mjOBJ_SITE, ee_site.c_str());
  if (ee_site_id_ < 0) {
    throw std::runtime_error("EE site not found: " + ee_site);
  }
  ee_body_id_ = m_->site_bodyid[ee_site_id_];
  home_id_ = mj_name2id(m_, mjOBJ_KEY, keyframe.c_str());
  if (home_id_ < 0) {
    throw std::runtime_error("keyframe not found: " + keyframe);
  }

  reset();
}

MujocoBackend::~MujocoBackend() {
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

void MujocoBackend::read_state(Eigen::VectorXd& q, Eigen::VectorXd& qdot) {
  for (int i = 0; i < m_->nq; ++i) q[i] = d_->qpos[i];
  for (int i = 0; i < m_->nv; ++i) qdot[i] = d_->qvel[i];
}

void MujocoBackend::apply_torque(const Eigen::VectorXd& tau) {
  for (int i = 0; i < m_->nu; ++i) d_->ctrl[i] = tau[i];
}

void MujocoBackend::step() { mj_step(m_, d_); }

Eigen::Vector3d MujocoBackend::ee_position() {
  const mjtNum* p = &d_->site_xpos[3 * ee_site_id_];
  return Eigen::Vector3d(p[0], p[1], p[2]);
}

void MujocoBackend::set_ee_force_world(const Eigen::Vector3d& force) {
  if (!force.allFinite()) {
    throw std::invalid_argument("EE force must be finite");
  }
  for (int i = 0; i < m_->nv; ++i) d_->qfrc_applied[i] = 0.0;
  const mjtNum force_mj[3] = {force.x(), force.y(), force.z()};
  const mjtNum torque_mj[3] = {0.0, 0.0, 0.0};
  const mjtNum* point = &d_->site_xpos[3 * ee_site_id_];
  mj_applyFT(m_, d_, force_mj, torque_mj, point, ee_body_id_,
             d_->qfrc_applied);
}

void MujocoBackend::applied_generalized_force(
    Eigen::VectorXd& torque_out) const {
  if (torque_out.size() != m_->nv) torque_out.resize(m_->nv);
  for (int i = 0; i < m_->nv; ++i) {
    torque_out[i] = d_->qfrc_applied[i];
  }
}

void MujocoBackend::reset() {
  // Start from the home keyframe (a keyframe alone does not change mj_resetData).
  mj_resetDataKeyframe(m_, d_, home_id_);
  mj_forward(m_, d_);  // populate site_xpos etc. for a valid pre-loop read
  for (int i = 0; i < m_->nv; ++i) d_->qfrc_applied[i] = 0.0;
}

void MujocoBackend::set_body_mass(const std::string& body_name, double mass) {
  if (!std::isfinite(mass) || mass < 0.0) {
    throw std::invalid_argument("body mass must be finite and non-negative");
  }
  const int body_id = mj_name2id(m_, mjOBJ_BODY, body_name.c_str());
  if (body_id < 0) {
    throw std::invalid_argument("body not found: " + body_name);
  }
  const double old_mass = m_->body_mass[body_id];
  if (!(old_mass > 0.0)) {
    throw std::runtime_error("cannot scale a body with zero source mass");
  }
  const double scale = mass / old_mass;
  m_->body_mass[body_id] = mass;
  for (int axis = 0; axis < 3; ++axis) {
    m_->body_inertia[3 * body_id + axis] *= scale;
  }
  mj_setConst(m_, d_);
  reset();
}

int MujocoBackend::dof() const { return m_->nu; }
double MujocoBackend::timestep() const { return m_->opt.timestep; }
double MujocoBackend::time() const { return d_->time; }

}  // namespace arm_control
