// MuJoCo simulation backend. Loads the SO-101 model, reads qpos/qvel, writes
// d->ctrl, advances with mj_step, and reports the gripperframe site position.
// This is the only file that touches the MuJoCo C API.
#pragma once
#include <string>

#include "arm_control/plant_interface.hpp"

struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

namespace arm_control {

class MujocoBackend : public PlantInterface {
public:
  // Loads xml_path; caches the EE site and home-keyframe ids and resets to home.
  explicit MujocoBackend(const std::string& xml_path,
                         const std::string& ee_site = "gripperframe",
                         const std::string& keyframe = "home");
  ~MujocoBackend() override;

  MujocoBackend(const MujocoBackend&) = delete;
  MujocoBackend& operator=(const MujocoBackend&) = delete;

  void read_state(Eigen::VectorXd& q, Eigen::VectorXd& qdot) override;
  void apply_torque(const Eigen::VectorXd& tau) override;
  void step() override;
  Eigen::Vector3d ee_position() override;
  void reset() override;

  // Simulation-only pre-run utility. Scales a fixed payload body's mass and
  // principal inertia together, then refreshes MuJoCo's derived constants.
  void set_body_mass(const std::string& body_name, double mass);

  int dof() const override;
  double timestep() const override;
  double time() const override;

private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int ee_site_id_ = -1;
  int home_id_ = -1;
};

}  // namespace arm_control
