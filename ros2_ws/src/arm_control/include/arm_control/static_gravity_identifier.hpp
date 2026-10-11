#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace arm_control {

class HardwareBackend;

class StaticGravityIdentifier {
public:
  struct Config {
    double move_duration_s = 2.0;
    double settle_duration_s = 1.2;
    double approach_rad = 0.07;
    int samples = 25;
    std::chrono::milliseconds sample_period{20};
    double quiet_velocity_threshold_rad_s = 0.05;
    int quiet_samples = 10;
    double quiet_timeout_s = 3.0;
    double reference_payload_mass = 0.20;
  };

  struct Calibration {
    Calibration(double scale = 1.0, double offset = 0.0,
                double minimum = 0.0, double maximum = 0.5)
        : raw_mass_scale(scale),
          raw_mass_offset(offset),
          min_mass(minimum),
          max_mass(maximum) {}

    double raw_mass_scale;
    double raw_mass_offset;
    double min_mass;
    double max_mass;
  };

  struct Observation {
    Eigen::VectorXd q;
    Eigen::VectorXd measured_torque;
    Eigen::VectorXd empty_gravity;
    Eigen::VectorXd payload_regressor;
    Eigen::VectorXd extra_torque;
  };

  struct Result {
    double raw_mass = 0.0;
    double estimated_mass = 0.0;
    std::vector<Observation> observations;
  };

  StaticGravityIdentifier(const std::string& empty_urdf_path,
                          const std::string& payload_urdf_path, Config config);

  Result identify(HardwareBackend& hardware, Calibration calibration) const;

private:
  void wait_until_quiet(HardwareBackend& hardware) const;
  bool arm_is_quiet(const Eigen::VectorXd& qdot) const;
  Eigen::VectorXd average_current(HardwareBackend& hardware) const;
  Eigen::VectorXd hold_current(HardwareBackend& hardware,
                               const Eigen::VectorXd& pose,
                               double approach_sign) const;

  Config config_;
  std::string empty_urdf_path_;
  std::string payload_urdf_path_;
};

}  // namespace arm_control
