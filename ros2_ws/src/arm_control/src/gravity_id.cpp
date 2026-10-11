// Static gravity identification: Present_Current as τ, bidirectional holds,
// least-squares m from τ − g_empty(q) = Φ_g(q) · m.
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "arm_control/hardware_backend.hpp"
#include "arm_control/static_gravity_identifier.hpp"  // shared six-pose path

using arm_control::HardwareBackend;
using arm_control::StaticGravityIdentifier;

int main(int argc, char** argv) {
  HardwareBackend::Config cfg;
  cfg.calib_path = "so101_follower_calib.json";
  std::string payload_urdf = "models/so101/so101_dynamics_payload.urdf";
  bool empty = false;
  bool have_kt = false;
  double kt = 1.0;
  double mass_cal_scale = 1.0;
  double mass_cal_offset = 0.0;
  for (int i = 1; i < argc; ++i) {
    const std::string opt = argv[i];
    auto need = [&]() -> const char* {
      if (i + 1 >= argc) throw std::invalid_argument("missing value");
      return argv[++i];
    };
    if (opt == "--port") {
      cfg.port = need();
    } else if (opt == "--calib") {
      cfg.calib_path = need();
    } else if (opt == "--urdf") {
      cfg.urdf_path = need();
    } else if (opt == "--payload-urdf") {
      payload_urdf = need();
    } else if (opt == "--gripper-closed") {
      cfg.gripper_closed = true;
    } else if (opt == "--gripper-torque") {
      cfg.gripper_torque_limit = std::stoi(need());
    } else if (opt == "--empty") {
      empty = true;
    } else if (opt == "--kt") {
      kt = std::stod(need());
      have_kt = true;
    } else if (opt == "--mass-cal-scale") {
      mass_cal_scale = std::stod(need());
    } else if (opt == "--mass-cal-offset") {
      mass_cal_offset = std::stod(need());
    } else {
      std::fprintf(stderr,
                   "usage: gravity_id [--port PATH] [--calib FILE] [--urdf FILE] "
                   "[--payload-urdf FILE] [--gripper-closed] "
                   "[--gripper-torque 1-1000] [--empty] [--kt N.m/A] "
                   "[--mass-cal-scale S] [--mass-cal-offset KG]\n"
                   "  --empty  fit kt from g(q) vs Present_Current (no mass)\n"
                   "  default  estimate payload mass (pass --kt from --empty)\n");
      return 2;
    }
  }

  try {
    if (!empty) {
      if (!have_kt) {
        std::fprintf(stderr,
                     "warning: --kt omitted, using 1.0 N.m/A (run --empty first)\n");
      }
      for (int j = 0; j < 5; ++j) cfg.kt_nm_per_a[j] = kt;
    } else {
      for (int j = 0; j < 5; ++j) cfg.kt_nm_per_a[j] = 1.0;
    }
    cfg.kt_nm_per_a[5] = 0.0;

    HardwareBackend plant(cfg);
    plant.reset();
    StaticGravityIdentifier identifier(
        cfg.urdf_path, payload_urdf, StaticGravityIdentifier::Config{});
    StaticGravityIdentifier::Calibration calibration;
    calibration.raw_mass_scale = mass_cal_scale;
    calibration.raw_mass_offset = mass_cal_offset;

    std::printf(
        "gravity_id  %s  %d poses, approach ±%.2f rad, settle %.1f s\n",
        empty ? "fit kt (empty arm)" : "estimate mass", 6, 0.07, 1.2);
    const auto result = identifier.identify(plant, calibration);
    double kt_num[5] = {};
    double kt_den[5] = {};

    for (size_t p = 0; p < result.observations.size(); ++p) {
      const auto& observation = result.observations[p];
      std::printf(
          "pose %zu  q=(%.3f,%.3f,%.3f,%.3f,%.3f)  "
          "tau=(%.3f,%.3f,%.3f)  g=(%.3f,%.3f,%.3f)\n",
          p, observation.q[0], observation.q[1], observation.q[2],
          observation.q[3], observation.q[4],
          observation.measured_torque[1], observation.measured_torque[2],
          observation.measured_torque[3], observation.empty_gravity[1],
          observation.empty_gravity[2], observation.empty_gravity[3]);

      if (empty) {
        for (int j = 1; j <= 3; ++j) {
          kt_num[j] += observation.empty_gravity[j] *
                       observation.measured_torque[j];
          kt_den[j] += observation.measured_torque[j] *
                       observation.measured_torque[j];
        }
      }
    }

    if (empty) {
      double pooled_n = 0.0;
      double pooled_d = 0.0;
      const char* names[] = {"pan", "lift", "elbow", "flex", "roll"};
      std::printf("fitted kt [N.m/A] from g / (sign·I) at kt=1 proxy:\n");
      for (int j = 1; j <= 3; ++j) {
        const double ktj = (kt_den[j] > 1e-8) ? kt_num[j] / kt_den[j] : 0.0;
        pooled_n += kt_num[j];
        pooled_d += kt_den[j];
        std::printf("  %s  %.4f\n", names[j], ktj);
      }
      const double pooled = (pooled_d > 1e-8) ? pooled_n / pooled_d : 0.0;
      std::printf("pooled (lift+elbow+flex) kt=%.4f\n"
                  "rerun loaded: gravity_id --gripper-closed --kt %.4f\n",
                  pooled, pooled);
    } else {
      std::printf("static-gravity LS raw mass=%.4f kg, calibrated=%.4f kg  "
                  "(template 0.20 kg, kt=%.4f)\n"
                  "freeze for tracking: hardware_run ... --mass %.4f\n",
                  result.raw_mass, result.estimated_mass, kt,
                  result.estimated_mass);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
