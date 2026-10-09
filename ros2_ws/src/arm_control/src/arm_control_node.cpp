// ROS 2 wrapper (step 4). The control loop runs on its OWN std::thread at a
// fixed rate and never touches rclcpp. Telemetry crosses to the non-RT side
// through a lock-free SPSC ring; a wall-timer on the executor thread drains it
// and publishes JointState + ArmMetrics. rclcpp::spin never touches the hot path
// -- exactly the isolation Phase 3's RT thread needs.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#endif
#include <array>
#include <atomic>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_control/adaptive_computed_torque_controller.hpp"
#include "arm_control/arm_types.hpp"
#include "arm_control/computed_torque_controller.hpp"
#include "arm_control/control_loop.hpp"
#include "arm_control/controller.hpp"
#include "arm_control/mujoco_backend.hpp"
#include "arm_control/feetech_bus.hpp"
#include "arm_control/hardware_backend.hpp"
#include "arm_control/pd_controller.hpp"
#include "arm_control/pinocchio_dynamics.hpp"
#include "arm_control/so101_calib.hpp"
#include "arm_control/rt_thread.hpp"
#include "arm_control/spsc_ring.hpp"
#include "arm_msgs/msg/arm_metrics.hpp"

using namespace std::chrono_literals;

namespace {
const std::vector<std::string> kJointNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper"};
constexpr int kArmJoints = 5;  // first 5 are the arm; index 5 is the held gripper
constexpr double kReferenceDuration = 1.0;
// Project bring-up settings, not servo datasheet limits.
constexpr double kHardwareRateHz = 200.0;
constexpr double kMaxLeadQRad = 0.12;
constexpr int kHardwareMaxBusFails = 3;
// STS3215 Goal_Speed (address 46) is 0–3400 steps/s. 0 means unlimited.
// Unlimited is rejected so the position bridge keeps a finite speed bound.
constexpr int kGoalSpeedMin = 1;
constexpr int kGoalSpeedMax = 3400;
// Worst-case rx+tx waits must leave this much of the period for the controller.
constexpr int64_t kMinControllerBudgetNs = 1000000;
constexpr int kTickMax = 4095;
constexpr double kRadPerTick = 2.0 * M_PI / 4096.0;

Eigen::VectorXd to_eigen(const std::vector<double>& v) {
  Eigen::VectorXd e(v.size());
  for (size_t i = 0; i < v.size(); ++i) e[i] = v[i];
  return e;
}

void minimum_jerk_reference(double time, const Eigen::VectorXd& target,
                            Eigen::VectorXd& q, Eigen::VectorXd& qdot,
                            Eigen::VectorXd& qddot) {
  const double s = std::clamp(time / kReferenceDuration, 0.0, 1.0);
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  q = (10.0 * s3 - 15.0 * s4 + 6.0 * s5) * target;
  qdot = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) * target /
         kReferenceDuration;
  qddot = (60.0 * s - 180.0 * s2 + 120.0 * s3) * target /
          (kReferenceDuration * kReferenceDuration);
}

bool all_finite(const std::vector<double>& v) {
  return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
}

// Params are repo-root relative (models/so101/...). Tests and launches start
// from other directories, so walk parents until the file is found.
std::string resolve_repo_path(const std::string& path) {
  const std::filesystem::path given(path);
  if (given.is_absolute()) return path;
  std::filesystem::path dir = std::filesystem::current_path();
  for (;;) {
    const std::filesystem::path candidate = dir / given;
    if (std::filesystem::exists(candidate)) {
      return std::filesystem::canonical(candidate).string();
    }
    const std::filesystem::path parent = dir.parent_path();
    if (parent == dir) break;
    dir = parent;
  }
  return path;
}

bool validate_gains(const std::vector<double>& v) {
  const bool is_valid_dimension = v.size() == arm_control::kDof;
  const bool is_finite = all_finite(v);
  const bool is_all_non_negative = std::all_of(v.begin(), v.end(), [](double x) { return x >= 0.0; });
  return is_valid_dimension && is_finite && is_all_non_negative;
}

// validate_gains() allows zero. The torque bridge divides by k_servo.
bool strictly_positive(const std::vector<double>& v) {
  return v.size() == arm_control::kDof && all_finite(v) &&
         std::all_of(v.begin(), v.end(), [](double x) { return x > 0.0; });
}

// Arm joints enter the payload regressor. Gripper current does not, so its
// kt may be zero under the current calibration convention.
bool valid_kt(const std::vector<double>& kt) {
  if (!validate_gains(kt)) return false;
  return std::all_of(kt.begin(), kt.begin() + kArmJoints,
                     [](double x) { return x > 0.0; });
}

bool frozen_while_configured(const std::string& name) {
  return name == "is_sim" || name == "model_path" || name == "urdf_path" ||
         name == "rate_hz" || name.rfind("hardware.", 0) == 0;
}

void validate_joint_calib(
    const std::array<arm_control::JointCalib, arm_control::kDof>& calib) {
  std::set<int> ids;
  for (int i = 0; i < arm_control::kDof; ++i) {
    const auto& c = calib[static_cast<size_t>(i)];
    const std::string& name = kJointNames[static_cast<size_t>(i)];
    if (c.id < 0 || c.id > 253 || !ids.insert(c.id).second) {
      throw std::invalid_argument(
          name + " servo id must be unique and in 0..253");
    }
    if (c.sign != 1 && c.sign != -1) {
      throw std::invalid_argument(name + " sign must be +1 or -1");
    }
    if (c.min_ticks < 0 || c.max_ticks > kTickMax || c.min_ticks >= c.max_ticks) {
      throw std::invalid_argument(
          name + " tick limits must satisfy 0 <= min < max <= 4095");
    }
    if (c.zero_ticks < 0 || c.zero_ticks > kTickMax) {
      throw std::invalid_argument(name + " zero_ticks must be in 0..4095");
    }
  }
}

void require_in_joint_limits(const char* what, int joint, double q,
                             const arm_control::JointCalib& c) {
  const double q_min =
      c.sign * (static_cast<double>(c.min_ticks) - c.zero_ticks) * kRadPerTick;
  const double q_max =
      c.sign * (static_cast<double>(c.max_ticks) - c.zero_ticks) * kRadPerTick;
  const double lo = std::min(q_min, q_max);
  const double hi = std::max(q_min, q_max);
  if (!(std::isfinite(q) && q >= lo && q <= hi)) {
    throw std::invalid_argument(
        std::string(what) + " " + kJointNames[static_cast<size_t>(joint)] +
        " is outside calibrated joint limits");
  }
}

}  // namespace

class ArmControlNode : public rclcpp_lifecycle::LifecycleNode {
public:
  ArmControlNode() : rclcpp_lifecycle::LifecycleNode("arm_control_node"), ring_(1024) {
    // --- parameters (gains / target / model / rate) ---
    const bool is_sim = declare_parameter<bool>("is_sim", true);
    const std::string model_path = declare_parameter<std::string>(
        "model_path", "models/so101/scene_torque.xml");
    const std::string controller_type =
        declare_parameter<std::string>("controller_type", "pd");
    const std::string urdf_path = declare_parameter<std::string>(
        "urdf_path", "models/so101/so101_dynamics.urdf");
    const std::string payload_urdf_path = declare_parameter<std::string>(
        "payload_urdf_path",
        "models/so101/so101_dynamics_payload.urdf");
    const double reference_payload_mass =
        declare_parameter<double>("reference_payload_mass", 0.20);
    const double plant_payload_mass =
        declare_parameter<double>("plant_payload_mass", -1.0);
    reference_type_ =
        declare_parameter<std::string>("reference_type", "smooth");
    rate_hz_ = declare_parameter<double>("rate_hz", 200.0);
    rt_enable_ = declare_parameter<bool>("rt_enable", true);
    rt_priority_ = declare_parameter<int>("rt_priority", 80);
    rt_cpu_ = declare_parameter<int>("rt_cpu", -1);
    jitter_samples_ = declare_parameter<int>("jitter_samples", 60000);
    worker_counters_ = declare_parameter<bool>("worker_counters", false);
    if (worker_counters_) {
      declare_parameter<int>("active_workers", 0);
      declare_parameter<int>("max_workers", 0);
    }
    jitter_csv_ = declare_parameter<std::string>("jitter_csv", "");
    const auto kp = declare_parameter<std::vector<double>>(
        "kp", is_sim
        ? std::vector<double>{40.0, 40.0, 25.0, 15.0, 8.0, 5.0}
        : std::vector<double>{8.0, 12.0, 2.0, 8.0, 8.0, 0.0});
    if (!validate_gains(kp)) {
      throw std::invalid_argument("Invalid kp gains");
    }
    const auto kd = declare_parameter<std::vector<double>>(
        "kd", is_sim
        ? std::vector<double>{3.0, 3.0, 2.0, 1.0, 0.6, 0.4}
        : std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!validate_gains(kd)) {
      throw std::invalid_argument("Invalid kd gains");
    }

    const auto computed_kp = declare_parameter<std::vector<double>>(
        "computed_kp", is_sim
        ? std::vector<double>{400.0, 400.0, 400.0, 400.0, 400.0, 400.0}
        : std::vector<double>{40.0, 40.0, 40.0, 40.0, 40.0, 40.0});
    if (!validate_gains(computed_kp)) {
      throw std::invalid_argument("Invalid computed_kp gains");
    }

    const auto computed_kd = declare_parameter<std::vector<double>>(
        "computed_kd", is_sim
        ? std::vector<double>{40.0, 40.0, 40.0, 40.0, 40.0, 40.0}
        : std::vector<double>{4.0, 4.0, 4.0, 4.0, 4.0, 4.0});
    if (!validate_gains(computed_kd)) {
      throw std::invalid_argument("Invalid computed_kd gains");
    }

    if (!is_sim) {
      declare_hardware_parameters();
    }

    arm_control::PayloadMassRlsEstimator::Config estimator_config;
    estimator_config.initial_mass =
        declare_parameter<double>("rls_initial_mass", 0.0);
    estimator_config.initial_covariance =
        declare_parameter<double>("rls_initial_covariance", 100.0);
    estimator_config.forgetting_factor =
        declare_parameter<double>("rls_forgetting_factor", 1.0);
    estimator_config.max_mass =
        declare_parameter<double>("rls_max_mass", 0.5);
    estimator_config.excitation_threshold =
        declare_parameter<double>("rls_excitation_threshold", 1e-8);
    target_ = declare_parameter<std::vector<double>>(
        "target", {0.6, 0.7, -0.8, 0.5, 0.4, 0.0});

    // register the on-set parameters callback
    on_set_parameters_callback_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) { return on_set_parameters_callback(parameters); });

    // register the post-set parameters callback
    post_set_parameters_callback_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) { return on_post_set_parameters_callback(parameters); });
  }

  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    try {
      const bool is_sim = get_parameter("is_sim").get_value<bool>();
      const double rate_hz = get_parameter("rate_hz").get_value<double>();
      if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
        throw std::invalid_argument("Invalid rate: " + std::to_string(rate_hz));
      }
      if (!is_sim && rate_hz != kHardwareRateHz) {
        throw std::invalid_argument(
            "hardware rate_hz must be the validated project rate of 200 Hz");
      }

      const std::string reference_type = get_parameter("reference_type").get_value<std::string>();
      if (reference_type != "step" && reference_type != "smooth") {
        throw std::invalid_argument("Invalid reference type: " + reference_type);
      }

      std::vector<double> target = get_parameter("target").get_value<std::vector<double>>();
      if (target.size() != arm_control::kDof || !all_finite(target)) {
        throw std::invalid_argument("target must be six finite values");
      }

      const std::vector<double> kp = get_parameter("kp").get_value<std::vector<double>>();
      const std::vector<double> kd = get_parameter("kd").get_value<std::vector<double>>();
      const std::vector<double> computed_kp = get_parameter("computed_kp").get_value<std::vector<double>>();
      const std::vector<double> computed_kd = get_parameter("computed_kd").get_value<std::vector<double>>();
      if (!validate_gains(kp) || !validate_gains(kd) || !validate_gains(computed_kp) || !validate_gains(computed_kd)) {
        throw std::invalid_argument(
            "controller gains must be six finite, nonnegative values");
      }

      const std::string controller_type = get_parameter("controller_type").get_value<std::string>();
      if (controller_type != "pd" && controller_type != "computed_torque" && controller_type != "adaptive_computed_torque") {
        throw std::invalid_argument("Invalid controller type: " + controller_type);
      }

      bool rt_enable = get_parameter("rt_enable").get_value<bool>();
      int rt_priority = get_parameter("rt_priority").get_value<int>();
      int rt_cpu = get_parameter("rt_cpu").get_value<int>();
      int jitter_samples = get_parameter("jitter_samples").get_value<int>();
      std::string jitter_csv = get_parameter("jitter_csv").get_value<std::string>();

      // --- build the control core ---
      const std::string model_path = resolve_repo_path(
          get_parameter("model_path").get_value<std::string>());
      const std::string urdf_path = resolve_repo_path(
          get_parameter("urdf_path").get_value<std::string>());
      const std::string payload_urdf_path = resolve_repo_path(
          get_parameter("payload_urdf_path").get_value<std::string>());

      std::unique_ptr<arm_control::PlantInterface> plant;
      if (is_sim) {
        plant = std::make_unique<arm_control::MujocoBackend>(model_path);
        const double plant_payload_mass = get_parameter("plant_payload_mass").get_value<double>();
        if (plant_payload_mass >= 0.0) {
          static_cast<arm_control::MujocoBackend*>(plant.get())->set_body_mass("known_payload", plant_payload_mass);
        }
      } else {
        const auto hardware_config = make_hardware_config(rate_hz, urdf_path, target);
        plant = std::make_unique<arm_control::HardwareBackend>(hardware_config);
      }

      const double reference_payload_mass = get_parameter("reference_payload_mass").get_value<double>();
      arm_control::PayloadMassRlsEstimator::Config estimator_config;
      estimator_config.initial_mass = get_parameter("rls_initial_mass").get_value<double>();
      estimator_config.initial_covariance = get_parameter("rls_initial_covariance").get_value<double>();
      estimator_config.forgetting_factor = get_parameter("rls_forgetting_factor").get_value<double>();
      estimator_config.max_mass = get_parameter("rls_max_mass").get_value<double>();
      estimator_config.excitation_threshold = get_parameter("rls_excitation_threshold").get_value<double>();

      std::unique_ptr<arm_control::Controller> controller;
      if (controller_type == "pd") {
        controller =
            std::make_unique<arm_control::PdController>(to_eigen(kp), to_eigen(kd));
      } else if (controller_type == "computed_torque") {
        controller = std::make_unique<arm_control::ComputedTorqueController>(
            urdf_path, to_eigen(computed_kp), to_eigen(computed_kd));
      } else if (controller_type == "adaptive_computed_torque") {
        controller =
            std::make_unique<arm_control::AdaptiveComputedTorqueController>(
                urdf_path, payload_urdf_path, reference_payload_mass,
                to_eigen(computed_kp), to_eigen(computed_kd),
                plant->timestep(), estimator_config);
      }

      Eigen::VectorXd target_eigen = to_eigen(target);
      auto loop = std::make_unique<arm_control::ControlLoop>(*plant, *controller, target_eigen);
      loop->reset();

      // --- publishers (non-RT side) ---
      auto joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      auto metrics_pub = create_publisher<arm_msgs::msg::ArmMetrics>("arm_metrics", 10);

      rt_enable_ = rt_enable;
      rt_priority_ = rt_priority;
      rt_cpu_ = rt_cpu;
      jitter_samples_ = jitter_samples;
      jitter_csv_ = std::move(jitter_csv);

      target_ = std::move(target);
      target_eigen_ = std::move(target_eigen);
      q_ref_ = Eigen::VectorXd::Zero(arm_control::kDof);
      qdot_ref_ = Eigen::VectorXd::Zero(arm_control::kDof);
      qddot_ref_ = Eigen::VectorXd::Zero(arm_control::kDof);

      rate_hz_ = rate_hz;
      reference_type_ = reference_type;
      plant_ = std::move(plant);
      controller_ = std::move(controller);
      loop_ = std::move(loop);

      joint_pub_ = std::move(joint_pub);
      metrics_pub_ = std::move(metrics_pub);

      RCLCPP_INFO(get_logger(),
                  "arm_control_node configured: is_sim=%d model=%s controller=%s rate=%.0f Hz",
                  is_sim, model_path.c_str(), controller_type.c_str(), rate_hz_);
    } catch (const std::invalid_argument& e) {
      RCLCPP_ERROR(get_logger(), "Configuration error: %s", e.what());
      return LifecycleNodeInterface::CallbackReturn::FAILURE;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Unexpected error in configuration: %s", e.what());
      return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
    if (worker_failed_.load(std::memory_order_acquire)) {
      RCLCPP_ERROR(get_logger(),
                   "Control loop fault is still set (%s); cleanup before activating",
                   worker_error_.c_str());
      return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    if (control_thread_.joinable()) {
      RCLCPP_ERROR(get_logger(), "Control thread already running");
      return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // check if current kp and kd are valid if controller is pd
    const std::string controller_type = get_parameter("controller_type").get_value<std::string>();
    if (controller_type == "pd") {
      const std::vector<double> current_kp = get_parameter("kp").get_value<std::vector<double>>();
      const std::vector<double> current_kd = get_parameter("kd").get_value<std::vector<double>>();
      const auto* pd = dynamic_cast<arm_control::PdController*>(controller_.get());
      if (pd == nullptr) {
        RCLCPP_ERROR(get_logger(),
                     "Invariant failure: controller_type is pd but controller is not PdController");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      const auto controller_gains = pd->get_gains();
      if (controller_gains.first != to_eigen(current_kp) || controller_gains.second != to_eigen(current_kd)) {
        RCLCPP_ERROR(get_logger(), "Controller gains have changed since last activation");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }

    try {
      auto result = LifecycleNode::on_activate(state);
      if (result != LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return result;
      }
      // --- non-RT drain+publish timer on the executor thread ---
      publish_timer_ = create_wall_timer(20ms, [this]() {
        mirror_worker_counters();
        drain_and_publish();
        log_dropped_samples();
        log_worker_fault();
      });
      // --- start the control thread (this is the fixed-rate loop) ---
      dropped_count_.store(0, std::memory_order_relaxed);
      previous_dropped_count_ = 0;
      max_workers_.store(0, std::memory_order_relaxed);
      running_.store(true, std::memory_order_release);
      control_thread_ = std::thread([this]() { control_loop(); });
    } catch (const std::exception& e) {
      running_.store(false, std::memory_order_release);
      if (control_thread_.joinable()) {
        control_thread_.join();
      }
      publish_timer_.reset();
      // deactivate the publishers
      joint_pub_->on_deactivate();
      metrics_pub_->on_deactivate();

      RCLCPP_ERROR(get_logger(), "Unexpected error in activation: %s", e.what());
      return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // reactivation is implicitly resuming from the last state.
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
    publish_timer_.reset();

    running_.store(false, std::memory_order_release);
    if (control_thread_.joinable()) {
      control_thread_.join();
      dropped_count_.store(0, std::memory_order_relaxed);
      previous_dropped_count_ = 0;
    }
    mirror_worker_counters();
    // discard the ring so that its empty for the next activation
    arm_control::Sample s;
    while (ring_.pop(s)) { /* do nothing */ }

    auto result = LifecycleNode::on_deactivate(state);
    if (result != LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return result;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override {
    auto result = LifecycleNode::on_cleanup(state);
    if (result != LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return result;
    }
    release_configured_resources();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // A failed activate while a worker fault is set ends in error processing.
  // Release the configured backend so the node does not stay unconfigured
  // with the hardware still held.
  LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
    RCLCPP_ERROR(get_logger(), "Lifecycle error; releasing the configured backend");
    if (joint_pub_ && joint_pub_->is_activated()) {
      joint_pub_->on_deactivate();
    }
    if (metrics_pub_ && metrics_pub_->is_activated()) {
      metrics_pub_->on_deactivate();
    }
    release_configured_resources();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // @todo: add on_shutdown()

  ~ArmControlNode() override {
    running_.store(false, std::memory_order_release);
    if (control_thread_.joinable()) control_thread_.join();
  }

private:
  // Validates hardware parameters, then opens the port. Called only from
  // on_configure(), before HardwareBackend is constructed.
  arm_control::HardwareBackend::Config make_hardware_config(
      double rate_hz, const std::string& urdf_path,
      const std::vector<double>& target) {
    arm_control::HardwareBackend::Config cfg;
    cfg.dt = 1.0 / rate_hz;
    cfg.urdf_path = urdf_path;

    cfg.port = get_parameter("hardware.port").get_value<std::string>();
    if (cfg.port.empty()) {
      throw std::invalid_argument("hardware.port must be a nonempty string");
    }
    cfg.baud = get_parameter("hardware.baud").get_value<int>();
    if (!arm_control::FeetechBus::baud_supported(cfg.baud)) {
      throw std::invalid_argument(
          "hardware.baud is not supported by the bus (115200 or 1000000)");
    }

    cfg.calib_path = resolve_repo_path(
        get_parameter("hardware.calib_path").get_value<std::string>());
    if (!std::filesystem::is_regular_file(cfg.calib_path)) {
      throw std::invalid_argument(
          "hardware.calib_path does not exist: " + cfg.calib_path);
    }
    std::array<arm_control::JointCalib, arm_control::kDof> calib{};
    try {
      calib = arm_control::load_so101_calib(cfg.calib_path);
    } catch (const std::exception& e) {
      throw std::invalid_argument(
          std::string("hardware.calib_path: ") + e.what());
    }
    validate_joint_calib(calib);

    if (!std::filesystem::is_regular_file(urdf_path)) {
      throw std::invalid_argument("urdf_path does not exist: " + urdf_path);
    }
    try {
      arm_control::PinocchioDynamics{urdf_path};
    } catch (const std::exception& e) {
      throw std::invalid_argument(std::string("urdf_path: ") + e.what());
    }

    const auto k_servo = get_parameter("hardware.bridge.k_servo")
                             .get_value<std::vector<double>>();
    if (!strictly_positive(k_servo)) {
      throw std::invalid_argument(
          "hardware.bridge.k_servo must be six finite, strictly positive values");
    }
    std::copy(k_servo.begin(), k_servo.end(), cfg.k_servo.begin());

    cfg.max_lead_q =
        get_parameter("hardware.bridge.max_lead_q").get_value<double>();
    if (!(std::isfinite(cfg.max_lead_q) && cfg.max_lead_q > 0.0 &&
          cfg.max_lead_q <= kMaxLeadQRad)) {
      throw std::invalid_argument(
          "hardware.bridge.max_lead_q must be finite and in (0, 0.12] rad");
    }

    cfg.goal_speed =
        get_parameter("hardware.bridge.goal_speed").get_value<int>();
    if (cfg.goal_speed < kGoalSpeedMin || cfg.goal_speed > kGoalSpeedMax) {
      throw std::invalid_argument(
          "hardware.bridge.goal_speed must be in [1, 3400]; 0 (unlimited) is not allowed");
    }

    const int rx_timeout_ns =
        get_parameter("hardware.bus.rx_timeout_ns").get_value<int>();
    const int tx_timeout_ns =
        get_parameter("hardware.bus.tx_timeout_ns").get_value<int>();
    if (rx_timeout_ns <= 0 || tx_timeout_ns <= 0) {
      throw std::invalid_argument("hardware bus timeouts must be positive");
    }
    const int64_t period_ns =
        static_cast<int64_t>(std::llround(1e9 / rate_hz));
    const int64_t io_ns = static_cast<int64_t>(rx_timeout_ns) +
                          static_cast<int64_t>(tx_timeout_ns);
    if (io_ns + kMinControllerBudgetNs > period_ns) {
      throw std::invalid_argument(
          "hardware bus timeouts leave no time for controller computation within the period");
    }
    cfg.rx_timeout_ns = rx_timeout_ns;
    cfg.tx_timeout_ns = tx_timeout_ns;

    cfg.max_bus_fails =
        get_parameter("hardware.bus.max_bus_fails").get_value<int>();
    if (cfg.max_bus_fails != kHardwareMaxBusFails) {
      throw std::invalid_argument("hardware.bus.max_bus_fails must be 3");
    }

    cfg.home_duration =
        get_parameter("hardware.home_duration").get_value<double>();
    if (!(std::isfinite(cfg.home_duration) && cfg.home_duration > 0.0)) {
      throw std::invalid_argument(
          "hardware.home_duration must be finite and positive");
    }

    cfg.gripper_closed =
        get_parameter("hardware.gripper_closed").get_value<bool>();
    cfg.gripper_q = get_parameter("hardware.gripper_q").get_value<double>();
    if (!cfg.gripper_closed) {
      require_in_joint_limits("hardware.gripper_q", 5, cfg.gripper_q, calib[5]);
    }

    cfg.gripper_torque_limit =
        get_parameter("hardware.gripper_torque_limit").get_value<int>();
    if (cfg.gripper_torque_limit < 1 || cfg.gripper_torque_limit > 1000) {
      throw std::invalid_argument(
          "hardware.gripper_torque_limit must be in [1, 1000]");
    }

    cfg.current_lsb_a =
        get_parameter("hardware.current_lsb_a").get_value<double>();
    if (!(std::isfinite(cfg.current_lsb_a) && cfg.current_lsb_a > 0.0)) {
      throw std::invalid_argument(
          "hardware.current_lsb_a must be finite and strictly positive");
    }

    const auto kt_nm_per_a = get_parameter("hardware.kt_nm_per_a")
                                 .get_value<std::vector<double>>();
    if (!valid_kt(kt_nm_per_a)) {
      throw std::invalid_argument(
          "hardware.kt_nm_per_a must be six finite, nonnegative values, "
          "positive on the arm joints used for payload identification");
    }
    std::copy(kt_nm_per_a.begin(), kt_nm_per_a.end(), cfg.kt_nm_per_a.begin());

    for (int i = 0; i < arm_control::kDof; ++i) {
      require_in_joint_limits("target", i, target[static_cast<size_t>(i)],
                              calib[static_cast<size_t>(i)]);
    }

    try {
      arm_control::FeetechBus probe;
      probe.open(cfg.port, cfg.baud);
    } catch (const std::exception& e) {
      throw std::invalid_argument(std::string("hardware.port: ") + e.what());
    }
    return cfg;
  }

  // Runs on its own thread. Fixed-rate; RT-clean body (no alloc, no rclcpp).
  void control_loop() {
    const int active = active_workers_.fetch_add(1, std::memory_order_acq_rel) + 1;
    int seen = max_workers_.load(std::memory_order_relaxed);
    while (active > seen && !max_workers_.compare_exchange_weak( seen, active, std::memory_order_relaxed)) {}
    struct StopWorker {
      std::atomic<int>& active;
      ~StopWorker() { active.fetch_sub(1, std::memory_order_acq_rel); }
    } stop{active_workers_};

    try {
      if (rt_enable_) {
        arm_control::RtConfig cfg;
        cfg.fifo_priority = rt_priority_;
        cfg.cpu_affinity = rt_cpu_;
        const arm_control::RtStatus rt = arm_control::configure_rt_thread(cfg);
        std::fprintf(stderr,
                     "rt: mlockall=%d fifo=%d affinity=%d cstates=%d\n",
                     rt.memory_locked, rt.fifo_set, rt.affinity_set,
                     rt.cstates_suppressed);
        if (!rt.error.empty()) {
          std::fprintf(stderr, "rt warnings: %s\n", rt.error.c_str());
        }
      }

      const size_t jitter_cap =
          jitter_samples_ > 0 ? static_cast<size_t>(jitter_samples_) : 0;
      std::vector<int64_t> loop_late_ns(jitter_cap);
      std::vector<int64_t> wake_late_ns(jitter_cap);
      size_t jitter_n = 0;
      int64_t max_loop_late_ns = 0;
      int64_t max_wake_late_ns = 0;
      uint64_t cycles = 0;

      const auto period =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(1.0 / rate_hz_));
      const auto origin = std::chrono::steady_clock::now();
      auto next = origin;
      auto* hardware = dynamic_cast<arm_control::HardwareBackend*>(plant_.get());
      arm_control::Sample s;  // reused; no per-iteration allocation
      while (running_.load(std::memory_order_acquire)) {
        if (hardware) {
          hardware->set_time(
              std::chrono::duration<double>(next - origin).count());
        }
        if (reference_type_ == "smooth") {
          minimum_jerk_reference(plant_->time(), target_eigen_, q_ref_,
                                 qdot_ref_, qddot_ref_);
          loop_->set_reference(q_ref_, qdot_ref_, qddot_ref_);
        }
        loop_->step_once(s);
        if (!ring_.push(s)) {
          dropped_count_.fetch_add(1, std::memory_order_relaxed);
        }
        ++cycles;
        const auto work_done = std::chrono::steady_clock::now();
        const auto deadline = next + period;
        const int64_t loop_late = std::chrono::duration_cast<std::chrono::nanoseconds>(work_done - deadline).count();
        if (loop_late > max_loop_late_ns) max_loop_late_ns = loop_late;
        next = deadline;
        // Missed deadline: jump to the next one still in the future.
        if (next <= work_done) {
          const auto skips = (work_done - next) / period + 1;
          next += skips * period;
        }
        arm_control::sleep_until_monotonic(next);
        const auto woke = std::chrono::steady_clock::now();
        const int64_t wake_late =
            std::chrono::duration_cast<std::chrono::nanoseconds>(woke - next)
                .count();
        if (wake_late > max_wake_late_ns) max_wake_late_ns = wake_late;
        if (jitter_n < jitter_cap) {
          loop_late_ns[jitter_n] = loop_late;
          wake_late_ns[jitter_n] = wake_late;
          ++jitter_n;
        }
      }

      if (cycles == 0) return;
      auto report = [](const char* name, const std::vector<int64_t>& samples,
                       size_t n, int64_t run_max_ns) {
        if (n == 0) {
          std::fprintf(stderr, "%s: n=0 run max: %.3f (us)\n", name,
                       run_max_ns / 1000.0);
          return;
        }
        int64_t min_ns = samples[0];
        int64_t max_ns = samples[0];
        long double sum = 0;
        for (size_t i = 0; i < n; ++i) {
          if (samples[i] < min_ns) min_ns = samples[i];
          if (samples[i] > max_ns) max_ns = samples[i];
          sum += static_cast<long double>(samples[i]);
        }
        std::fprintf(stderr,
                     "%s: n=%zu Min: %.3f Avg: %.3f Max: %.3f  run max: %.3f "
                     "(us)\n",
                     name, n, min_ns / 1000.0,
                     static_cast<double>(sum / n) / 1000.0, max_ns / 1000.0,
                     run_max_ns / 1000.0);
      };
      report("control loop lateness", loop_late_ns, jitter_n, max_loop_late_ns);
      report("wakeup jitter", wake_late_ns, jitter_n, max_wake_late_ns);
      if (jitter_csv_.empty() || jitter_n == 0) return;
      std::ofstream out(jitter_csv_);
      if (!out) {
        std::fprintf(stderr, "failed to write %s\n", jitter_csv_.c_str());
        return;
      }
      out << "# rate_hz=" << rate_hz_ << "\n";
      out << "i,loop_late_us,wake_late_us\n";
      out.precision(9);
      for (size_t i = 0; i < jitter_n; ++i) {
        out << i << ',' << loop_late_ns[i] / 1000.0 << ','
            << wake_late_ns[i] / 1000.0 << '\n';
      }
      std::fprintf(stderr, "wrote %s\n", jitter_csv_.c_str());
    } catch (const std::exception& e) {
      worker_error_ = e.what();
      worker_failed_.store(true, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      std::fprintf(stderr, "control loop stopped: %s\n", e.what());
    } catch (...) {
      worker_error_ = "unknown exception";
      worker_failed_.store(true, std::memory_order_release);
      running_.store(false, std::memory_order_release);
      std::fprintf(stderr, "control loop stopped: unknown exception\n");
    }
  }

  // Runs on the executor (non-RT) thread. Drains the ring and publishes the
  // most recent sample as JointState + ArmMetrics.
  void drain_and_publish() {
    arm_control::Sample s;
    bool got = false;
    while (ring_.pop(s)) got = true;  // keep only the latest
    if (!got) return;

    const auto stamp = now();

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = kJointNames;
    js.position.assign(s.q.begin(), s.q.end());
    js.velocity.assign(s.qd.begin(), s.qd.end());
    js.effort.assign(s.tau.begin(), s.tau.end());
    joint_pub_->publish(js);

    arm_msgs::msg::ArmMetrics m;
    m.header.stamp = stamp;
    m.ee_position = {s.ee[0], s.ee[1], s.ee[2]};
    m.joint_error.resize(arm_control::kDof);
    double sumsq = 0.0;
    for (int i = 0; i < arm_control::kDof; ++i) {
      const double err = target_[i] - s.q[i];
      m.joint_error[i] = err;
      if (i < kArmJoints) sumsq += err * err;
    }
    m.arm_rms_error = std::sqrt(sumsq / kArmJoints);
    m.estimated_payload_mass = s.estimated_payload_mass;
    m.sample_time = s.t;
    metrics_pub_->publish(m);
  }

  // Executor thread. The control thread stores the message, then sets the flag.
  // The transition runs on a later timer tick so this callback is not inside
  // on_deactivate while that function resets the publish timer.
  void log_worker_fault() {
    if (!worker_failed_.load(std::memory_order_acquire) || worker_fault_logged_) {
      return;
    }
    worker_fault_logged_ = true;
    RCLCPP_ERROR(get_logger(), "Control loop stopped: %s", worker_error_.c_str());
    RCLCPP_ERROR(get_logger(), "Deactivating; cleanup before activating again");
    fault_action_timer_ = create_wall_timer(1ms, [this]() {
      fault_action_timer_.reset();
      handle_worker_fault();
    });
  }

  void handle_worker_fault() {
    try {
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to leave active after the control-loop fault: %s", e.what());
    }
  }

  void release_configured_resources() {
    loop_.reset();
    controller_.reset();
    plant_.reset();
    target_.clear();
    target_eigen_.resize(0);
    q_ref_.resize(0);
    qdot_ref_.resize(0);
    qddot_ref_.resize(0);
    joint_pub_.reset();
    metrics_pub_.reset();
    publish_timer_.reset();
    fault_action_timer_.reset();
    worker_failed_.store(false, std::memory_order_relaxed);
    worker_fault_logged_ = false;
    worker_error_.clear();
  }

  /// Executor thread diagnostics. Logs a warning if the number of dropped samples changes.
  void log_dropped_samples() {
    const int dropped = dropped_count_.load(std::memory_order_relaxed);
    if (dropped == 0) return;

    // log only when a value change is observed
    if (dropped != previous_dropped_count_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Dropped %d telemetry samples since activation", dropped);
      previous_dropped_count_ = dropped;
    }
  }

  // Executor thread only. The control thread updates the atomics and never
  // touches rclcpp; tests read the mirrored parameters.
  void mirror_worker_counters() {
    if (!worker_counters_) return;
    set_parameter(rclcpp::Parameter(
        "active_workers", active_workers_.load(std::memory_order_acquire)));
    set_parameter(rclcpp::Parameter(
        "max_workers", max_workers_.load(std::memory_order_relaxed)));
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    const bool unconfigured =
        get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

    std::vector<double> current_kp = get_parameter("kp").get_value<std::vector<double>>();
    std::vector<double> current_kd = get_parameter("kd").get_value<std::vector<double>>();
    for (const auto &param : parameters) {
      if (!unconfigured && frozen_while_configured(param.get_name())) {
        result.successful = false;
        result.reason = param.get_name() + " cannot change while configured";
        break;
      }
      if (param.get_name() == "kp" || param.get_name() == "kd") {
        // kp or kd parameter is only allowed to be set when node is inactive
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          result.successful = false;
          result.reason = "Node is not inactive";
          break;
        }

        // fail if controller is not pd
        const std::string controller_type = get_parameter("controller_type").get_value<std::string>();
        if (controller_type != "pd") {
          result.successful = false;
          result.reason = "Controller is not pd";
          break;
        }

        // update the current kp or kd
        if (param.get_name() == "kp") {
          current_kp = param.get_value<std::vector<double>>();
        } else if (param.get_name() == "kd") {
          current_kd = param.get_value<std::vector<double>>();
        }

        // validate both kp and kd
        // return false if dimension is not kdof, not finite values, or not all non-negative
        if (!validate_gains(current_kp) || !validate_gains(current_kd)) {
          result.successful = false;
          result.reason = "Invalid gains";
          break;
        }
      }

      // param controller_type is only allowed to be set when node is unconfigured
      if (param.get_name() == "controller_type") {
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          result.successful = false;
          result.reason = "Controller type is only allowed to be set when node is unconfigured";
          break;
        }
      }
    }
    return result;
  }

  void on_post_set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &param : parameters) {
      if (param.get_name() != "kp" && param.get_name() != "kd") {
        continue;
      }
      const std::string controller_type = get_parameter("controller_type").get_value<std::string>();
      const std::vector<double> current_kp = get_parameter("kp").get_value<std::vector<double>>();
      const std::vector<double> current_kd = get_parameter("kd").get_value<std::vector<double>>();
      if (controller_type == "pd") {
        auto* pd = dynamic_cast<arm_control::PdController*>(controller_.get());
        if (pd == nullptr) {
          RCLCPP_ERROR(get_logger(),
                       "Invariant failure: controller_type is pd but controller is not PdController");
          return;
        }
        pd->set_gains(to_eigen(current_kp), to_eigen(current_kd));
      }
    }
  }

  inline void declare_hardware_parameters() {
    declare_parameter<std::string>("hardware.port", "/dev/ttyACM0");
    declare_parameter<int>("hardware.baud", 1000000);
    declare_parameter<std::string>("hardware.calib_path", "so101_follower_calib.json");
    declare_parameter<double>("hardware.home_duration", 4.0);
    declare_parameter<double>("hardware.gripper_q", 0.0);
    declare_parameter<bool>("hardware.gripper_closed", false);
    declare_parameter<int>("hardware.gripper_torque_limit", 200);
    declare_parameter<double>("hardware.current_lsb_a", 0.0065);
    const auto kt_nm_per_a = declare_parameter<std::vector<double>>("hardware.kt_nm_per_a", {1.0, 1.0, 1.0, 1.0, 1.0, 0.0});
    if (!valid_kt(kt_nm_per_a)) {
      throw std::invalid_argument("Invalid kt_nm_per_a gains");
    }
    declare_parameter<int>("hardware.bus.rx_timeout_ns", 2500000);
    declare_parameter<int>("hardware.bus.tx_timeout_ns", 750000);
    declare_parameter<int>("hardware.bus.max_bus_fails", 3);
    const auto k_servo = declare_parameter<std::vector<double>>("hardware.bridge.k_servo", {50.0, 90.0, 11.0, 50.0, 50.0, 50.0});
    if (!strictly_positive(k_servo)) {
      throw std::invalid_argument("Invalid k_servo gains");
    }
    declare_parameter<double>("hardware.bridge.max_lead_q", 0.12);
    declare_parameter<int>("hardware.bridge.goal_speed", 40);
  }

  // Control core.
  std::unique_ptr<arm_control::PlantInterface> plant_;
  std::unique_ptr<arm_control::Controller> controller_;
  std::unique_ptr<arm_control::ControlLoop> loop_;
  std::vector<double> target_;
  std::string reference_type_;
  Eigen::VectorXd target_eigen_;
  Eigen::VectorXd q_ref_;
  Eigen::VectorXd qdot_ref_;
  Eigen::VectorXd qddot_ref_;
  double rate_hz_ = 200.0;
  bool rt_enable_ = true;
  int rt_priority_ = 80;
  int rt_cpu_ = -1;
  int jitter_samples_ = 60000;
  std::string jitter_csv_;

  // RT <-> non-RT handoff.
  arm_control::SpscRing<arm_control::Sample> ring_;
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> worker_failed_{false};
  std::string worker_error_;
  bool worker_fault_logged_ = false;
  std::atomic<int> active_workers_{0};
  std::atomic<int> max_workers_{0};
  std::atomic<int> dropped_count_{0};
  int previous_dropped_count_ = 0;
  bool worker_counters_ = false;

  // ROS side.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<arm_msgs::msg::ArmMetrics>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr fault_action_timer_;

  // param updates
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_parameters_callback_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmControlNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
