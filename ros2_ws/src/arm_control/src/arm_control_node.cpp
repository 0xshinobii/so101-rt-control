// ROS 2 wrapper (step 4). The control loop runs on its OWN std::thread at a
// fixed rate and never touches rclcpp. Telemetry crosses to the non-RT side
// through a lock-free SPSC ring; a wall-timer on the executor thread drains it
// and publishes JointState + ArmMetrics. rclcpp::spin never touches the hot path
// -- exactly the isolation Phase 3's RT thread needs.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <atomic>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_control/adaptive_computed_torque_controller.hpp"
#include "arm_control/arm_types.hpp"
#include "arm_control/computed_torque_controller.hpp"
#include "arm_control/control_loop.hpp"
#include "arm_control/controller.hpp"
#include "arm_control/mujoco_backend.hpp"
#include "arm_control/pd_controller.hpp"
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
}  // namespace

class ArmControlNode : public rclcpp::Node {
public:
  ArmControlNode() : rclcpp::Node("arm_control_node"), ring_(1024) {
    // --- parameters (gains / target / model / rate) ---
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
    jitter_csv_ = declare_parameter<std::string>("jitter_csv", "");
    const auto kp = declare_parameter<std::vector<double>>(
        "kp", {40.0, 40.0, 25.0, 15.0, 8.0, 5.0});
    const auto kd = declare_parameter<std::vector<double>>(
        "kd", {3.0, 3.0, 2.0, 1.0, 0.6, 0.4});
    const auto computed_kp = declare_parameter<std::vector<double>>(
        "computed_kp", {400.0, 400.0, 400.0, 400.0, 400.0, 400.0});
    const auto computed_kd = declare_parameter<std::vector<double>>(
        "computed_kd", {40.0, 40.0, 40.0, 40.0, 40.0, 40.0});
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
    if (reference_type_ != "step" && reference_type_ != "smooth") {
      throw std::invalid_argument("reference_type must be step or smooth");
    }
    target_eigen_ = to_eigen(target_);
    q_ref_ = Eigen::VectorXd::Zero(arm_control::kDof);
    qdot_ref_ = Eigen::VectorXd::Zero(arm_control::kDof);
    qddot_ref_ = Eigen::VectorXd::Zero(arm_control::kDof);

    // --- build the control core ---
    plant_ = std::make_unique<arm_control::MujocoBackend>(model_path);
    if (plant_payload_mass >= 0.0) {
      plant_->set_body_mass("known_payload", plant_payload_mass);
    }
    if (controller_type == "pd") {
      controller_ =
          std::make_unique<arm_control::PdController>(to_eigen(kp), to_eigen(kd));
    } else if (controller_type == "computed_torque") {
      controller_ = std::make_unique<arm_control::ComputedTorqueController>(
          urdf_path, to_eigen(computed_kp), to_eigen(computed_kd));
    } else if (controller_type == "adaptive_computed_torque") {
      controller_ =
          std::make_unique<arm_control::AdaptiveComputedTorqueController>(
              urdf_path, payload_urdf_path, reference_payload_mass,
              to_eigen(computed_kp), to_eigen(computed_kd),
              plant_->timestep(), estimator_config);
    } else {
      throw std::invalid_argument(
          "controller_type must be pd, computed_torque, or "
          "adaptive_computed_torque");
    }
    loop_ = std::make_unique<arm_control::ControlLoop>(*plant_, *controller_,
                                                       target_eigen_);
    loop_->reset();

    // --- publishers (non-RT side) ---
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    metrics_pub_ = create_publisher<arm_msgs::msg::ArmMetrics>("arm_metrics", 10);

    // --- non-RT drain+publish timer on the executor thread ---
    publish_timer_ = create_wall_timer(20ms, [this]() { drain_and_publish(); });

    // --- start the control thread (this is the fixed-rate loop) ---
    control_thread_ = std::thread([this]() { control_loop(); });

    RCLCPP_INFO(get_logger(),
                "arm_control_node up: model=%s controller=%s rate=%.0f Hz",
                model_path.c_str(), controller_type.c_str(), rate_hz_);
  }

  ~ArmControlNode() override {
    running_.store(false, std::memory_order_release);
    if (control_thread_.joinable()) control_thread_.join();
  }

private:
  // Runs on its own thread. Fixed-rate; RT-clean body (no alloc, no rclcpp).
  void control_loop() {
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
    std::vector<int64_t> late_ns(jitter_cap);
    size_t jitter_n = 0;

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    auto next = std::chrono::steady_clock::now();
    arm_control::Sample s;  // reused; no per-iteration allocation
    while (running_.load(std::memory_order_acquire)) {
      if (reference_type_ == "smooth") {
        minimum_jerk_reference(plant_->time(), target_eigen_, q_ref_,
                               qdot_ref_, qddot_ref_);
        loop_->set_reference(q_ref_, qdot_ref_, qddot_ref_);
      }
      loop_->step_once(s);
      ring_.push(s);  // best-effort; never blocks the control thread
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          period);
      arm_control::sleep_until_monotonic(next);
      if (jitter_n < jitter_cap) {
        const auto now = std::chrono::steady_clock::now();
        late_ns[jitter_n++] =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - next)
                .count();
      }
    }

    if (jitter_n == 0) return;
    int64_t min_ns = late_ns[0];
    int64_t max_ns = late_ns[0];
    long double sum = 0;
    for (size_t i = 0; i < jitter_n; ++i) {
      if (late_ns[i] < min_ns) min_ns = late_ns[i];
      if (late_ns[i] > max_ns) max_ns = late_ns[i];
      sum += static_cast<long double>(late_ns[i]);
    }
    std::fprintf(stderr,
                 "control jitter: n=%zu Min: %.3f Avg: %.3f Max: %.3f (us)\n",
                 jitter_n, min_ns / 1000.0,
                 static_cast<double>(sum / jitter_n) / 1000.0,
                 max_ns / 1000.0);
    if (jitter_csv_.empty()) return;
    std::ofstream out(jitter_csv_);
    if (!out) {
      std::fprintf(stderr, "failed to write %s\n", jitter_csv_.c_str());
      return;
    }
    out << "# rate_hz=" << rate_hz_ << "\n";
    out << "i,late_us\n";
    out.precision(9);
    for (size_t i = 0; i < jitter_n; ++i) {
      out << i << ',' << late_ns[i] / 1000.0 << '\n';
    }
    std::fprintf(stderr, "wrote %s\n", jitter_csv_.c_str());
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
    metrics_pub_->publish(m);
  }

  // Control core.
  std::unique_ptr<arm_control::MujocoBackend> plant_;
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
  std::atomic<bool> running_{true};

  // ROS side.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<arm_msgs::msg::ArmMetrics>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControlNode>());
  rclcpp::shutdown();
  return 0;
}
