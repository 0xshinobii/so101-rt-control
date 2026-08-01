// ROS 2 wrapper (step 4). The control loop runs on its OWN std::thread at a
// fixed rate and never touches rclcpp. Telemetry crosses to the non-RT side
// through a lock-free SPSC ring; a wall-timer on the executor thread drains it
// and publishes JointState + ArmMetrics. rclcpp::spin never touches the hot path
// -- exactly the isolation Phase 3's RT thread needs.
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_control/arm_types.hpp"
#include "arm_control/control_loop.hpp"
#include "arm_control/mujoco_backend.hpp"
#include "arm_control/pd_controller.hpp"
#include "arm_control/spsc_ring.hpp"
#include "arm_msgs/msg/arm_metrics.hpp"

using namespace std::chrono_literals;

namespace {
const std::vector<std::string> kJointNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper"};
constexpr int kArmJoints = 5;  // first 5 are the arm; index 5 is the held gripper

Eigen::VectorXd to_eigen(const std::vector<double>& v) {
  Eigen::VectorXd e(v.size());
  for (size_t i = 0; i < v.size(); ++i) e[i] = v[i];
  return e;
}
}  // namespace

class ArmControlNode : public rclcpp::Node {
public:
  ArmControlNode() : rclcpp::Node("arm_control_node"), ring_(1024) {
    // --- parameters (gains / target / model / rate) ---
    const std::string model_path = declare_parameter<std::string>(
        "model_path", "models/so101/scene_torque.xml");
    rate_hz_ = declare_parameter<double>("rate_hz", 200.0);
    const auto kp = declare_parameter<std::vector<double>>(
        "kp", {40.0, 40.0, 25.0, 15.0, 8.0, 5.0});
    const auto kd = declare_parameter<std::vector<double>>(
        "kd", {3.0, 3.0, 2.0, 1.0, 0.6, 0.4});
    target_ = declare_parameter<std::vector<double>>(
        "target", {0.6, 0.7, -0.8, 0.5, 0.4, 0.0});

    // --- build the control core ---
    plant_ = std::make_unique<arm_control::MujocoBackend>(model_path);
    controller_ = std::make_unique<arm_control::PdController>(to_eigen(kp), to_eigen(kd));
    loop_ = std::make_unique<arm_control::ControlLoop>(*plant_, *controller_,
                                                       to_eigen(target_));
    loop_->reset();

    // --- publishers (non-RT side) ---
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    metrics_pub_ = create_publisher<arm_msgs::msg::ArmMetrics>("arm_metrics", 10);

    // --- non-RT drain+publish timer on the executor thread ---
    publish_timer_ = create_wall_timer(20ms, [this]() { drain_and_publish(); });

    // --- start the control thread (this is the fixed-rate loop) ---
    control_thread_ = std::thread([this]() { control_loop(); });

    RCLCPP_INFO(get_logger(), "arm_control_node up: model=%s rate=%.0f Hz",
                model_path.c_str(), rate_hz_);
  }

  ~ArmControlNode() override {
    running_.store(false, std::memory_order_release);
    if (control_thread_.joinable()) control_thread_.join();
  }

private:
  // Runs on its own thread. Fixed-rate; RT-clean body (no alloc, no rclcpp).
  void control_loop() {
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    auto next = std::chrono::steady_clock::now();
    arm_control::Sample s;  // reused; no per-iteration allocation
    while (running_.load(std::memory_order_acquire)) {
      loop_->step_once(s);
      ring_.push(s);  // best-effort; never blocks the control thread
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
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
    metrics_pub_->publish(m);
  }

  // Control core.
  std::unique_ptr<arm_control::MujocoBackend> plant_;
  std::unique_ptr<arm_control::PdController> controller_;
  std::unique_ptr<arm_control::ControlLoop> loop_;
  std::vector<double> target_;
  double rate_hz_ = 200.0;

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
