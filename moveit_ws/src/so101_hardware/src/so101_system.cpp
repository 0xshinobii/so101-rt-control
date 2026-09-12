#include "so101_hardware/so101_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace so101_hardware {
namespace {

std::string require_param(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& key) {
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    throw std::runtime_error("missing hardware parameter '" + key + "'");
  }
  return it->second;
}

}  // namespace

bool So101System::has_interface(
    const std::vector<hardware_interface::InterfaceInfo>& interfaces,
    const std::string& name) const {
  return std::any_of(
      interfaces.begin(), interfaces.end(),
      [&](const hardware_interface::InterfaceInfo& info) { return info.name == name; });
}

hardware_interface::CallbackReturn So101System::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    device_ = require_param(info_.hardware_parameters, "device");
    baud_ = std::stoi(require_param(info_.hardware_parameters, "baud"));
    torque_limit_ =
        std::stoi(require_param(info_.hardware_parameters, "torque_limit"));
    const std::string enable_torque =
        require_param(info_.hardware_parameters, "enable_torque");
    // xacro emits Python-bool "True"/"False" for true/false args.
    enable_torque_ = (enable_torque == "true" || enable_torque == "True" ||
                      enable_torque == "1");
    const std::string calib_file =
        require_param(info_.hardware_parameters, "calib_file");
    const std::string calib_text = load_calib_text(calib_file);

    const std::size_t n = info_.joints.size();
    if (n == 0) {
      throw std::runtime_error("URDF ros2_control block has no joints");
    }

    calib_.resize(n);
    ids_.resize(n);
    ticks_.assign(n, 0);
    q_.assign(n, 0.0);
    q_prev_.assign(n, 0.0);
    rx_.assign(n * 2, 0);
    tx_.assign(n * 2, 0);
    pos_state_keys_.resize(n);
    vel_state_keys_.resize(n);
    pos_cmd_keys_.resize(n);

    for (std::size_t i = 0; i < n; ++i) {
      const auto& joint = info_.joints[i];
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        throw std::runtime_error(
            joint.name + ": need exactly one position command interface");
      }
      if (!has_interface(joint.state_interfaces,
                         hardware_interface::HW_IF_POSITION) ||
          !has_interface(joint.state_interfaces,
                         hardware_interface::HW_IF_VELOCITY)) {
        throw std::runtime_error(
            joint.name + ": need position and velocity state interfaces");
      }

      calib_[i] = load_joint_calib(calib_text, joint.name);
      ids_[i] = static_cast<uint8_t>(calib_[i].id);
      pos_state_keys_[i] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
      vel_state_keys_[i] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
      pos_cmd_keys_[i] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void So101System::disable_torque() {
  const uint8_t off = 0;
  for (uint8_t id : ids_) {
    bus_.write(id, FeetechBus::kAddrTorqueEnable, &off, 1);
  }
}

hardware_interface::CallbackReturn So101System::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    bus_.open(device_, baud_);
    // Library defaults are sized for a single request/reply (measured 1.96 ms
    // max). This plugin issues a six-servo sync_read whose replies arrive in
    // series, so the deadline must be proportionally larger. Still well inside
    // the 20 ms cycle at 50 Hz.
    bus_.set_rx_timeout_ns(8'000'000);   // 8 ms
    bus_.set_tx_timeout_ns(2'000'000);   // 2 ms
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      if (!bus_.ping(ids_[i])) {
        throw std::runtime_error("ping failed for " + info_.joints[i].name +
                                 " id=" + std::to_string(ids_[i]));
      }
    }

    const uint16_t limit = static_cast<uint16_t>(
        std::clamp(torque_limit_, 1, 1000));
    const uint8_t limit_le[2] = {static_cast<uint8_t>(limit & 0xFF),
                                 static_cast<uint8_t>((limit >> 8) & 0xFF)};
    for (uint8_t id : ids_) {
      if (!bus_.write(id, FeetechBus::kAddrTorqueLimit, limit_le, 2)) {
        throw std::runtime_error("torque limit write failed");
      }
    }

    if (!bus_.sync_read(ids_, FeetechBus::kAddrPresentPosition, 2, rx_) ||
        rx_.size() < ids_.size() * 2) {
      throw std::runtime_error("cannot read present positions before torque on");
    }
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      ticks_[i] = static_cast<int>(rx_[2 * i] | (rx_[2 * i + 1] << 8));
      q_[i] = calib_[i].sign * (ticks_[i] - calib_[i].zero_ticks) * kRadPerTick;
      q_prev_[i] = q_[i];
      set_state<double>(pos_state_keys_[i], q_[i]);
      set_state<double>(vel_state_keys_[i], 0.0);
      set_command<double>(pos_cmd_keys_[i], q_[i]);
    }

    if (enable_torque_) {
      const uint8_t on = 1;
      for (uint8_t id : ids_) {
        if (!bus_.write(id, FeetechBus::kAddrTorqueEnable, &on, 1)) {
          disable_torque();
          throw std::runtime_error("torque enable failed");
        }
      }
    } else {
      disable_torque();
    }

    consecutive_read_failures_ = 0;
    consecutive_write_failures_ = 0;
    time_since_good_read_ = 0.0;
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "on_activate failed: %s", e.what());
    if (bus_.is_open()) {
      disable_torque();
      bus_.close();
    }
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
      get_logger(),
      "SO-101 hardware active on %s @ %d baud, torque_limit=%d, enable_torque=%s",
      device_.c_str(), baud_, torque_limit_, enable_torque_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn So101System::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (bus_.is_open()) {
    disable_torque();
    bus_.close();
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type So101System::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  time_since_good_read_ += period.seconds();

  if (!bus_.sync_read(ids_, FeetechBus::kAddrPresentPosition, 2, rx_) ||
      rx_.size() < ids_.size() * 2) {
    if (++consecutive_read_failures_ >= kMaxConsecutiveReadFailures) {
      const auto& s = bus_.stats();
      RCLCPP_ERROR(get_logger(),
                   "sync_read failed %d cycles in a row: timeouts=%lu checksum_errors=%lu",
                   consecutive_read_failures_, s.timeouts, s.checksum_errors);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "sync_read failed, holding last state (%d in a row)",
                         consecutive_read_failures_);
    return hardware_interface::return_type::OK;
  }
  consecutive_read_failures_ = 0;

  const double dt = time_since_good_read_;
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    ticks_[i] = static_cast<int>(rx_[2 * i] | (rx_[2 * i + 1] << 8));
    q_[i] = calib_[i].sign * (ticks_[i] - calib_[i].zero_ticks) * kRadPerTick;
    const double vel = (dt > 0.0) ? (q_[i] - q_prev_[i]) / dt : 0.0;
    q_prev_[i] = q_[i];
    set_state<double>(pos_state_keys_[i], q_[i]);
    set_state<double>(vel_state_keys_[i], vel);
  }
  time_since_good_read_ = 0.0;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type So101System::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!enable_torque_) {
    return hardware_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    const double cmd = get_command<double>(pos_cmd_keys_[i]);
    const auto& c = calib_[i];
    const int tick =
        c.zero_ticks + static_cast<int>(std::lround(c.sign * cmd / kRadPerTick));
    const int clamped = std::clamp(tick, c.min_ticks, c.max_ticks);
    tx_[2 * i] = static_cast<uint8_t>(clamped & 0xFF);
    tx_[2 * i + 1] = static_cast<uint8_t>((clamped >> 8) & 0xFF);
  }
  if (!bus_.sync_write(ids_, FeetechBus::kAddrGoalPosition, tx_)) {
    if (++consecutive_write_failures_ >= kMaxConsecutiveWriteFailures) {
      const auto& s = bus_.stats();
      RCLCPP_ERROR(get_logger(),
                   "sync_write failed %d cycles in a row: timeouts=%lu checksum_errors=%lu",
                   consecutive_write_failures_, s.timeouts, s.checksum_errors);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "sync_write failed, holding last goal (%d in a row)",
                         consecutive_write_failures_);
    return hardware_interface::return_type::OK;
  }
  consecutive_write_failures_ = 0;
  return hardware_interface::return_type::OK;
}

}  // namespace so101_hardware

PLUGINLIB_EXPORT_CLASS(so101_hardware::So101System,
                       hardware_interface::SystemInterface)
