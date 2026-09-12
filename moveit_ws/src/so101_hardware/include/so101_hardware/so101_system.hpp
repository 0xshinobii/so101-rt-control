#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so101_hardware/feetech_bus.hpp"
#include "so101_hardware/so101_calib.hpp"

namespace so101_hardware {

class So101System : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params)
      override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
  static constexpr double kRadPerTick = 2.0 * 3.14159265358979323846 / 4096.0;

  bool has_interface(
      const std::vector<hardware_interface::InterfaceInfo>& interfaces,
      const std::string& name) const;

  void disable_torque();

  FeetechBus bus_;
  std::string device_;
  int baud_ = 1000000;
  int torque_limit_ = 300;
  bool enable_torque_ = true;

  std::vector<JointCalib> calib_;
  std::vector<uint8_t> ids_;
  std::vector<int> ticks_;
  std::vector<double> q_;
  std::vector<double> q_prev_;
  std::vector<uint8_t> rx_;
  std::vector<uint8_t> tx_;
  std::vector<std::string> pos_state_keys_;
  std::vector<std::string> vel_state_keys_;
  std::vector<std::string> pos_cmd_keys_;
};

}  // namespace so101_hardware
