// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMC_ROS_HW_PLUGIN_HPP
#define SMC_ROS_HW_PLUGIN_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "smc_ros_hw_plugin/visibility_control.h"
#include "smc_ros_hw_plugin/smc_cppserial_lib.hpp"
#include "smc_ros_hw_plugin/motor.hpp"

namespace smc_ros_hw_plugin
{
class SMCDriverHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string motorA_wheel_name = "";
  std::string motorB_wheel_name = "";
  std::string port = "";
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SMCDriverHardware);

  SMC_ROS_HW_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SMC_ROS_HW_PLUGIN_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  SMC smc_; // serial communication
  Config cfg_; // configuration
  Motor motorA_; // motorA setup
  Motor motorB_; // motorB setup
};

}  // namespace smc_ros_hw_plugin

#endif  // SMC_ROS_HW_PLUGIN_HPP
