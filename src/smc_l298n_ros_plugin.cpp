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

#include "smc_l298n_ros_plugin/smc_l298n_ros_plugin.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

namespace smc_l298n_ros_plugin
{
  hardware_interface::CallbackReturn SMCHardwareDriver::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.motorA_wheel_name = info_.hardware_parameters["motorA_wheel_name"];
    cfg_.motorB_wheel_name = info_.hardware_parameters["motorB_wheel_name"];
    cfg_.port = info_.hardware_parameters["port"];

    motorA_.setup(cfg_.motorA_wheel_name);
    motorB_.setup(cfg_.motorB_wheel_name);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SMCHardwareDriver"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SMCHardwareDriver"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SMCHardwareDriver"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SMCHardwareDriver"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("SMCHardwareDriver"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> SMCHardwareDriver::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorA_.name, hardware_interface::HW_IF_POSITION, &motorA_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorA_.name, hardware_interface::HW_IF_VELOCITY, &motorA_.angVel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorB_.name, hardware_interface::HW_IF_POSITION, &motorB_.angPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        motorB_.name, hardware_interface::HW_IF_VELOCITY, &motorB_.angVel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> SMCHardwareDriver::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        motorA_.name, hardware_interface::HW_IF_VELOCITY, &motorA_.cmdAngVel));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        motorB_.name, hardware_interface::HW_IF_VELOCITY, &motorB_.cmdAngVel));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn SMCHardwareDriver::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Configuring ...please wait...");
    if (smc_.connected())
    {
      smc_.disconnect();
    }
    smc_.connect(cfg_.port, 115200, 100);
    for (int i = 1; i <= 6; i += 1)
    { // wait for the smc to fully setup
      delay_ms(1000);
      RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "configuring controller: %d sec", (i));
    }
    smc_.sendTargetVel(0.000, 0.000); // targetA, targetB
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SMCHardwareDriver::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Cleaning up ...please wait...");
    if (smc_.connected())
    {
      smc_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SMCHardwareDriver::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Activating ...please wait...");
    if (!smc_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    smc_.sendTargetVel(0.000, 0.000); // targetA, targetB
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Successfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SMCHardwareDriver::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Deactivating ...please wait...");
    smc_.sendTargetVel(0.000, 0.000); // targetA, targetB
    RCLCPP_INFO(rclcpp::get_logger("SMCHardwareDriver"), "Successfully Deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type SMCHardwareDriver::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!smc_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    try
    {

      float motorA_angPos, motorB_angPos;
      float motorA_angVel, motorB_angVel;

      smc_.getMotorsPos(motorA_angPos, motorB_angPos); // gets angPosA, angPosB
      smc_.getMotorsVel(motorA_angVel, motorB_angVel); // gets angVelA, angVelB

      motorA_.angPos = (double)motorA_angPos;
      motorB_.angPos = (double)motorB_angPos;

      motorA_.angVel = (double)motorA_angVel;
      motorB_.angVel = (double)motorB_angVel;
    }
    catch (...)
    {
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type smc_l298n_ros_plugin ::SMCHardwareDriver::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!smc_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    float motorA_cmdAngVel, motorB_cmdAngVel;

    motorA_cmdAngVel = (float)motorA_.cmdAngVel;
    motorB_cmdAngVel = (float)motorB_.cmdAngVel;

    smc_.sendTargetVel(motorA_cmdAngVel, motorB_cmdAngVel); // targetA, targetB

    return hardware_interface::return_type::OK;
  }

} // namespace smc_l298n_ros_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    smc_l298n_ros_plugin::SMCHardwareDriver, hardware_interface::SystemInterface)
