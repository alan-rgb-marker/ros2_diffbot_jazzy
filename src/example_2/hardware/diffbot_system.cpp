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

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "libserial/SerialPort.h"

using namespace LibSerial;
using namespace std::chrono_literals;

namespace ros2_control_demo_example_2
{
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    config_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    config_.device = info_.hardware_parameters["device"];
    config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    config_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    config_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    wheel_l_.setup(config_.left_wheel_name, config_.enc_counts_per_rev);
    wheel_r_.setup(config_.right_wheel_name, config_.enc_counts_per_rev);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(get_logger(), "Init success. Joints: %s, %s | Config: %s, %s",
                info_.joints[0].name.c_str(), info_.joints[1].name.c_str(),
                config_.left_wheel_name.c_str(), config_.right_wheel_name.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> ros2_control_demo_example_2::DiffBotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    // return std::vector<hardware_interface::StateInterface>();
    return state_interfaces;
  }
  
  std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));
    // return std::vector<hardware_interface::CommandInterface>();
    return command_interfaces;
  }
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    // stm32_comms_.connect(config_.device, config_.baud_rate, config_.timeout_ms);

    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    try
    {
      stm32_comms_.connect(config_.device, config_.baud_rate, config_.timeout_ms);
      if (!stm32_comms_.isConnected())
      {
        RCLCPP_ERROR(get_logger(), "Failed to connect to STM32 on %s", config_.device.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(get_logger(), "Successfully connected to STM32 on %s", config_.device.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception during STM32 connection: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化輪子狀態
    // wheel_l_.pos = 0.0;
    // wheel_l_.vel = 0.0;
    // wheel_l_.cmd = 0.0;
    // wheel_r_.pos = 0.0;
    // wheel_r_.vel = 0.0;
    // wheel_r_.cmd = 0.0;

    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    // stm32_comms_.connect(config_.device, config_.baud_rate, config_.timeout_ms);

    RCLCPP_INFO(get_logger(), "Activated. Checking cmd: L=%f, R=%f", wheel_l_.cmd, wheel_r_.cmd);

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    stm32_comms_.disconnect();

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffBotSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    stm32_comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

    double delta_seconds = period.seconds();

    double pos_prev = wheel_l_.pos;
    wheel_l_.pos = wheel_l_.calcEncAngle();

    wheel_l_.vel = (wheel_l_.calcEncAngle() - pos_prev) / delta_seconds;

    pos_prev = wheel_r_.pos;
    wheel_r_.pos = wheel_r_.calcEncAngle();
    wheel_r_.vel = (wheel_r_.calcEncAngle() - pos_prev) / delta_seconds;

    // wheel_l_.pos = 0.0;
    // wheel_l_.vel = 0.0;
    // wheel_r_.pos = 0.0;
    // wheel_r_.vel = 0.0;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ros2_control_demo_example_2 ::DiffBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    int motor_l_counts_per_loop = (int)(wheel_l_.cmd / wheel_l_.rads_per_count / config_.loop_rate);
    int motor_r_counts_per_loop = (int)(wheel_r_.cmd / wheel_r_.rads_per_count / config_.loop_rate);
    RCLCPP_INFO(get_logger(), "Left cmd: %f, Right cmd: %f", wheel_l_.cmd, wheel_r_.cmd);
    stm32_comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
