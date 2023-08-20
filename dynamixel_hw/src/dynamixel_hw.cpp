// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#include "dynamixel_hw/dynamixel_hw.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "dynamixel_hw/dynamixel_connection.hpp"
#include "dynamixel_hw/dynamixel_wheel.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define ADDR_TORQUE_ENABLE 24  // 1 byte
#define ADDR_CW_ANGLE_LIMIT 6  // 2 byte
#define ADDR_CCW_ANGLE_LIMIT 8 // 2 byte
#define ADDR_MOVING_SPEED 32   // 2 byte

#define LEFT_SERVO_ID 10
#define RIGHT_SERVO_ID 11

#define LEFT_SERVO_NAME "left_wheel_joint"
#define RIGHT_SERVO_NAME "right_wheel_joint"

#define LEFT_SERVO_INVERT false
#define RIGHT_SERVO_INVERT true

#define MIN_MOVING_SPEED_VALUE 125
#define MAX_MOVING_SPEED_VALUE 1023
#define STOP 0
#define REVERSE_OFFSET 1024

// Protocol version
#define PROTOCOL_VERSION 1.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

namespace dynamixel_hw
{

  CallbackReturn DynamixelHw::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "configure");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    left_wheel_.init(LEFT_SERVO_NAME, LEFT_SERVO_ID, false);
    right_wheel_.init(RIGHT_SERVO_NAME, RIGHT_SERVO_ID, true);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DynamixelHw::export_state_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DynamixelHw::export_command_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd));

    return command_interfaces;
  }

  CallbackReturn DynamixelHw::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "start");

    dynamixel_connection_.connect(DEVICE_NAME, BAUDRATE);

    left_wheel_.setup(dynamixel_connection_);
    right_wheel_.setup(dynamixel_connection_);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHw::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "stop");

    left_wheel_.shutdown();
    right_wheel_.shutdown();

    dynamixel_connection_.disconnect();

    return CallbackReturn::SUCCESS;
  }

  return_type DynamixelHw::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    double delta_seconds = period.seconds();

    // Update for the left wheel
    double left_vel = left_wheel_.get_velocity();
    left_wheel_.vel = left_vel;
    left_wheel_.pos += left_vel * delta_seconds; // Integrate velocity to get position

    // Update for the right wheel
    double right_vel = right_wheel_.get_velocity();
    right_wheel_.vel = right_vel;
    right_wheel_.pos += right_vel * delta_seconds; // Integrate velocity to get position

    return return_type::OK;
  }

  return_type DynamixelHw::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    bool is_success = false;

    is_success = left_wheel_.rotate(left_wheel_.cmd);
    if (!is_success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "Failed to rotate left wheel.");
    }

    is_success = right_wheel_.rotate(right_wheel_.cmd);
    if (!is_success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "Failed to rotate right wheel.");
    }

    return return_type::OK;
  }

} // namespace dynamixel_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hw::DynamixelHw, hardware_interface::SystemInterface)
