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

namespace dynamixel_hw
{

  CallbackReturn DynamixelHw::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "configure");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    config_.usb_port = info_.hardware_parameters["usb_port"];
    config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

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

      if (joint.name == config_.left_wheel_name)
      {
        std::string boolStr = joint.parameters.at("invert");
        std::transform(boolStr.begin(), boolStr.end(), boolStr.begin(), [](unsigned char c){ return std::tolower(c); });

        left_wheel_.init(joint.name, std::stoi(joint.parameters.at("id")), boolStr == "true");
      }
      if (joint.name == config_.right_wheel_name)
      {
        std::string boolStr = joint.parameters.at("invert");
        std::transform(boolStr.begin(), boolStr.end(), boolStr.begin(), [](unsigned char c){ return std::tolower(c); });

        right_wheel_.init(joint.name, std::stoi(joint.parameters.at("id")), boolStr == "true");
      }
    }

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

    dynamixel_connection_.connect(config_.usb_port, config_.baud_rate);

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
