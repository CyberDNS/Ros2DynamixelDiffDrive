#include <cmath>
#include <string>
#include <algorithm>

#include "dynamixel_hw/dynamixel_connection.hpp"
#include "dynamixel_hw/dynamixel_wheel.hpp"
#include "rclcpp/rclcpp.hpp"

#define ADDR_TORQUE_ENABLE 24  // 1 byte
#define ADDR_CW_ANGLE_LIMIT 6  // 2 byte
#define ADDR_CCW_ANGLE_LIMIT 8 // 2 byte
#define ADDR_MOVING_SPEED 32   // 2 byte
#define ADDR_PRESENT_SPEED 38  // 2 byte

#define MIN_MOVING_SPEED_VALUE 125
#define MAX_MOVING_SPEED_VALUE 1023
#define STOP 0
#define REVERSE_OFFSET 1024
#define MAX_REV_PER_MIN 60

#define LOGGER_IDENTIFIER "dynamixel_hw > dynamixel_wheel"

const double CONVERSION_FACTOR = MAX_MOVING_SPEED_VALUE / (MAX_REV_PER_MIN * 2 * M_PI / 60.0);

namespace dynamixel_hw
{
    void DynamixelWheel::init(const std::string name, const uint8_t dynamixel_id, bool is_inverted)
    {
        this->name = name;
        this->dynamixel_id_ = dynamixel_id;
        this->inverted_ = is_inverted ? -1 : 1;

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "JOINT: %s Inverted=%d", name.c_str(), this->inverted_);
    }

    bool DynamixelWheel::setup(const DynamixelConnection &connection)
    {
        connection_ = connection;

        bool is_success = true;
        // Use wheel mode
        is_success = connection_.write2ByteTxRx(dynamixel_id_, ADDR_CW_ANGLE_LIMIT, 0);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set Wheel Control Mode for CW.");
            return false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to set Wheel Control Mode for CW.");
        }

        is_success = connection_.write2ByteTxRx(dynamixel_id_, ADDR_CCW_ANGLE_LIMIT, 0);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set Wheel Control Mode for CCW.");
            return false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to set Wheel Control Mode for CCW.");
        }

        // Enable Torque of DYNAMIXEL
        is_success = connection_.write1ByteTxRx(dynamixel_id_, ADDR_TORQUE_ENABLE, 1);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to enable torque.");
            return false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to enable torque.");
        }

        return true;
    }

    void DynamixelWheel::shutdown()
    {
        // Disable Torque of DYNAMIXEL
        bool is_success = connection_.write1ByteTxRx(dynamixel_id_, ADDR_TORQUE_ENABLE, 0);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to disable torque.");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to disable torque.");
        }
    }

    bool DynamixelWheel::rotate(double velocity)
    {

        uint16_t cur_dynamixel_vel = convert_to_dynamixel_vel(vel);
        uint16_t dynamixel_vel = convert_to_dynamixel_vel(velocity);

        if (dynamixel_vel != cur_dynamixel_vel)
        {
            bool is_success = connection_.write2ByteTxRx(dynamixel_id_, ADDR_MOVING_SPEED, dynamixel_vel);
            if (!is_success)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set Wheel rotation.");
                return false;
            }
            else
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to set Wheel rotation. %u", dynamixel_vel);
            }
        }

        return true;
    }

    double DynamixelWheel::get_velocity()
    {
        uint16_t dynamixel_vel;

        bool is_success = connection_.read2ByteTxRx(dynamixel_id_, ADDR_PRESENT_SPEED, &dynamixel_vel);
        if (!is_success)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read Wheel rotation.");
            return vel;
        }
        else
        {
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to read Wheel rotation. %u", dynamixel_vel);
        }

        return convert_to_ros_control_vel(dynamixel_vel);
    }

    uint16_t DynamixelWheel::convert_to_dynamixel_vel(double rad_vel)
    {
        int16_t dynamixel_vel = this->inverted_ * static_cast<int>(std::round(CONVERSION_FACTOR * rad_vel));
        dynamixel_vel = std::clamp(dynamixel_vel, static_cast<int16_t>(-1 * MAX_MOVING_SPEED_VALUE), static_cast<int16_t>(MAX_MOVING_SPEED_VALUE));
        int16_t dynamixel_value = dynamixel_vel;
        if (dynamixel_value < 0)
        {
            dynamixel_value = REVERSE_OFFSET - dynamixel_value;
        }

        return static_cast<uint16_t>(dynamixel_value);
    }

    double DynamixelWheel::convert_to_ros_control_vel(uint16_t dynamixel_vel)
    {
        int16_t dynamixel_value = static_cast<int16_t>(dynamixel_vel);
        if (dynamixel_value >= REVERSE_OFFSET)
        {
            dynamixel_value = REVERSE_OFFSET - dynamixel_value;
        }

        return (this->inverted_ * dynamixel_value) / CONVERSION_FACTOR;
    }
}