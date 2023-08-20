#ifndef DYNAMIXEL_HW__DYNAMIXEL_WHEEL_HPP_
#define DYNAMIXEL_HW__DYNAMIXEL_WHEEL_HPP_

#include <string>
#include <dynamixel_hw/dynamixel_connection.hpp>

namespace dynamixel_hw
{
  class DynamixelWheel
  {
  public:
    std::string name;

    double cmd = 0;
    double pos = 0;
    double vel = 0;

    void init(const std::string name, const uint8_t dynamixel_id, bool is_inverted);
    bool setup(const DynamixelConnection &connection);
    void shutdown();

    bool rotate(double velocity);
    double get_velocity();

  private:
    dynamixel_hw::DynamixelConnection connection_;

    uint8_t dynamixel_id_;
    int8_t inverted_;

    uint16_t convert_to_dynamixel_vel(double rad_vel);
    double convert_to_ros_control_vel(uint16_t dynamixel_vel);
  };
}

#endif