# ros2_control_dynamixel_ax12a Library

 ROS2 Control Hardware interface for Dynamixel AX-12A servos controlled by diff_drive_controller

## Usage

Clone Dynamixel SDK and this project into your workspace.

Example controllers config file:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 30

    diff_cont:
      type: diff_drive_controller/DiffDriveController

    joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster
  

diff_cont:
  ros__parameters:
    publish_rate: 30.0

    base_frame_id: base_link

    left_wheel_names: ['left_wheel_joint']
    right_wheel_names: ['right_wheel_joint']

    wheel_separation: 0.184
    wheel_radius: 0.06

    use_stamped_vel: false
```

Example xacro file:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <ros2_control name="DynamixelHw" type="system">
        <hardware>
            <plugin>dynamixel_hw/DynamixelHw</plugin>
            <param name="usb_port">/dev/ttyUSB0</param>
            <param name="baud_rate">1000000</param>
            <param name="left_wheel_name">left_wheel_joint</param>
            <param name="right_wheel_name">right_wheel_joint</param>
        </hardware>
        <joint name="left_wheel_joint">
            <param name="id">10</param>
            <param name="invert">false</param>
            <command_interface name="velocity">
                <param name="min">-6</param>
                <param name="max">6</param>
            </command_interface>
            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>
        <joint name="right_wheel_joint">
            <param name="id">11</param>
            <param name="invert">true</param>
            <command_interface name="velocity">
                <param name="min">-6</param>
                <param name="max">6</param>
            </command_interface>
            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>
    </ros2_control>
</robot>
```
