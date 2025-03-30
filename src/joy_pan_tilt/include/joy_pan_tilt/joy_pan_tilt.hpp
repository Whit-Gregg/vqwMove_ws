#ifndef JOY_PAN_TILT__JOY_PAN_TILT_HPP_
#define JOY_PAN_TILT__JOY_PAN_TILT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "joy_pan_tilt/visibility_control.h"

#define BUTTON_SELECT_LEFT 0
#define BUTTON_SELECT_RIGHT 1
#define BUTTON_RESET 2

#define AXIS_PAN 2
#define AXIS_TILT 3

#define CMD_OFFSET_LEFT 0
#define CMD_OFFSET_RIGHT 2

#define PAN_RESET_VALUE 0.0
#define TILT_RESET_VALUE 0.5

namespace joy_pan_tilt
{

class JoyPanTilt : public rclcpp::Node
{
public:
  JoyPanTilt(const rclcpp::NodeOptions & options) : Node("joy_pan_tilt", options)
  {
    // ...
    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyPanTilt::joy_callback, this, std::placeholders::_1));
    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JoyPanTilt::joint_callback, this, std::placeholders::_1));
  }  

  virtual ~JoyPanTilt() = default;

  private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
    

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    double scale_factor_ = 0.1;
    double current_position_left_pan_ = 0.0;
    double current_position_left_tilt_ = 0.0;
    double current_position_right_pan_ = 0.0;
    double current_position_right_tilt_ = 0.0;

};

}  // namespace joy_pan_tilt

#endif  // JOY_PAN_TILT__JOY_PAN_TILT_HPP_
