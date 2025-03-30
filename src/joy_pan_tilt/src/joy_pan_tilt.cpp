#include "joy_pan_tilt/joy_pan_tilt.hpp"

namespace joy_pan_tilt
{

    // Float32MultiArray

    void JoyPanTilt::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        int ndx = 0;
        for (auto J : msg->name)
            {
                if (J == "camera_L_tilt_joint")
                    {
                        current_position_left_tilt_ = msg->position[ndx];
                    }
                if (J == "camera_L_pan_joint")
                    {
                        current_position_left_pan_ = msg->position[ndx];
                    }
                if (J == "camera_R_tilt_joint")
                    {
                        current_position_right_tilt_ = msg->position[ndx];
                    }
                if (J == "camera_R_pan_joint")
                    {
                        current_position_right_pan_ = msg->position[ndx];
                    }

                    ndx++;
            }

        // if (msg->name.size() < 2) { return; }
        // if (msg->name[0] == "pan" && msg->name[1] == "tilt")
        //     {
        //         scale_factor_ = msg->position[0];
        //     }
    }

    void JoyPanTilt::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std_msgs::msg::Float64MultiArray pan_tilt;
        bool                             should_publish = false;
        if (msg->buttons.size() < 3) { return; }
        for (int i = 0; i < 4; i++) { pan_tilt.data.push_back(std::numeric_limits<double>::quiet_NaN()); }

        if (msg->buttons[BUTTON_RESET] != 0)
            {
                if (msg->buttons[BUTTON_SELECT_LEFT] != 0)
                    {
                        pan_tilt.data[CMD_OFFSET_LEFT]     = PAN_RESET_VALUE;
                        pan_tilt.data[CMD_OFFSET_LEFT + 1] = TILT_RESET_VALUE;
                        should_publish                     = true;
                    }
                if (msg->buttons[BUTTON_SELECT_RIGHT] != 0)
                    {
                        pan_tilt.data[CMD_OFFSET_RIGHT]     = PAN_RESET_VALUE;
                        pan_tilt.data[CMD_OFFSET_RIGHT + 1] = TILT_RESET_VALUE;
                        should_publish                      = true;
                    }
            }
        else
            {
                if (msg->buttons[BUTTON_SELECT_LEFT] != 0)
                    {
                        pan_tilt.data[CMD_OFFSET_LEFT]     = current_position_left_pan_ + (msg->axes[AXIS_PAN] * scale_factor_);
                        pan_tilt.data[CMD_OFFSET_LEFT + 1] = current_position_left_tilt_ + (msg->axes[AXIS_TILT] * scale_factor_);
                        should_publish                     = true;
                    }
                if (msg->buttons[BUTTON_SELECT_RIGHT] != 0)
                    {
                        pan_tilt.data[CMD_OFFSET_RIGHT]     = current_position_right_pan_ + (msg->axes[AXIS_PAN] * scale_factor_);
                        pan_tilt.data[CMD_OFFSET_RIGHT + 1] = current_position_right_tilt_ + (msg->axes[AXIS_TILT] * scale_factor_);
                        should_publish                      = true;
                    }
            }

        if (should_publish) { pub_->publish(pan_tilt); }
    }

}       // namespace joy_pan_tilt

#include <rclcpp_components/register_node_macro.hpp>

    RCLCPP_COMPONENTS_REGISTER_NODE(joy_pan_tilt::JoyPanTilt)


