#include "vqw_servo_driver_component/vqw_servo_driver_component.hpp"

namespace vqw_servo_driver_component
{

    VqwServoDriverComponent::VqwServoDriverComponent(const rclcpp::NodeOptions &options) : Node("vqw_servo_driver_component", options)
    {
        this->declare_parameter("i2c_device_name", "/dev/i2c-1");
        this->declare_parameter("i2c_address", 0x40);
        this->declare_parameter("topic_name", "vqw_servo_driver");

        std::string my_device_name = "/dev/i2c-1";
        int         my_i2c_address = 0x40;
        std::string my_topic_name  = "vqw_servo_driver";

        try
            {
                my_device_name = this->get_parameter("i2c_device_name").as_string();
            }
        catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() i2c_device_name: %s", e.what());
                //return;
            }

        try
            {
                my_i2c_address = this->get_parameter("i2c_address").as_int();
            }
        catch (const rclcpp::ParameterTypeException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() i2c_address: %s", e.what());
                //return;
            }

        try
            {
                my_topic_name = this->get_parameter("topic_name").as_string();
            }
        catch (const rclcpp::ParameterTypeException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() topic_name: %s", e.what());
                //return;
            }

        if (my_device_name.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() missing or empty i2c_device_name");
                my_device_name = "/dev/i2c-1";
            }

        if (my_i2c_address < 1 || my_i2c_address > 127)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() Invalid i2c_address: %d,  must be 0..127", my_i2c_address);
                my_i2c_address = 0x40;
            }

        if (my_topic_name.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() missing or empty topic_name");
                my_topic_name = "vqw_servo_driver";
            }

        i2c_address = my_i2c_address;
        device_name = my_device_name;
        topic_name  = my_topic_name;

        RCLCPP_INFO(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() i2c_device_name: \"%s\"  i2c_address: 0x%02X  topic_name: \"%s\"",
                    device_name.c_str(), i2c_address, topic_name.c_str());

        //---- Create the subscription -------------------
        auto topic_callback = [this](std_msgs::msg::Float32MultiArray::UniquePtr msg) -> void { this->on_msg_recv(std::move(msg)); };
        subscription_       = this->create_subscription<std_msgs::msg::Float32MultiArray>(topic_name, 10, topic_callback);
    }

    VqwServoDriverComponent::~VqwServoDriverComponent()
    {
        if (i2c_controller) { i2c_controller->close_port(); }
        //...
    }

    bool VqwServoDriverComponent::open()
    {
        if (i2c_controller) { i2c_controller->close_port(); }
        if (pwm_servo_driver) { pwm_servo_driver->close(); }
        //---- Create the I2C controller -------------------
        i2c_controller = std::make_shared<I2CController>();
        int rc         = i2c_controller->open_port(device_name, i2c_address);
        if (rc < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() Failed to open i2c device: %s   errno=%d  %s",
                             device_name.c_str(), errno, strerror(errno));
                return false;
            }

        //---- Create the PWM driver -------------------
        pwm_servo_driver = std::make_shared<PWMServoDriver>(i2c_controller);
        int rc2          = pwm_servo_driver->begin();
        if (!rc2)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::VqwServoDriverComponent() Failed to initialize PWM driver");
                return false;
            }
            return true;
    }

    void VqwServoDriverComponent::on_msg_recv(std_msgs::msg::Float32MultiArray::UniquePtr msg)
    {
        if (!pwm_servo_driver)
            {
                bool ok = open();
                if (!ok) { return; }
            }
        if (msg->data.size() != 2)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::on_msg_recv() Invalid message size: %d,  must be 2", (int)msg->data.size());
                return;
            }
        int   pwm_index = static_cast<int>(msg->data[0] + 0.49);
        float pwm_value = msg->data[1];
        set_pwm(pwm_index, pwm_value);
    }

    /// @brief set the pwm value for a specific pwm index
    /// @param pwm_index 0..15
    /// @param pwm_value in Radians, -pi/2 to pi/2
    void VqwServoDriverComponent::set_pwm(int pwm_index, float pwm_value)
    {
        if (pwm_index < 0 || pwm_index > 15)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::set_pwm(int,float) Invalid pwm index: %d,  must be 0..15", pwm_index);
                return;
            }
        if (pwm_value < -M_PI / 2 || pwm_value > M_PI / 2)
            {
                RCLCPP_ERROR(this->get_logger(), "VqwServoDriverComponent::set_pwm(int,float) Invalid pwm value: %f,  must be between -pi/2 and pi/2", pwm_value);
                return;
            }
        float pluse_width_ms = ((pwm_value + M_PI / 2) / M_PI) + 1.0;
        pwm_servo_driver->setPWM(pwm_index, 0, pluse_width_ms);
    }

    // // // // // // void VqwServoDriverComponent::open_pwm()
    // // // // // // {
    // // // // // //     int rc = i2c_controller->open_port(device_name, i2c_address);
    // // // // // //     if (rc < 0) {
    // // // // // //         RCLCPP_ERROR(this->get_logger(), "Failed to open i2c device: %s   errno=%d  %s", device_name.c_str(), errno, strerror(errno));
    // // // // // //         return;
    // // // // // //     }
    // // // // // // }

    //~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=
}       // namespace vqw_servo_driver_component

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vqw_servo_driver_component::VqwServoDriverComponent)
