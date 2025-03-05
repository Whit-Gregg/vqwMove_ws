#ifndef CAM_PAN_TILT_HARDWARE_INTERFACE__CAM_PAN_TILT_HARDWARE_INTERFACE_HPP_
#define CAM_PAN_TILT_HARDWARE_INTERFACE__CAM_PAN_TILT_HARDWARE_INTERFACE_HPP_

#include <map>
#include <string>
#include <vector>

#include "cam_pan_tilt_hardware_interface/visibility_control.h"
#include "hardware_interface/actuator_interface.hpp"

#include "cam_pan_tilt_hardware_interface/i2c_controller.hpp"
#include "cam_pan_tilt_hardware_interface/pwm_servo_driver.hpp"

// #include "std_msgs/msg/float32_multi_array.hpp"
//  #include <hardware_interface/hardware_info.hpp>
//  #include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/rclcpp.hpp>

using hardware_interface::CallbackReturn;

namespace cam_pan_tilt_hardware_interface
{

    class CamPanTiltHardwareInterface : public hardware_interface::ActuatorInterface
    {
      public:
        CamPanTiltHardwareInterface()                                         = default;
        virtual ~CamPanTiltHardwareInterface()                                = default;
        CamPanTiltHardwareInterface(const CamPanTiltHardwareInterface &other) = delete;
        CamPanTiltHardwareInterface(CamPanTiltHardwareInterface &&other)      = default;

        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface::SharedPtr>    on_export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

      private:
        class ServoInfo
        {
          public:
            std::string                                     name;
            int                                             channel  = -1;
            double                                          position = 0.0;
            double                                          offset   = 0.0;
            hardware_interface::CommandInterface::SharedPtr cmd_if   = nullptr;
            hardware_interface::StateInterface::SharedPtr   state_if = nullptr;
        };

        std::map<std::string, ServoInfo> servos;

        ServoInfo *AddServo(const std::string &name, int channel, double offset);
        ServoInfo *GetServo(const std::string &name);

        CallbackReturn get_hardware_parameters(const hardware_interface::HardwareInfo &hardware_info);
        std::string    i2c_device_name;
        int            i2c_address = 0x40;
        std::string    my_name;

        I2CController::SharedPtr  i2c_controller;
        PWMServoDriver::SharedPtr pwm_servo_driver;
        bool                      open_servo_driver();
        bool                      close_servo_driver();

        void SendMsgToI2C(int channel, double value);

        void Dump_HardwareInfo(const hardware_interface::HardwareInfo &hardware_info);
    };

}       // namespace cam_pan_tilt_hardware_interface

#endif       // CAM_PAN_TILT_HARDWARE_INTERFACE__CAM_PAN_TILT_HARDWARE_INTERFACE_HPP_
