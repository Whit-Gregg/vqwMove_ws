#include "cam_pan_tilt_hardware_interface/cam_pan_tilt_hardware_interface.hpp"
#include <cmath>
#include <stdlib.h>
#include <string>

namespace cam_pan_tilt_hardware_interface
{

    CamPanTiltHardwareInterface::ServoInfo *CamPanTiltHardwareInterface::AddServo(const std::string &name, int channel, double offset)
    {
        ServoInfo servo;
        servo.name     = name;
        servo.channel  = channel;
        servo.offset   = offset;
        servo.position = 0.0;
        servo.cmd_if   = nullptr;
        servo.state_if = nullptr;
        servos[name]   = servo;
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::AddServo(\"%s\", %d, %.2f)", my_name.c_str(),
                    name.c_str(), channel, offset);
        return GetServo(name);
    }

    CamPanTiltHardwareInterface::ServoInfo *CamPanTiltHardwareInterface::GetServo(const std::string &name)
    {
        auto it = servos.find(name);
        if (it == servos.end())
            {
                RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::GetServo(\"%s\")  NOT FOUND!",
                             my_name.c_str(), name.c_str());
                return nullptr;
            }
        RCLCPP_DEBUG(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::GetServo(\"%s\") Found, channel = %d",
                     my_name.c_str(), name.c_str(), it->second.channel);
        return &it->second;
    }

    /// Initialization of the hardware interface from data parsed from the robot's URDF.
    /**
     * \param[in] hardware_info structure with data from URDF.
     * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
     * \returns CallbackReturn::ERROR if any error happens or data are missing.
     */
    CallbackReturn CamPanTiltHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if (ActuatorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        //===================================================================================================
        my_name                               = hardware_info.name;
        hardware_interface::CallbackReturn rc = get_hardware_parameters(hardware_info);
        //===================================================================================================
        return rc;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_configure()....", my_name.c_str());
        if (ActuatorInterface::on_configure(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_configure()  ActuatorInterface::on_configure() finished.", my_name.c_str());

        bool ok = open_servo_driver();
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_configure()  open_servo_driver() isOK = %s", my_name.c_str(), 
                    ok ? "true" : "false");
        if (!ok)
            {
                RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"),
                             "CamPanTiltHardwareInterface[%s]::on_configure(). Servo Driver did not open", my_name.c_str());
                return CallbackReturn::ERROR;
            }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_activate()....", my_name.c_str());
        if (ActuatorInterface::on_activate(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_deactivate()....", my_name.c_str());
        if (ActuatorInterface::on_deactivate(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_cleanup()....", my_name.c_str());
        if (ActuatorInterface::on_cleanup(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        close_servo_driver();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CamPanTiltHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_shutdown()....", my_name.c_str());
        if (ActuatorInterface::on_shutdown(previous_state) != CallbackReturn::SUCCESS) { return CallbackReturn::ERROR; }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> CamPanTiltHardwareInterface::on_export_state_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "CamPanTiltHardwareInterface[%s]::on_export_state_interfaces()....", my_name.c_str());

        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces = ActuatorInterface::on_export_state_interfaces();
        int                                                             ndx              = 0;
        for (auto &state_interface : state_interfaces)
            {
                std::string name   = state_interface->get_prefix_name();
                ServoInfo  *pServo = GetServo(name);
                // if (pServo == nullptr) { pServo = AddServo(name, -1); }
                if (pServo != nullptr)
                    {
                        for (auto &joint_state : joint_states_)
                            {
                                if (joint_state->get_prefix_name() == name)
                                    {
                                        bool ok = joint_state->set_value(0.001);
                                        if (!ok) { ok = true; }

                                        pServo->state_if = joint_state;
                                    }
                            }
                        if (pServo->state_if == nullptr)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"),
                                             "CamPanTiltHardwareInterface::on_export_state_interfaces() name \"%s\" not found in joint_states!", name.c_str());
                            }
                    }
                else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"),
                                     "CamPanTiltHardwareInterface::on_export_state_interfaces() joint_name \"%s\" not found in servos!", name.c_str());
                    }
                ndx++;
            }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> CamPanTiltHardwareInterface::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces = ActuatorInterface::on_export_command_interfaces();
        for (auto &command_interface : command_interfaces)
            {
                bool        ok     = command_interface->set_value(0.001);
                std::string name   = command_interface->get_prefix_name();
                ServoInfo  *pServo = GetServo(name);
                // if (pServo == nullptr) { pServo = AddServo(name, -1); }
                if (pServo != nullptr)
                    {
                        pServo->cmd_if = command_interface;
                        if ((name == "camera_L_tilt_joint") || (name == "camera_R_tilt_joint")) { ok = pServo->cmd_if->set_value(0.5); }
                    }
                if (!ok) { ok = true; }
            }
        return command_interfaces;
    }

    /// Read the current state values from the actuator.
    /**
     * The data readings from the physical hardware has to be updated
     * and reflected accordingly in the exported state interfaces.
     * That is, the data pointed by the interfaces shall be updated.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
    hardware_interface::return_type CamPanTiltHardwareInterface::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        // ToDo: Read the current state values from the actuator, and update the state interfaces.
        return hardware_interface::return_type::OK;
    }

    /// Write the current command values to the actuator.
    /**
     * The physical hardware shall be updated with the latest value from
     * the exported command interfaces.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
    hardware_interface::return_type CamPanTiltHardwareInterface::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if ((i2c_controller == nullptr) || (i2c_controller->isOpen() == false))
            {
                RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"), "CamPanTiltHardwareInterface::write() i2c port is not open");
                open_servo_driver();
            }
        for (auto &[name, servo] : servos)
            {
                double value = 0;
                if (servo.cmd_if == nullptr) { continue; }
                if (servo.state_if == nullptr) { continue; }
                //if (!servo.cmd_if->get_value(value)) { continue; }
                auto val = servo.cmd_if->get_optional<double>();
                if (!val) { continue; }
                value = *val;

                if (std::isnan(value)) continue;
                if (value != servo.position)
                    {
                        double pluse_width_ms = (((value + servo.offset) + M_PI / 2) / M_PI) + 1.0;       // convert radians to pulse width in milliseconds
                        // RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                        //             "CamPanTiltHardwareInterface[%s]::write() name=%s, channel=%d, value=%.3f  offset=%.3f  pluse_width_ms=%.4f",
                        //            my_name.c_str(), name.c_str(), servo.channel, value, servo.offset, pluse_width_ms);

                        if (pwm_servo_driver) { pwm_servo_driver->setPWM(servo.channel, pluse_width_ms); }
                        servo.position = value;
                        bool ok        = servo.state_if->set_value(value);
                        if (!ok) { ok = true; }
                    }
            }

        return hardware_interface::return_type::OK;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=
    bool CamPanTiltHardwareInterface::open_servo_driver()
    {
        if (i2c_controller) { i2c_controller->close_port(); }
        if (pwm_servo_driver) { pwm_servo_driver->close(); }
        //---- Create the I2C controller -------------------
        i2c_controller = std::make_shared<I2CController>();
        int rc         = i2c_controller->open_port(i2c_device_name, i2c_address);
        if (rc < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                             "CamPanTiltHardwareInterface::open_servo_driver() i2c_device = \"%s\" i2c_address = 0x%02X   errno=%d %s", i2c_device_name.c_str(),
                             i2c_address, errno, strerror(errno));
                return false;
            }

        //---- Create the PWM driver -------------------
        pwm_servo_driver = std::make_shared<PWMServoDriver>(i2c_controller);
        int rc2          = pwm_servo_driver->begin();
        if (!rc2)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                             "CamPanTiltHardwareInterface::open_servo_driver() Failed to initialize PWM driver");
                return false;
            }
        return true;
    }

    bool CamPanTiltHardwareInterface::close_servo_driver()
    {
        if (i2c_controller) { i2c_controller->close_port(); }
        if (pwm_servo_driver) { pwm_servo_driver->close(); }
        return true;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=
    //-=+~-=+~-=  get_hardware_parameters
    //-=+~-=+~-=
    hardware_interface::CallbackReturn CamPanTiltHardwareInterface::get_hardware_parameters(const hardware_interface::HardwareInfo &hardware_info)
    {
        // Dump_HardwareInfo(hardware_info);

        CallbackReturn ret = CallbackReturn::SUCCESS;

        try       //----------- i2c_topic -------------
            {
                i2c_device_name = hardware_info.hardware_parameters.at("i2c_device");
            }
        catch (const std::out_of_range &)
            {
                i2c_device_name = "/dev/i2c-1";
                // ret = CallbackReturn::ERROR;
                RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                             "CamPanTiltHardwareInterface::get_hardware_parameters() "
                             "'i2c_device' must be defined as a hardware parameter"
                             " on the 'hardware' element in the URDF file."
                             " using default value \"%s\"",
                             i2c_device_name.c_str());
            }
        if (ret != CallbackReturn::SUCCESS) { return ret; }
        RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"), "CamPanTiltHardwareInterface::get_hardware_parameters() i2c_device = \"%s\"",
                    i2c_device_name.c_str());

        //----------- joints -------------

        int count = 0;
        for (auto &joint : hardware_info.joints)
            {
                std::string joint_name = std::to_string(count);
                try
                    {
                        joint_name             = joint.name;
                        std::string cannel_str = joint.parameters.at("channel");
                        int         channel    = atoi(cannel_str.c_str());
                        std::string offset_str = joint.parameters.at("offset");
                        double      offset     = atof(offset_str.c_str());
                        AddServo(joint_name, channel, offset);
                        // RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                        //             "CamPanTiltHardwareInterface::get_hardware_parameters() added joint_name = \"%s\",  channel=%d to servo map",
                        //             joint_name.c_str(), channel);
                    }
                catch (const std::out_of_range &)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                                     "CamPanTiltHardwareInterface::get_hardware_parameters()"
                                     " joint='%s'"
                                     " 'channel' and 'offset' must be defined as"
                                     " parameters on the ros2_control Joint in the URDF file.",
                                     joint_name.c_str());
                    }
                count++;
            }

        //===================================================================================================
        return ret;
    }       // end of: CamPanTiltHardwareInterface::get_hardware_parameters

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    void CamPanTiltHardwareInterface::Dump_HardwareInfo(const hardware_interface::HardwareInfo &hardware_info)
    {
        std::string hardware_parameters;
        for (auto &param : hardware_info.hardware_parameters) { hardware_parameters += param.first + "=" + param.second + " "; }
        RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                    "CamPanTiltHardwareInterface::Dump_HardwareInfo()    "
                    "HardwareInfo.name='%s', HardwareInfo.type='%s', "
                    "HardwareInfo.hardware_plugin_name='%s'  hardware_parameters='%s'",
                    hardware_info.name.c_str(), hardware_info.type.c_str(), hardware_info.hardware_plugin_name.c_str(), hardware_parameters.c_str());

        for (uint s = 0; s < hardware_info.joints.size(); s++)
            {
                std::string joint_parameters;
                for (auto &param : hardware_info.joints[s].parameters) { joint_parameters += param.first + "=" + param.second + " "; }
                RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                            "CamPanTiltHardwareInterface::Dump_HardwareInfo()           "
                            "joint[%d].name='%s', joint[%d].type='%s', "
                            "joint[%d].command_interfaces.size()=%ld, joint[%d].state_interfaces.size()=%ld  params='%s'",
                            s, hardware_info.joints[s].name.c_str(), s, hardware_info.joints[s].type.c_str(), s,
                            hardware_info.joints[s].command_interfaces.size(), s, hardware_info.joints[s].state_interfaces.size(), joint_parameters.c_str());

                for (uint i = 0; i < hardware_info.joints[0].command_interfaces.size(); i++)
                    {
                        std::string command_interface_parameters;
                        for (auto &param : hardware_info.joints[s].command_interfaces[i].parameters)
                            {
                                command_interface_parameters += param.first + "=" + param.second + " ";
                            }
                        RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                                    "CamPanTiltHardwareInterface::Dump_HardwareInfo()                "
                                    "joint[%d].command_interfaces[%d].name='%s', "
                                    "joint[%d].command_interfaces[%d].data_type='%s',  params='%s'",
                                    s, i, hardware_info.joints[s].command_interfaces[i].name.c_str(), s, i,
                                    hardware_info.joints[s].command_interfaces[i].data_type.c_str(), command_interface_parameters.c_str());
                    }

                for (uint i = 0; i < hardware_info.joints[0].state_interfaces.size(); i++)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("CamPanTiltHardwareInterface"),
                                    "CamPanTiltHardwareInterface::Dump_HardwareInfo()                "
                                    "joint[%d].state_interfaces[%d].name='%s', "
                                    "joint[%d].state_interfaces[%d].data_type='%s'",
                                    s, i, hardware_info.joints[s].state_interfaces[i].name.c_str(), s, i,
                                    hardware_info.joints[s].state_interfaces[i].data_type.c_str());
                    }
            }
    }

}       // namespace cam_pan_tilt_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cam_pan_tilt_hardware_interface::CamPanTiltHardwareInterface, hardware_interface::ActuatorInterface)
