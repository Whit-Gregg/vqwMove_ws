
#include <cstdint>
#include <limits>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "bno086_hardware_interface/Bno086HardwareInterface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace bno086_hardware_interface
{

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  on_init
    //-=+~-=+~-=
    hardware_interface::CallbackReturn Bno086Hardwareinterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        //===================================================================================================
        get_hardware_parameters(hardware_info);
        //===================================================================================================

        if (serial_port_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"),
                         "Bno086Hardwareinterface::on_init()  serial_port_name parameter must be provided, in URDF !!!!!");
            return CallbackReturn::ERROR;
        }

        if (serial_port_speed == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"),
                         "Bno086Hardwareinterface::on_init()  serial_port_speed parameter must be provided, in URDF !!!!!");
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  on_configure
    //-=+~-=+~-=
    hardware_interface::CallbackReturn Bno086Hardwareinterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

        pPipeDriver = std::make_shared<vqw::vqwPipe_Driver_Linux>();
        pPipe = std::make_shared<vqw::vqwPipe>(pPipeDriver.get(), "bno086");
        pChannelROS = std::make_shared<vqw::vqwPipe_Channel>((uint8_t)vqw::vqwPipe_Channel_Number_Assignments::Channel_ROS, pPipe.get(), "ROS");

        pChannelROS->addCallback<vqwPipe_chan_IMU_data_msg>(
            [this](const vqwPipe_chan_IMU_data_msg *pMsg)
            {
                this->on_vqwPipe_chan_IMU_data_msg(pMsg);
            });

        return CallbackReturn::SUCCESS;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  on_export_state_interfaces
    //-=+~-=+~-=
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> Bno086Hardwareinterface::on_export_state_interfaces()
    {
        pOrientation_X = std::make_shared<hardware_interface::StateInterface>(Name, "orientation.x", &orientation_[0]);
        pOrientation_Y = std::make_shared<hardware_interface::StateInterface>(Name, "orientation.y", &orientation_[1]);
        pOrientation_Z = std::make_shared<hardware_interface::StateInterface>(Name, "orientation.z", &orientation_[2]);
        pOrientation_W = std::make_shared<hardware_interface::StateInterface>(Name, "orientation.w", &orientation_[3]);

        pAngular_velocity_X = std::make_shared<hardware_interface::StateInterface>(Name, "angular_velocity.x", &angular_velocity_[0]);
        pAngular_velocity_Y = std::make_shared<hardware_interface::StateInterface>(Name, "angular_velocity.y", &angular_velocity_[1]);
        pAngular_velocity_Z = std::make_shared<hardware_interface::StateInterface>(Name, "angular_velocity.z", &angular_velocity_[2]);

        pLinear_acceleration_X = std::make_shared<hardware_interface::StateInterface>(Name, "linear_acceleration.x", &linear_acceleration_[0]);
        pLinear_acceleration_Y = std::make_shared<hardware_interface::StateInterface>(Name, "linear_acceleration.y", &linear_acceleration_[1]);
        pLinear_acceleration_Z = std::make_shared<hardware_interface::StateInterface>(Name, "linear_acceleration.z", &linear_acceleration_[2]);

        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        state_interfaces.push_back(pOrientation_X);
        state_interfaces.push_back(pOrientation_Y);
        state_interfaces.push_back(pOrientation_Z);
        state_interfaces.push_back(pOrientation_W);

        state_interfaces.push_back(pAngular_velocity_X);
        state_interfaces.push_back(pAngular_velocity_Y);
        state_interfaces.push_back(pAngular_velocity_Z);

        state_interfaces.push_back(pLinear_acceleration_X);
        state_interfaces.push_back(pLinear_acceleration_Y);
        state_interfaces.push_back(pLinear_acceleration_Z);

        return state_interfaces;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  on_activate
    //-=+~-=+~-=
    hardware_interface::CallbackReturn Bno086Hardwareinterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // prepare the robot to receive commands and/or state
        bool isOK = pPipeDriver->connect(serial_port_name, serial_port_speed);

        if (!isOK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "Bno086Hardwareinterface::on_activate()  pPipeDriver->connect(\'%s\', %d) failed !!!!!", serial_port_name.c_str(), serial_port_speed);
            return CallbackReturn::ERROR;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "Bno086Hardwareinterface::on_activate()  pPipeDriver->connect(\'%s\', %d) IS ok.", serial_port_name.c_str(), serial_port_speed);
        }
        return CallbackReturn::SUCCESS;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  on_deactivate
    //-=+~-=+~-=
    hardware_interface::CallbackReturn Bno086Hardwareinterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        pPipeDriver->disconnect();
        // TODO(anyone): prepare the robot to stop receiving commands

        return CallbackReturn::SUCCESS;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  read
    //-=+~-=+~-=
    hardware_interface::return_type Bno086Hardwareinterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        pPipe->loop();
        return hardware_interface::return_type::OK;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  on_vqwPipe_chan_IMU_data_msg
    //-=+~-=+~-=
    void Bno086Hardwareinterface::on_vqwPipe_chan_IMU_data_msg(const vqwPipe_chan_IMU_data_msg *pData)
    {
        switch (pData->data_type_code)
        {
        case vqwPipe_chan_IMU_data_msg::data_type_code_Accel:
            // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Accel: X=%.3f, Y=%.3f, Z=%.3f", pData->X, pData->Y, pData->Z);
            break;
        case vqwPipe_chan_IMU_data_msg::data_type_code_Gyro:
            angular_velocity_[0] = pData->X;
            angular_velocity_[1] = pData->Y;
            angular_velocity_[2] = pData->Z;
            // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Gyro: X=%.3f, Y=%.3f, Z=%.3f", pData->X, pData->Y, pData->Z);
            break;
        case vqwPipe_chan_IMU_data_msg::data_type_code_Mag:
            // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Mag: X=%.3f, Y=%.3f, Z=%.3f", pData->X, pData->Y, pData->Z);
            break;
        case vqwPipe_chan_IMU_data_msg::data_type_code_LinAccel:
            linear_acceleration_[0] = pData->X;
            linear_acceleration_[1] = pData->Y;
            linear_acceleration_[2] = pData->Z;
            // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "LinAccel: X=%.3f, Y=%.3f, Z=%.3f", pData->X, pData->Y, pData->Z);
            break;
        case vqwPipe_chan_IMU_data_msg::data_type_code_Orientation:
            orientation_[0] = pData->X;
            orientation_[1] = pData->Y;
            orientation_[2] = pData->Z;
            orientation_[3] = pData->W;
            // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Orientation: X=%.3f, Y=%.3f, Z=%.3f, W=%.3f", pData->X, pData->Y, pData->Z, pData->W);
            break;
        default:
            // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Unknown data_type_code: %d", pData->data_type_code);
            break;
        }
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  BNO086_CLI
    //-=+~-=+~-=
    std::string Bno086Hardwareinterface::BNO086_CLI([[maybe_unused]] std::string cmd_text)
    {
        std::string result = "????";
        return result;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=
    //-=+~-=+~-=  get_hardware_parameters
    //-=+~-=+~-=
    void Bno086Hardwareinterface::get_hardware_parameters(const hardware_interface::HardwareInfo &hardware_info)
    {
        try //----------- serial_port_name -------------
        {
            serial_port_name = hardware_info.hardware_parameters.at("bno086_serial_port_name");
        }
        catch (const std::out_of_range &)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Bno086Hardwareinterface"),
                         "Bno086Hardwareinterface::get_hardware_parameters() 'bno086_serial_port_name' must be defined as a hardware parameter in the URDF file.");
        }

        try //----------- serial_port_speed -------------
        {
            serial_port_speed = 0;
            auto port__speed = hardware_info.hardware_parameters.at("bno086_serial_port_speed");
            serial_port_speed = std::stol(port__speed);
        }
        catch (const std::out_of_range &)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Bno086Hardwareinterface"),
                         "Bno086Hardwareinterface::get_hardware_parameters() 'bno086_serial_port_speed' must be defined as a hardware parameter in the URDF file.");
        }

        RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"),
                    "Bno086Hardwareinterface::get_hardware_parameters() bno086_serial_port_name='%s', bno086_serial_port_speed=%d", serial_port_name.c_str(), serial_port_speed);

        /*
        inside semantic_components/imu_sensor/hpp:
            state_interfaces{0..3] = orientation.x, orientation.y, orientation.z, orientation.w
            state_interfaces{4..6] = angular_velocity.x, angular_velocity.y, angular_velocity.z
            state_interfaces{7..9] = linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
        */

        //===================================================================================================
        RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Bno086Hardwareinterface::get_hardware_parameters()    "
                                                                   "HardwareInfo.name='%s', HardwareInfo.type='%s', HardwareInfo.hardware_plugin_name='%s'",
                    hardware_info.name.c_str(), hardware_info.type.c_str(), hardware_info.hardware_plugin_name.c_str());

        for (uint s = 0; s < info_.sensors.size(); s++)
        {
            RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Bno086Hardwareinterface::get_hardware_parameters()           "
                                                                       "sensor[%d].name='%s', sensor[%d].type='%s', sensor[%d].state_interfaces.size()=%ld",
                        s, info_.sensors[s].name.c_str(), s, info_.sensors[s].type.c_str(), s, info_.sensors[s].state_interfaces.size());

            for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
            {
                RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Bno086Hardwareinterface::get_hardware_parameters()                "
                                                                           "sensor[%d].state_interfaces[%d].name='%s', sensor[%d].state_interfaces[%d].data_type='%s'",
                            s, i, info_.sensors[s].state_interfaces[i].name.c_str(), s, i, info_.sensors[s].state_interfaces[i].data_type.c_str());
            }
        }

    } // end of: Bno086Hardwareinterface::get_hardware_parameters

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

} //  end of:  namespace bno086_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bno086_hardware_interface::Bno086Hardwareinterface, hardware_interface::SensorInterface)
