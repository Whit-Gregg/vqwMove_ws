
#include <cstdint>
#include <limits>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "bno086_hardware_interface/Bno086HardwareInterface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

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
    //-=+~-=+~-=  on_export_state_interfaces
    //-=+~-=+~-=
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() const
    {
            std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "orientation.x", &orientation_[0]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "orientation.y", &orientation_[1]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "orientation.z", &orientation_[2]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "orientation.w", &orientation_[3]));

            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "angular_velocity.x", &angular_velocity_[0]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "angular_velocity.y", &angular_velocity_[1]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "angular_velocity.z", &angular_velocity_[2]));

            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "linear_acceleration.x", &linear_acceleration_[0]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "linear_acceleration.y", &linear_acceleration_[1]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(name_, "linear_acceleration.z", &linear_acceleration_[2]));
            
            return state_interfaces;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  export_state_interfaces
    //-=+~-=+~-=
    // // // std::vector<hardware_interface::StateInterface> Bno086Hardwareinterface::export_state_interfaces()
    // // // {
    // // //     std::vector<hardware_interface::StateInterface> state_interfaces;

    // // //     state_interfaces.emplace_back(name_, "orientation.x", &orientation_[0]);
    // // //     state_interfaces.emplace_back(name_, "orientation.y", &orientation_[1]);
    // // //     state_interfaces.emplace_back(name_, "orientation.z", &orientation_[2]);
    // // //     state_interfaces.emplace_back(name_, "orientation.w", &orientation_[3]);

    // // //     state_interfaces.emplace_back(name_, "angular_velocity.x", &angular_velocity_[0]);
    // // //     state_interfaces.emplace_back(name_, "angular_velocity.y", &angular_velocity_[1]);
    // // //     state_interfaces.emplace_back(name_, "angular_velocity.z", &angular_velocity_[2]);

    // // //     state_interfaces.emplace_back(name_, "linear_acceleration.x", &linear_acceleration_[0]);
    // // //     state_interfaces.emplace_back(name_, "linear_acceleration.y", &linear_acceleration_[1]);
    // // //     state_interfaces.emplace_back(name_, "linear_acceleration.z", &linear_acceleration_[2]);

    // // //     return state_interfaces;
    // // // }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  export_command_interfaces
    //-=+~-=+~-=
    // // // // // std::vector<hardware_interface::CommandInterface> Bno086Hardwareinterface::export_command_interfaces()
    // // // // // {
    // // // // //     std::vector<hardware_interface::CommandInterface> command_interfaces;
    // // // // //     return command_interfaces;
    // // // // // }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=
    //-=+~-=+~-=
    //-=+~-=+~-=  on_activate
    //-=+~-=+~-=
    hardware_interface::CallbackReturn Bno086Hardwareinterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO(anyone): prepare the robot to receive commands
        bool isOK = pPipeDriver->connect(serial_port_name, serial_port_speed);

        if (!isOK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "Bno086Hardwareinterface::on_activate()  pPipeDriver->connect(\'%s\', %d) failed !!!!!", serial_port_name.c_str(), serial_port_speed);
            return CallbackReturn::ERROR;
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
    //-=+~-=+~-=  BNO086_CLI
    //-=+~-=+~-=
    std::string Bno086Hardwareinterface::BNO086_CLI([[maybe_unused]] std::string cmd_text)
    {
        std::string result = "????";
        return result;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    void Bno086Hardwareinterface::get_hardware_parameters(const hardware_interface::HardwareInfo &hardware_info)
    {
        try //----------- serial_port_name -------------
        {
            serial_port_name = hardware_info.hardware_parameters.at("bno086_serial_port_name");
        }
        catch (const std::out_of_range &)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Bno086Hardwareinterface"),
                         "Bno086Hardwareinterface::on_init() 'bno086_serial_port_name' must be defined as a hardware parameter in the URDF file.");
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
                         "Bno086Hardwareinterface::on_init() 'bno086_serial_port_speed' must be defined as a hardware parameter in the URDF file.");
        }
        RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"),
                    "Bno086Hardwareinterface::get_hardware_parameters() bno086_serial_port_name='%s', bno086_serial_port_speed=%d", serial_port_name.c_str(), serial_port_speed);

    } // end of: Bno086Hardwareinterface::get_hardware_parameters

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    // // // // // bool Bno086Hardwareinterface::connect(const std::string &serial_port_name_)
    // // // // // {
    // // // // //     // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "connect(%s)...", device.c_str());
    // // // // //     if (serial_port_fd != -1) { close(serial_port_fd); }
    // // // // //     serial_port_fd = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY);
    // // // // //     connected_     = serial_port_fd != -1;

    // // // // //     if (connected_)
    // // // // //         {
    // // // // //             serial_port_name = serial_port_name_;
    // // // // //             setSerialDeviceOptions();
    // // // // //         }
    // // // // //     else
    // // // // //         {
    // // // // //             RCLCPP_ERROR(rclcpp::get_logger("Bno086Hardwareinterface"), "connect(%s) !! FAILED !!", serial_port_name_.c_str());
    // // // // //             // std::cerr << "Failed to open serial device: " << device << std::endl;
    // // // // //             perror("Error");
    // // // // //         }
    // // // // //     RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "connect(%s) == %d", serial_port_name_.c_str(), connected_);
    // // // // //     return connected_;
    // // // // // }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    // // // // void Bno086Hardwareinterface::disconnect()
    // // // // {
    // // // //     if (connected_)
    // // // //         {
    // // // //             close(serial_port_fd);
    // // // //             connected_     = false;
    // // // //             serial_port_fd = -1;
    // // // //         }
    // // // // }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    // // // // // bool Bno086Hardwareinterface::restart()
    // // // // // {
    // // // // //     // RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "ReStarting...........%s", device_name_.c_str());
    // // // // //     disconnect();
    // // // // //     return connect(serial_port_name);
    // // // // // }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    // // // // // // // void Bno086Hardwareinterface::setSerialDeviceOptions()
    // // // // // // // {
    // // // // // // //     const int      read_timeout_ms = 100;
    // // // // // // //     struct termios options;
    // // // // // // //     int            rc = tcgetattr(serial_port_fd, &options);

    // // // // // // //     options.c_cflag &= ~PARENB;              // Clear parity bit, disabling parity (most common)
    // // // // // // //     options.c_cflag &= ~CSTOPB;              // Clear stop field, only one stop bit used in communication (most common)
    // // // // // // //     options.c_cflag &= ~CSIZE;               // Clear all the size bits
    // // // // // // //     options.c_cflag |= CS8;                  // 8 bits per byte (most common)
    // // // // // // //     options.c_cflag &= ~CRTSCTS;             // Disable RTS/CTS hardware flow control (most common)
    // // // // // // //     options.c_cflag |= CREAD | CLOCAL;       // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // // // // // // //     options.c_lflag &= ~ICANON;       // Disable canonical mode
    // // // // // // //     options.c_lflag &= ~ECHO;         // Disable echo
    // // // // // // //     options.c_lflag &= ~ECHOE;        // Disable erasure
    // // // // // // //     options.c_lflag &= ~ECHONL;       // Disable new-line echo
    // // // // // // //     options.c_lflag &= ~ISIG;         // Disable interpretation of INTR, QUIT and SUSP

    // // // // // // //     options.c_iflag &= ~(IXON | IXOFF | IXANY);                                            // Turn off s/w flow ctrl
    // // // // // // //     options.c_iflag &= ~(ICRNL | INLCR | IGNBRK | BRKINT | PARMRK | ISTRIP | IGNCR);       // Turn off translation of carriage return and newline

    // // // // // // //     options.c_oflag &= ~OPOST;       // Prevent special interpretation of output bytes (e.g. newline chars)
    // // // // // // //     options.c_oflag &= ~ONLCR;       // Prevent conversion of newline to carriage return/line feed

    // // // // // // //     // Set in/out baud rate to be 9600, 38400, 115200
    // // // // // // //     cfsetispeed(&options, serial_port_speed);
    // // // // // // //     cfsetospeed(&options, serial_port_speed);

    // // // // // // //     options.c_cc[VMIN]  = 0;
    // // // // // // //     options.c_cc[VTIME] = (cc_t)read_timeout_ms / 100;

    // // // // // // //     tcflush(serial_port_fd, TCIFLUSH);
    // // // // // // //     rc = tcsetattr(serial_port_fd, TCSANOW, &options);
    // // // // // // //     if (rc != 0) { perror("tcsetattr"); }
    // // // // // // // }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

} // namespace bno086_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bno086_hardware_interface::Bno086Hardwareinterface, hardware_interface::SensorInterface)
