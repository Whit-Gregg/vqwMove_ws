#ifndef BNO086_HARDWARE_INTERFACE__BNO086HARDWAREINTERFACE_HPP_
#define BNO086_HARDWARE_INTERFACE__BNO086HARDWAREINTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

// // #include "bno086_hardware_interface/SPI.hpp"
// // #include "bno086_hardware_interface/BNO08x.hpp"
#include "bno086_hardware_interface/visibility_control.h"
#include "hardware_interface/sensor_interface.hpp"
//#include "semantic_components/imu_sensor.hpp"

#include "vqwPipe.h"
#include "vqwPipe_Channel.h"
#include "vqwPipe_Driver_Linux.hpp"
#include "Channel_ROS_msg.h"

namespace bno086_hardware_interface
{

  class Bno086Hardwareinterface : public hardware_interface::SensorInterface
                                  //public semantic_components::IMUSensor
  {
  public:
    Bno086Hardwareinterface() : Name("bno086") {
      //RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Bno086Hardwareinterface()...");
    };
    Bno086Hardwareinterface(std::string _name) : Name(_name) {
      //RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "Bno086Hardwareinterface(%s)...", Name.c_str());
    };

    virtual ~Bno086Hardwareinterface() = default;
    // // // {
    // // //   RCLCPP_INFO(rclcpp::get_logger("Bno086Hardwareinterface"), "~Bno086Hardwareinterface()...");
    // // // };

    virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    virtual hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override;

    virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

//    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces();
//    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces();

    virtual hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

//    virtual std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() const override;

    std::string BNO086_CLI(std::string cmd_text);

    // virtual hardware_interface::CallbackReturn on_cleanup() override { return hardware_interface::CallbackReturn::OK; }
    // virtual hardware_interface::CallbackReturn on_shutdown() override { return hardware_interface::CallbackReturn::OK; }
    // virtual hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override ;

    /// data values members inherited from IMUSensor
    // Order is: orientation X,Y,Z,W angular velocity X,Y,Z and linear acceleration X,Y,Z
    // std::array<double, 4> orientation_;
    // std::array<double, 3> angular_velocity_;
    // std::array<double, 3> linear_acceleration_;

    // // // // void set_reports();

  private:
    // // // bool connect(const std::string &device);
    // // // void disconnect();
    // // // bool restart();
    // // // void setSerialDeviceOptions();
    void get_hardware_parameters(const hardware_interface::HardwareInfo &hardware_info);

    std::array<double, 4> orientation_;
    std::array<double, 3> angular_velocity_;
    std::array<double, 3> linear_acceleration_;

    hardware_interface::StateInterface::ConstSharedPtr pOrientation_X;
    hardware_interface::StateInterface::ConstSharedPtr pOrientation_Y;
    hardware_interface::StateInterface::ConstSharedPtr pOrientation_Z;
    hardware_interface::StateInterface::ConstSharedPtr pOrientation_W;
    
    hardware_interface::StateInterface::ConstSharedPtr pAngular_velocity_X;
    hardware_interface::StateInterface::ConstSharedPtr pAngular_velocity_Y;
    hardware_interface::StateInterface::ConstSharedPtr pAngular_velocity_Z;

    hardware_interface::StateInterface::ConstSharedPtr pLinear_acceleration_X;
    hardware_interface::StateInterface::ConstSharedPtr pLinear_acceleration_Y;
    hardware_interface::StateInterface::ConstSharedPtr pLinear_acceleration_Z;

    std::string Name;
    std::string serial_port_name;
    int serial_port_speed = 0;

    std::shared_ptr<vqw::vqwPipe_Driver_Linux> pPipeDriver;
    std::shared_ptr<vqw::vqwPipe> pPipe;;
    std::shared_ptr<vqw::vqwPipe_Channel> pChannelROS;

    void on_vqwPipe_chan_IMU_data_msg(const vqwPipe_chan_IMU_data_msg *pData);


    bool connected_ = false;
  }; // end of:  class Bno086Hardwareinterface

} // end of:  namespace bno086_hardware_interface

#endif // BNO086_HARDWARE_INTERFACE__BNO086HARDWAREINTERFACE_HPP_
