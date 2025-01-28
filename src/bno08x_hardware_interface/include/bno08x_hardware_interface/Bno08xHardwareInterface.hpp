#ifndef BNO08X_HARDWARE_INTERFACE__BNO08XHARDWAREINTERFACE_HPP_
#define BNO08X_HARDWARE_INTERFACE__BNO08XHARDWAREINTERFACE_HPP_

#include "bno08x_hardware_interface/visibility_control.h"
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/rclcpp.hpp>
#include "elapsedMillis.h"
#include "OneLineBuffer.hpp"
#include "vqwPipe.hpp"
#include "vqwPipe_Channel.hpp"
#include "vqwPipe_Driver_Linux.hpp"
#include "Channel_ROS_msg.hpp"

namespace bno08x_hardware_interface
{

  class Bno08xHardwareInterface : public hardware_interface::SensorInterface
  {
  public:
    Bno08xHardwareInterface();

    virtual ~Bno08xHardwareInterface() = default;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=
    //-=+~-=+~-=+~-=   overrides from SensorInterface
    //-=+~-=+~-=+~-=
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    /// Initialization of the hardware interface from data parsed from the robot's URDF.
    /**
     * \param[in] hardware_info structure with data from URDF.
     * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
     * \returns CallbackReturn::ERROR if any error happens or data are missing.
     */
    virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    /**
     * Import the InterfaceDescription for the StateInterfaces from the HardwareInfo.
     * Separate them into the possible types: Sensor and store them.
     */
    // // // // // virtual void import_state_interface_descriptions(const hardware_interface::HardwareInfo &hardware_info);

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    /**
     * Default implementation for exporting the StateInterfaces. The StateInterfaces are created
     * according to the InterfaceDescription. The memory accessed by the controllers and hardware is
     * assigned here and resides in the sensor_interface.
     *
     * \return vector of shared pointers to the created and stored StateInterfaces
     */
    virtual std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
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
    virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=
    //-=+~-=+~-=+~-=   overrides from LifecycleNodeInterface
    //-=+~-=+~-=+~-=
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    // virtual CallbackReturn 	on_shutdown (const rclcpp_lifecycle::State &previous_state) override { return CallbackReturn::SUCCESS; }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    // virtual CallbackReturn 	on_error (const rclcpp_lifecycle::State &previous_state) override { return CallbackReturn::SUCCESS; }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    std::string BNO086_CLI(std::string cmd_text);

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  private:
    CallbackReturn get_hardware_parameters(const hardware_interface::HardwareInfo &hardware_info);

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

    std::string sensor_name;
    std::string Name;
    std::string serial_port_name;
    int serial_port_speed = 0;


    long count_of_IMU_data_msg = 0;
    long count_of_Accel = 0;
    long count_of_Gyro = 0;
    long count_of_Mag = 0;
    long count_of_LinAccel = 0;
    long count_of_Orientation = 0;

    long count_of_IMU_data_msg_previous = 0;
    long count_of_Accel_previous = 0;
    long count_of_Gyro_previous = 0;
    long count_of_Mag_previous = 0;
    long count_of_LinAccel_previous = 0;
    long count_of_Orientation_previous = 0;


    elapsedMillis elap_since_IMU_count_display;
    uint32_t elap_since_IMU_count_display_span = 25 * 60 * 1000;

    std::shared_ptr<vqw::vqwPipe_Driver_Linux> pPipeDriver;
    std::shared_ptr<vqw::vqwPipe> pPipe;

    std::shared_ptr<vqw::vqwPipe_Channel> pChannelROS;

    void on_vqwPipe_chan_IMU_data_msg(const vqwPipe_chan_IMU_data_msg *pData);

    vqw::OneLineBuffer pipe_rouge_data_buffer;
    void on_RougeData_line(const uint8_t *pData, int dataSize);
    elapsedMillis elap_since_last_rouge_data_display;
    uint32_t elap_since_last_rouge_data_display_span = 20000;

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  }; // end of:  class Bno08xHardwareInterface : public hardware_interface::SensorInterface

} // namespace bno08x_hardware_interface

#endif // BNO08X_HARDWARE_INTERFACE__BNO08XHARDWAREINTERFACE_HPP_
