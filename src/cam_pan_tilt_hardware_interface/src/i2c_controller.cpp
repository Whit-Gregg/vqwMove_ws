#include "cam_pan_tilt_hardware_interface/i2c_controller.hpp"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>


/*
uint8_t command[] = {
    0xE0,
    (uint8_t)(target >> 0  & 0xFF),
    (uint8_t)(target >> 8  & 0xFF),
    (uint8_t)(target >> 16 & 0xFF),
    (uint8_t)(target >> 24 & 0xFF),
  };
  struct i2c_msg message = { address, 0, sizeof(command), command };
  struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);


   uint8_t command[] = { 0xA1, offset };
  struct i2c_msg messages[] = {
    { address, 0, sizeof(command), command },
    { address, I2C_M_RD, length, buffer },
  };
  struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
*/


namespace cam_pan_tilt_hardware_interface
{

I2CController::~I2CController() { close_port(); }

int I2CController::open_port(const std::string &_dev_name, uint8_t slave__address)
{
    if (isOpen()) { close_port(); }
    dev_name    = _dev_name;
    port_handle = open(dev_name.c_str(), O_RDWR);
    if (port_handle < 0)
        {
            status = Status::NOT_OPENED;
            RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::open_port() Failed. dev_name=%s errno=%d  %s", dev_name.c_str(),
                         errno, strerror(errno));
            return -1;
        }
    status = Status::IDLE;
    set_slave_address(slave__address);
    return 1;
}

int I2CController::write_reg(const uint8_t _reg, const uint8_t *_data, size_t _len)
{
    if (!isOpen())
        {
            RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::write_reg() Port is not opened");
            return -1;
        }
    int     result;
    uint8_t buffer[64];
    if (_len > 60) { _len=60; }
    buffer[0] = _reg;
    for (size_t i = 0; i < _len; i++) { buffer[i + 1] = _data[i]; }
    wait_for_being_idle();
    status = Status::BUSY;
    result = write(port_handle, buffer, _len + 1);
    status = Status::IDLE;
    if (result < 0) { 
        RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::write_reg() write() ERROR. errno=%d  %s", errno, strerror(errno));
        //....
         }
    return result;
}

int I2CController::read_reg(const uint8_t _reg, uint8_t *_data, size_t _len)
{
    if (!isOpen())
        {
            RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::read_reg() Port is not opened");
            return -1;
        }
    uint8_t buffer_send[1] {_reg};

    struct i2c_msg msg[2];

    msg[0].addr  = slave_address;
    msg[0].flags = 0;
    msg[0].len   = 1;
    msg[0].buf   = buffer_send;

    msg[1].addr  = slave_address;
    msg[1].flags = I2C_M_RD;
    msg[1].len   = _len;
    msg[1].buf   = _data;

    struct i2c_rdwr_ioctl_data msg_set
    {
        msg, 2
    };

    wait_for_being_idle();
    status     = Status::BUSY;
    int result = ioctl(port_handle, I2C_RDWR, &msg_set);
    if (result < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::read_reg() ioctl(..) Failed. errno=%d  %s", errno, strerror(errno));
        }
    status = Status::IDLE;
    return result;
}

int I2CController::close_port()
{
    if (!isOpen()) { return -1; }
    wait_for_being_idle();
    close(port_handle);
    port_handle = -1;
    status      = Status::NOT_OPENED;
    return 1;
}

int I2CController::wait_for_being_idle()
{
    if (status != Status::BUSY) { return 1; }
    RCLCPP_INFO(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::wait_for_being_idle() status == BUSY");
    int count = 0;
    while ((status == Status::BUSY) && (count++ < 1000)) { 
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        //....
         }
    if (status == Status::BUSY) { 
        RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::wait_for_being_idle() Failed. status == BUSY");
        return -1;
    }
    return 1;
}

int I2CController::set_slave_address(const std::uint8_t _slave_address)
{
    int rc;
    rc = ioctl(port_handle, I2C_SLAVE, _slave_address);
    if (rc < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "I2CController::set_slave_address() Failed. errno=%d  %s", errno,
                         strerror(errno));
            return -1;
        }
    slave_address = _slave_address;
    return 1;
}


} // namespace cam_pan_tilt_hardware_interface