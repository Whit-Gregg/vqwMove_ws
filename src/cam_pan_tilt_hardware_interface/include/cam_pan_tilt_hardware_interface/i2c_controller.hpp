#pragma once
#include <array>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <memory>
#include <sstream>
#include <sys/ioctl.h>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <vector>

// see https://www.kernel.org/doc/Documentation/i2c/dev-interface

/*It may possible be deleted...*/
// #include "cam_pan_tilt_hardware_interface/metafunctions.h"

// const std::string channel_name_form = "/dev/i2c-";

namespace cam_pan_tilt_hardware_interface
{

    class I2CController
    {
      protected:
        std::string  dev_name;
        int          port_handle = -1;
        std::uint8_t slave_address;
        enum class Status
        {
            NOT_OPENED,
            IDLE,
            BUSY,
        };
        Status status = Status::NOT_OPENED;

      public:
        I2CController() = default;
        I2CController(const std::string &_dev_name, uint8_t slave__address) : dev_name(_dev_name), slave_address(slave__address)  {}
        I2CController(const I2CController &) = delete;
        I2CController(const I2CController && other) = delete;
        virtual ~I2CController();

        // returns 1 on success, -1 on failure
        int open_port(const std::string &_dev_name, uint8_t slave__address);

        // returns 1 on success, -1 on failure
        int open_port() { return open_port(dev_name, slave_address); }

        bool   isOpen() { return port_handle != -1; }

        // returns 1 on success, -1 on failure
        int  close_port();

        // returns 1 on success, -1 on timeout
        int wait_for_being_idle();

        std::string get_device_name() { return dev_name; }
        int get_i2c_address() { return slave_address; }

        // returns 1 on success, -1 on failure
        int  set_slave_address(const std::uint8_t _slave_address);

        // returns 1 on success, -1 on failure
        int write_reg(const uint8_t _reg, const uint8_t *_data, size_t _len);

        // returns 1 on success, -1 on failure
        int read_reg(const uint8_t _reg, uint8_t *_data, size_t _len);

        using SharedPtr = std::shared_ptr<I2CController>;
    };

}       // namespace cam_pan_tilt_hardware_interface