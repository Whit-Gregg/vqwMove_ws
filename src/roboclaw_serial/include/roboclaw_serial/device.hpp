// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace roboclaw_serial
{

    class SerialDevice
    {
      public:
        typedef std::shared_ptr<SerialDevice> SharedPtr;

        SerialDevice() = default;

        explicit SerialDevice(const std::string device, int read_timeout_ms = 100)
        {
            set_read_timeout(read_timeout_ms);
            connect(device);
        }
        virtual ~SerialDevice() { disconnect(); }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        virtual bool connect(const std::string &device)
        {
            // RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "connect(%s)...", device.c_str());
            if (fd_ != -1) { close(fd_); }
            fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);

            if (fd_ != -1)
                {
                    device_name_ = device;
                    setSerialDeviceOptions();
                }
            else { RCLCPP_ERROR(rclcpp::get_logger("RoboclawSerialDevice"), "connect(\"%s\") FAILED!! errno=%d  %s", device.c_str(), errno, strerror(errno)); }

            info_count++;
            if ((info_count < 10) || (info_count % info_count_span == 0) || (fd_ == -1))
                {
                    RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "connect(%s) == %s     total trys=%ld", device.c_str(),
                                ((fd_ != -1) ? "OK" : "FAILED"), info_count);
                    info_count_span += info_count_span / 9;
                }
            return fd_ != -1;
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        virtual void disconnect()
        {
            if (fd_ != -1)
                {
                    close(fd_);
                    fd_ = -1;
                }
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        bool restart()
        {
            // RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "ReStarting...........%s", device_name_.c_str());
            disconnect();
            return connect(device_name_);
        }

        bool connected() const { return fd_ != -1; }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        virtual ssize_t write(const std::byte *buffer, std::size_t count)
        {
            ssize_t result = ::write(fd_, buffer, count);
            if (result < 0)
                {
                    // Error writing to device
                    throw std::range_error("Error writing to the serial device!");
                }
            return static_cast<ssize_t>(result) == count;
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        virtual ssize_t read(std::byte *buffer, std::size_t count)
        {
            memset(buffer, 0, count);

            ssize_t result = ::read(fd_, buffer, count);
            if (result < 0)
                {
                    // Error reading from the device
                    throw std::range_error("Error reading from the serial device!");
                }
            return result;
            //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        void set_read_timeout(int timeout_ms) { read_timeout_ms_ = timeout_ms; }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

      private:
        void setSerialDeviceOptions()
        {
            struct termios options;
            int            rc = tcgetattr(fd_, &options);
            if (rc != 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RoboclawSerialDevice"), "setSerialDeviceOptions() !! FAILED !!   tcgetattr  rc=%d  errno=%d  %s", rc,
                                 errno, strerror(errno));
                }

            cfmakeraw(&options);
            cfsetspeed(&options, B38400);

            options.c_cc[VMIN]  = 0;
            options.c_cc[VTIME] = (cc_t)read_timeout_ms_ / 100;

            rc = tcsetattr(fd_, TCSANOW, &options);
            if (rc != 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RoboclawSerialDevice"), "setSerialDeviceOptions() !! FAILED !!   tcsetattr  rc=%d  errno=%d  %s", rc,
                                 errno, strerror(errno));
                }

            // // // const int read_timeout_ms = 100;
            // // // struct termios options;
            // // // int rc = tcgetattr(fd_, &options);

            // // // options.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
            // // // options.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
            // // // options.c_cflag &= ~CSIZE;         // Clear all the size bits
            // // // options.c_cflag |= CS8;            // 8 bits per byte (most common)
            // // // options.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
            // // // options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

            // // // options.c_lflag &= ~ICANON; // Disable canonical mode
            // // // options.c_lflag &= ~ECHO;   // Disable echo
            // // // options.c_lflag &= ~ECHOE;  // Disable erasure
            // // // options.c_lflag &= ~ECHONL; // Disable new-line echo
            // // // options.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

            // // // options.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
            // // // options.c_iflag &= ~(ICRNL | INLCR | IGNBRK | BRKINT | PARMRK | ISTRIP | IGNCR); // Turn off translation of carriage return and newline

            // // // options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            // // // options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

            // // // // Set in/out baud rate to be 9600
            // // // cfsetispeed(&options, B38400);
            // // // cfsetospeed(&options, B38400);

            // // // options.c_cc[VMIN] = 0;
            // // // options.c_cc[VTIME] = (cc_t)read_timeout_ms / 100;

            // // // tcflush(fd_, TCIFLUSH);
            // // // rc = tcsetattr(fd_, TCSANOW, &options);
            // // // if (rc != 0)
            // // // {
            // // //   perror("tcsetattr");
            // // // }
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

        int         fd_ = -1;
        std::string device_name_;
        int         read_timeout_ms_ = 100;       // Default read timeout in milliseconds
        long        info_count       = 0;
        long        info_count_span  = 100;
    };

}       // namespace roboclaw_serial
