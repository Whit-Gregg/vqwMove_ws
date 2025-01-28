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

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "roboclaw_serial/command.hpp"
#include "roboclaw_serial/crc.hpp"
#include "roboclaw_serial/device.hpp"
#include "roboclaw_serial/serialized_buffer.hpp"

namespace roboclaw_serial
{
    class Interface
    {
      public:
        typedef std::shared_ptr<Interface> SharedPtr;
        Interface() = default;
        explicit Interface(const SerialDevice::SharedPtr &device) { setDevice(device); }

        void setDevice(const SerialDevice::SharedPtr &device) { device_ = device; }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Request>
        void read(Request &request, const unsigned char address = 128)
        {
            request.fields = read<Request>(address);
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Request>
        typename Request::ArgsTuple read(const unsigned char address = 128)
        {
            // Prevent parallel reads/writes
            std::lock_guard<std::mutex> lock(mutex_);

            this->bufferSetupRead<Request>(address);

            auto cmd = Request::read_command;
            crc_     = 0;
            for (const auto &byte : buffer_) { crc16::update(crc_, byte); }
            //-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`
            bool isException = false;
            try
                {
                    // Write the buffer to the serial device
                    ssize_t sz = device_->write(buffer_.data(), buffer_.size());
                }
            catch (const std::exception &e)
                {
                    auto log = rclcpp::get_logger("RoboclawSerialInterface");
                    RCLCPP_ERROR(log, "roboclaw_serial::Interface::read(0x%02X,..) device_->write() exception: %s", cmd, e.what());
                    isException = true;
                }
            //-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`

            // Set the buffer to the size of the fields, size of CRC
            buffer_.resize(buffer_.max_size());

            //-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`
            std::size_t bytes_read = 0;
            try
                {
                    // Read the response from the device
                    if (!isException) { bytes_read = device_->read(buffer_.data(), buffer_.size()); }
                }
            catch (const std::exception &e)
                {
                    auto log = rclcpp::get_logger("RoboclawSerialInterface");
                    RCLCPP_ERROR(log, "roboclaw_serial::Interface::read(0x%02X,..) device_->read() exception: %s", cmd, e.what());
                    isException = true;
                }
            //-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`
            uint16_t recv_crc = 0xe2a8;

            if ((bytes_read > 2) && (!isException))
                {
                    buffer_.resize(bytes_read);

                    // Extract the CRC from the the back of the buffer
                    recv_crc = buffer_.pop_back<uint16_t>();

                    // Update the CRC with the new data and ensure it matches
                    for (auto byte : buffer_) { crc16::update(crc_, byte); }
                }

            typename Request::ArgsTuple fields;

            // int field_count = std::tuple_size<typename Request::ArgsTuple>::value;

            if ((crc_ != recv_crc) && (!isException))
                {
                    consecutive_crc_errors_++;
                    total_crc_errors_++;

                    // this delay cauases the Roboclaw to clear it's buffer
                    int64_t delay_millis = 10;
                    rclcpp::sleep_for(std::chrono::milliseconds(delay_millis));

                    if ((total_crc_errors_ < 5) || ((total_crc_errors_ % total_crc_errors_MOD_) == 0))
                        {
                            total_crc_errors_MOD_    = total_crc_errors_MOD_ + 100;
                            double crc_error_percent = ((double)total_crc_errors_ / (double)total_reads_) * 100.0;
                            auto   log               = rclcpp::get_logger("RoboclawSerialInterface");
                            RCLCPP_INFO(log, " read(0x%02X,..) CRCs do not match, consec=%d, total=%d out of %ld total reads (%.3f%%)", cmd,
                                        consecutive_crc_errors_, total_crc_errors_, total_reads_, crc_error_percent);
                        }
                    device_->restart();
                }
            else
                {
                    if (!isException)
                        {
                            consecutive_crc_errors_ = 0;
                            total_reads_++;
                            try
                                {
                                    // Extract the data
                                    std::apply([&](auto &&...args) { buffer_.unpack(args...); }, fields);
                                }
                            catch (const std::exception &e)
                                {
                                    auto log = rclcpp::get_logger("RoboclawSerialInterface");
                                    RCLCPP_ERROR(log, "roboclaw_serial::Interface::read(0x%02X,..) buffer_.unpack() exception: %s", cmd, e.what());
                                }
                        }
                }
            return fields;
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        // template <typename Request, typename...Args>
        // int write(Args&&...args)
        // {
        //     // Convert to tuple and use tuple-based write call
        //     return write<Request>(std::forward_as_tuple(args...));
        // }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Request>
        void write(const Request &request, const unsigned char address = 128)
        {
            // Write the fields to the roboclaw
            write<Request>(request.fields, address);
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Request>
        void write(const typename Request::ArgsTuple &fields, const unsigned char address = 128)
        {
            // Prevent parallel read/writes
            std::lock_guard<std::mutex> lock(mutex_);

            auto cmd = Request::write_command;

            // Initialize buffer with Write request, fields, and CRC
            this->bufferSetupWrite<Request>(address, fields);

            // Write the request
            ssize_t bytes_written = device_->write(buffer_.data(), buffer_.size());

            if (!this->readAck())
                {
                    consecutive_ack_errors_++;
                    total_ack_errors_++;

                    // this delay cauases the Roboclaw to clear it's buffer
                    int64_t delay_millis = 10;
                    rclcpp::sleep_for(std::chrono::milliseconds(delay_millis));

                    if ((total_ack_errors_ < 5) || ((total_ack_errors_ % 10) == 0))
                        {
                            double ack_error_percent = ((double)total_ack_errors_ / (double)total_writes_) * 100.0;
                            auto   log               = rclcpp::get_logger("RoboclawSerialInterface");
                            RCLCPP_INFO(log, "write( 0x%02X,..) did not get an ACK, consec=%d, total_err=%d total_good=%d  (%.3f %%)", cmd,
                                        consecutive_ack_errors_, total_ack_errors_, total_good_acks_, ack_error_percent);
                        }
                    device_->restart();
                }
            else
                {
                    consecutive_ack_errors_ = 0;
                    total_good_acks_++;
                    total_writes_++;
                }
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
      private:
        template<typename Request>
        void bufferPackFields(const typename Request::ArgsTuple &fields)
        {
            auto pushToBuffer = [this](const auto &...items) { (this->buffer_.push_back(items), ...); };

            std::apply(pushToBuffer, fields);
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Request>
        void bufferUnpackFields(typename Request::ArgsTuple &fields)
        {
            std::apply([this](auto &&...args) { ((args = this->buffer_.pop_front<std::decay_t<decltype(args)>>()), ...); }, fields);
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        // Compute the CRC
        void bufferAddCRC16()
        {
            // Calculate the CRC16 value
            crc_ = 0;
            for (const auto &byte : buffer_) { crc16::update(crc_, byte); }

            // Add the CRC6 to the end of the buffer
            buffer_.push_back(crc_);
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        // Setup the buffer for a write request
        template<typename Request>
        void bufferSetupWrite(const unsigned char address, const typename Request::ArgsTuple &fields)
        {
            // Set size of the buffer to 0
            buffer_.clear();

            // Serialize request into buffer
            buffer_.push_back(address);
            buffer_.push_back(Request::write_command);

            // Pack the tuple into the buffer
            this->bufferPackFields<Request>(fields);

            // Add the CRC to the buffer
            this->bufferAddCRC16();
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        // Setup the buffer for a read request
        template<typename Request>
        void bufferSetupRead(const unsigned char address)
        {
            // Set size of the buffer to 0
            buffer_.clear();

            // Serialize request into buffer
            buffer_.push_back(address);
            buffer_.push_back(Request::read_command);
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        bool readAck()
        {
            // We only expect an ACK from the roboclaw
            buffer_.resize(1);
            device_->read(buffer_.data(), buffer_.size());

            return buffer_.pop_back() == ACK;
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        const std::byte         ACK = std::byte(255U);
        uint16_t                crc_;
        SerialDevice::SharedPtr device_;
        SerializedBuffer<64>    buffer_;
        std::mutex              mutex_;

        long total_reads_             = 0;
        long total_writes_            = 0;
        int  consecutive_crc_errors_  = 0;
        int  total_crc_errors_        = 0;
        int  total_crc_errors_MOD_    = 50;
        int  consecutive_read_errors_ = 0;
        int  total_read_errors_       = 0;
        int  consecutive_ack_errors_  = 0;
        int  total_ack_errors_        = 0;
        int  total_good_acks_         = 0;

        const int max_consecutive_errors_                = 30000;
        const int max_consecutive_errors_before_restart_ = 2;
    };

}       // namespace roboclaw_serial
