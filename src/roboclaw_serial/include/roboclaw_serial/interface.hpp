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

#include "elapsedMillis.hpp"
#include "value_distribution.hpp"

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
            request.fields = read<Request>(read_timeout_ms_, address);
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Request>
        typename Request::ArgsTuple read(const long read_timeout, const unsigned char address = 128)
        {
            // Prevent parallel reads/writes
            std::lock_guard<std::mutex> lock(mutex_);

            bool isDataValid = false;
            typename Request::ArgsTuple fields;

            this->bufferSetupRead<Request>(address);

            auto cmd = Request::read_command;
            crc_     = 0;
            for (const auto &byte : buffer_) { crc16::update(crc_, byte); }
            //-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`-=+~`
            bool isException = false;
            bool isTimeOut   = false;
            try
                {
                    // Write the buffer to the serial device
                    ssize_t sz = device_->write(buffer_.data(), buffer_.size());
                    write_size_distribution.addValue(sz);
                    total_bytes_written_ += sz;
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
            if (!isException)
                {
                    try
                        {
                            std::size_t   read_size            = getReadSize(fields);
                            std::size_t   bytes_read_this_time = 0;
                            elapsedMillis read_elap;
                            // Read the response from the device
                            while ((bytes_read < read_size) && (read_elap < read_timeout))
                                {
                                    const std::byte *buffer = buffer_.data() + bytes_read;
                                    bytes_read_this_time    = device_->read(const_cast<std::byte *>(buffer), read_size - bytes_read);
                                    read_size_distribution.addValue(bytes_read_this_time);
                                    total_bytes_read_ += bytes_read_this_time;
                                    bytes_read += bytes_read_this_time;
                                }
                            if (read_elap >= read_timeout)
                                {
                                    isTimeOut = true;
                                    // auto log = rclcpp::get_logger("RoboclawSerialInterface");
                                    // RCLCPP_ERROR(log, "roboclaw_serial::Interface::read(0x%02X,..) read timeout after %ld ms", cmd, read_timeout);
                                }
                        }
                    catch (const std::exception &e)
                        {
                            auto log = rclcpp::get_logger("RoboclawSerialInterface");
                            RCLCPP_ERROR(log, "roboclaw_serial::Interface::read(0x%02X,..) device_->read() exception: %s", cmd, e.what());
                            isException = true;
                        }
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


            // int field_count = std::tuple_size<typename Request::ArgsTuple>::value;

            if ((crc_ != recv_crc) && (!isException))
                {
                    consecutive_crc_errors_++;
                    total_crc_errors_++;

                    // this delay cauases the Roboclaw to clear it's buffer
                    int64_t delay_millis = 10;
                    rclcpp::sleep_for(std::chrono::milliseconds(delay_millis));

                    if ((total_crc_errors_ < 50) || ((total_crc_errors_ % total_crc_errors_MOD_) == 0))
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
                                    isDataValid = true;
                                }
                            catch (const std::exception &e)
                                {
                                    auto log = rclcpp::get_logger("RoboclawSerialInterface");
                                    RCLCPP_ERROR(log, "roboclaw_serial::Interface::read(0x%02X,..) buffer_.unpack() exception: %s", cmd, e.what());
                                }
                        }
                }

            if ((crc_ != recv_crc) || (isException))
                {
                    long bytes_read_since_last_error    = total_bytes_read_ - total_bytes_read_at_last_error_;
                    long bytes_written_since_last_error = total_bytes_written_ - total_bytes_written_at_last_error_;

                    auto log = rclcpp::get_logger("RoboclawSerialInterface");
                    RCLCPP_ERROR(log,
                                 "roboclaw_serial::Interface::read(0x%02X,..) CRC error or exception, bytes_read=%ld, bytes_written=%ld, "
                                 "bytes_read_since_last_error=%ld, bytes_written_since_last_error=%ld",
                                 cmd, total_bytes_read_, total_bytes_written_, bytes_read_since_last_error, bytes_written_since_last_error);

                    total_bytes_read_at_last_error_    = total_bytes_read_;
                    total_bytes_written_at_last_error_ = total_bytes_written_;
                }
            // if (!isDataValid)
            //     {
            //         int field_count = std::tuple_size<typename Request::ArgsTuple>::value;
            //         for (int i = 0; i < field_count; i++) { fields[i] = 0; }
            //     }
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
            ssize_t buf_size = buffer_.size();
            ssize_t bytes_written = device_->write(buffer_.data(), buf_size);
            write_size_distribution.addValue(buf_size);
            total_bytes_written_ += buf_size;

            // if (cmd == (uint8_t)Command::DRIVE_M1_M2_SGN_SPD)
            //     {
            //         auto log = rclcpp::get_logger("RoboclawSerialInterface");
            //         std::string request_str = this->getRequestAsString<Request>(fields);
            //         RCLCPP_INFO(log, "roboclaw_serial::Interface::write(...) cmd:0x%02X  request: %s", cmd, request_str.c_str());
            //     }

            // // // // // // // delay 15 ms to allow the roboclaw to process the request
            // // // // // // // rclcpp::sleep_for(std::chrono::milliseconds(15));

            bool ACK_OK = this->readAck();
            if (!ACK_OK)
                {
                    consecutive_ack_errors_++;
                    total_ack_errors_++;

                    // // this delay cauases the Roboclaw to clear it's buffer
                    // int64_t delay_millis = 10;
                    // rclcpp::sleep_for(std::chrono::milliseconds(delay_millis));

                    if ((total_ack_errors_ < 50) || ((total_ack_errors_ % 50) == 0))
                        {
                            double ack_error_percent = ((double)total_ack_errors_ / (double)total_writes_) * 100.0;
                            auto   log               = rclcpp::get_logger("RoboclawSerialInterface");
                            RCLCPP_INFO(log, "write( 0x%02X,..) did not get an ACK, consec=%d, total_err=%d total_good=%d  (%.3f %%)", cmd,
                                        consecutive_ack_errors_, total_ack_errors_, total_good_acks_, ack_error_percent);
                        }

                    long bytes_read_since_last_error    = total_bytes_read_ - total_bytes_read_at_last_error_;
                    long bytes_written_since_last_error = total_bytes_written_ - total_bytes_written_at_last_error_;

                    auto log = rclcpp::get_logger("RoboclawSerialInterface");
                    RCLCPP_ERROR(log,
                                 "roboclaw_serial::Interface::write(0x%02X,..) CRC error or exception, bytes_read=%ld, bytes_written=%ld, "
                                 "bytes_read_since_last_error=%ld, bytes_written_since_last_error=%ld",
                                 cmd, total_bytes_read_, total_bytes_written_, bytes_read_since_last_error, bytes_written_since_last_error);

                    total_bytes_read_at_last_error_    = total_bytes_read_;
                    total_bytes_written_at_last_error_ = total_bytes_written_;

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
        void dump_distributions()
        {
            auto log = rclcpp::get_logger("RoboclawSerialInterface");
            RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()-------------");

            // read_size_distribution
            RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()---  read_size_distribution  ----------");
            for (int x = 0; x < read_size_distribution.MAX_VALUE; x++)
                {
                    long count = read_size_distribution.getValue(x);
                    if (count > 0) { RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()---  read_size_distribution[%d] = %ld", x, count); }
                }
            RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()---  write_size_distribution  ----------");
            for (int x = 0; x < write_size_distribution.MAX_VALUE; x++)
                {
                    long count = write_size_distribution.getValue(x);
                    if (count > 0) { RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()---  write_size_distribution[%d] = %ld", x, count); }
                }

            RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()---  read_ACK_time_distribution  ----------");
            for (int x = 0; x < read_ACK_time_distribution.MAX_VALUE; x++)
                {
                    long count = read_ACK_time_distribution.getValue(x);
                    if (count > 0) { RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()---  read_ACK_time_distribution[%d] = %ld", x, count); }
                }
            RCLCPP_INFO(log, "roboclaw_serial::Interface::dump_distributions()------------- done -------------------");
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
        bool readAck(long timeout_ms = 50)
        {
            // We only expect an ACK from the roboclaw
            buffer_.resize(1);
            int           bytes_read = 0;
            elapsedMillis read_elap;
            while ((bytes_read < 1) && (read_elap < timeout_ms))
                {
                    bytes_read = device_->read(buffer_.data(), buffer_.size());
                    // Wait for the ACK
                    if (bytes_read < 1)
                        {
                            // If we don't get an ACK, wait a bit and try again
                            // This is to allow the roboclaw to process the request and send the ACK
                            rclcpp::sleep_for(std::chrono::milliseconds(2));
                        }
                }
            long read_time = read_elap;
            read_ACK_time_distribution.addValue(read_time);
            total_bytes_read_ += bytes_read;
            bool retval = false;
            if ((bytes_read > 0) && (buffer_.pop_back() == ACK)) { retval = true; }
            return retval;
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Request>
        std::string getRequestAsString(const typename Request::ArgsTuple &fields)
        {
            std::ostringstream oss;
            //oss << "Request: " << Request::name << " | ";
            std::apply([&oss](const auto&... tupleArgs) {
                ((oss << tupleArgs << " | "), ...);
            }, fields);
            return oss.str();
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        template<typename Tuple_t>
        int getReadSize(const Tuple_t &tup)
        {
            int  sz       = 0;
            std::apply([&sz](const auto&... tupleArgs) {
                auto sumSizes = [&sz](const auto &item) {
                    sz += sizeof(item);
                };
                (sumSizes(tupleArgs), ...);
                // or use fold expression in C++17
                // std::apply([&sz](const auto &...tupleArgs) {
                //     (sz += sizeof(tupleArgs), ...);
                // });
            }, tup);

            // auto sumSizes = [this](const auto &...items) { (sz += sizeof(items), ...); };
            // std::apply(sumSizes, fields);
            return sz + 2;       // 2 for the crc
        }

        //-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~-=+^~
        const std::byte         ACK              = std::byte(255U);
        const long              read_timeout_ms_ = 50;       // Default read timeout in milliseconds
        uint16_t                crc_;
        SerialDevice::SharedPtr device_;
        SerializedBuffer<64>    buffer_;
        std::mutex              mutex_;

        long total_reads_                       = 0;
        long total_writes_                      = 0;
        long total_bytes_written_               = 0;
        long total_bytes_read_                  = 0;
        long total_bytes_written_at_last_error_ = 0;
        long total_bytes_read_at_last_error_    = 0;

        // Error counters
        int consecutive_crc_errors_  = 0;
        int total_crc_errors_        = 0;
        int total_crc_errors_MOD_    = 50;
        int consecutive_read_errors_ = 0;
        int total_read_errors_       = 0;
        int consecutive_ack_errors_  = 0;
        int total_ack_errors_        = 0;
        int total_good_acks_         = 0;

        const int max_consecutive_errors_                = 30000;
        const int max_consecutive_errors_before_restart_ = 2;

        vqw::ValueDistribution read_size_distribution;
        vqw::ValueDistribution write_size_distribution;
        vqw::ValueDistribution read_ACK_time_distribution;
    };

}       // namespace roboclaw_serial
