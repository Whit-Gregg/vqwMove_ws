#include <cstdio>
#include <cstddef>

// #include <roboclaw_serial/device.hpp>
// #include <roboclaw_serial/command.hpp>
//  #include <../../roboclaw_serial/include/roboclaw_serial/device.hpp>
//  #include<../../../roboclaw_serial/device.hpp>
//  #include <../../roboclaw_serial/include/roboclaw_serial/command.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sys/ioctl.h>



// // // // // namespace roboclaw_serial
// // // // // {

// // // // //     enum class Command : unsigned char
// // // // //     {
// // // // //         DRIVE_M1_FWD = 0,                        // Drive Forward Motor 1
// // // // //         DRIVE_M1_BWD = 1,                        // Drive Backwards Motor 1
// // // // //         SET_MAIN_VOLT_MIN = 2,                   // Set Main Voltage Minimum
// // // // //         SET_MAIN_VOLT_MAX = 3,                   // Set Main Voltage Maximum
// // // // //         DRIVE_M2_FWD = 4,                        // Drive Forward Motor 2
// // // // //         DRIVE_M2_BWD = 5,                        // Drive Backwards Motor 2
// // // // //         DRIVE_M1_7BIT = 6,                       // Drive Motor 1 (7 Bit)
// // // // //         DRIVE_M2_7BIT = 7,                       // Drive Motor 2 (7 Bit)
// // // // //         DRIVE_FWD_MIXED = 8,                     // Drive Forward Mixed Mode
// // // // //         DRIVE_BWD_MIXED = 9,                     // Drive Backwards Mixed Mode
// // // // //         TURN_RIGHT_MIXED = 10,                   // Turn Right Mixed Mode
// // // // //         TURN_LEFT_MIXED = 11,                    // Turn Left Mixed Mode
// // // // //         DRIVE_7BIT = 12,                         // Drive Forward or Backward (7 bit)
// // // // //         TURN_7BIT = 13,                          // Turn Left or Right (7 Bit)
// // // // //         SET_SERIAL_TIMEOUT = 14,                 // Set Serial Timeout
// // // // //         READ_SERIAL_TIMEOUT = 15,                // Read Serial Timeout
// // // // //         READ_M1_ENC = 16,                        // Read Encoder Count/Value for M1
// // // // //         READ_M2_ENC = 17,                        // Read Encoder Count/Value for M2
// // // // //         READ_M1_ENC_SPD = 18,                    // Read M1 Speed in Encoder Counts Per Second
// // // // //         READ_M2_ENC_SPD = 19,                    // Read M2 Speed in Encoder Counts Per Second
// // // // //         RESET_M1_M2_ENC_REG = 20,                // Resets Encoder Registers for M1 and M2 (Quadrature only)
// // // // //         READ_FIRMWARE_VER = 21,                  // Read Firmware Version
// // // // //         SET_ENC1_REG = 22,                       // Set Encoder 1 Register (Quadrature only)
// // // // //         SET_ENC2_REG = 23,                       // Set Encoder 2 Register (Quadrature only)
// // // // //         READ_MAIN_BATT_VOLT = 24,                // Read Main Battery Voltage
// // // // //         READ_LOGIC_BATT_VOLT = 25,               // Read Logic Battery Voltage
// // // // //         SET_MIN_LOGIC_VOLT = 26,                 // Set Minimum Logic Voltage Level
// // // // //         SET_MAX_LOGIC_VOLT = 27,                 // Set Maximum Logic Voltage Level
// // // // //         SET_M1_VEL_PID_CONST = 28,               // Set Velocity PID Constants for M1
// // // // //         SET_M2_VEL_PID_CONST = 29,               // Set Velocity PID Constants for M2
// // // // //         READ_M1_RAW_SPD = 30,                    // Read Current M1 Raw Speed
// // // // //         READ_M2_RAW_SPD = 31,                    // Read Current M2 Raw Speed
// // // // //         DRIVE_M1_SGN_DUTY = 32,                  // Drive M1 With Signed Duty Cycle (Encoders not required)
// // // // //         DRIVE_M2_SGN_DUTY_CYCLE = 33,            // Drive M2 with Signed Duty Cycle (Encoders not required)
// // // // //         DRIVE_M1_M2_SGN_DUTY_CYCLE = 34,         // Drive M1/M2 with Signed Duty Cycle (Encoders not required)
// // // // //         DRIVE_M1_SGN_SPD = 35,                   // Drive M1 with Signed Speed
// // // // //         DRIVE_M2_SGN_SPD = 36,                   // Drive M2 with Signed Speed
// // // // //         DRIVE_M1_M2_SGN_SPD = 37,                // Drive M1/M2 with Signed Speed
// // // // //         DRIVE_M1_SGN_SPD_ACCEL = 38,             // Drive M1 with Signed Speed and Acceleration
// // // // //         DRIVE_M2_SGN_SPD_ACCEL = 39,             // Drive M2 with Signed Speed and Acceleration
// // // // //         DRIVE_M1_M2_SGN_SPD_ACCEL = 40,          // Drive M1/M2 with Signed Speed and Acceleration
// // // // //         DRIVE_M1_SGN_SPD_DIST_BUF = 41,          // Drive M1 with Signed Speed and Distance (Buffered)
// // // // //         DRIVE_M2_SGN_SPD_DIST_BUF = 42,          // Drive M2 with Signed Speed and Distance (Buffered)
// // // // //         DRIVE_M1_M2_SGN_SPD_DIST_BUF = 43,       // Drive M1/M2 with Signed Speed and Distance (Buffered)
// // // // //         DRIVE_M1_SGN_SPD_ACCEL_DIST_BUF = 44,    // Drive M1 with Signed Speed, Acceleration and Distance (Buffered)
// // // // //         DRIVE_M2_SGN_SPD_ACCEL_DIST_BUF = 45,    // Drive M2 with Signed Speed, Acceleration and Distance (Buffered)
// // // // //         DRIVE_M1_M2_SGN_SPD_ACCEL_DIST_BUF = 46, // Drive M1/M2 with Signed Speed, Acceleration and Distance (Buffered)
// // // // //         READ_BUF_LEN = 47,                       // Read Buffer Length
// // // // //         READ_M1_M2_PWM = 48,                     // Read Motor PWMs
// // // // //         READ_M1_M2_CURRENT = 49,                 // Read Motor Currents
// // // // //         DRIVE_M1_M2_SGN_SPD_ACCEL_IND = 50,      // Drive M1/M2 with Individual Signed Speed and Acceleration
// // // // //         DRIVE_M1_M2_SGN_SPD_ACCEL_DIST_IND = 51, // Drive M1/M2 with Individual Signed Speed, Accel and Distance
// // // // //         DRIVE_M1_SGN_DUTY_ACCEL = 52,            // Drive M1 with Signed Duty and Accel (Encoders not required)
// // // // //         DRIVE_M2_SGN_DUTY_ACCEL = 53,            // Drive M2 with Signed Duty and Accel (Encoders not required)
// // // // //         DRIVE_M1_M2_SGN_DUTY_ACCEL = 54,         // Drive M1/M2 with Signed Duty and Accel (Encoders not required)
// // // // //         READ_M1_VEL_PID_CONST = 55,              // Read Motor 1 Velocity PID Constants
// // // // //         READ_M2_VEL_PID_CONST = 56,              // Read Motor 2 Velocity PID Constants
// // // // //         SET_MAIN_BATT_VOLT = 57,                 // Set Main Battery Voltages
// // // // //         SET_LOGIC_BATT_VOLT = 58,                // Set Logic Battery Voltages
// // // // //         READ_MAIN_BATT_VOLT_SET = 59,            // Read Main Battery Voltage Settings
// // // // //         READ_LOGIC_BATT_VOLT_SET = 60,           // Read Logic Battery Voltage Settings
// // // // //         SET_POS_PID_CONST_M1 = 61,               // Set Position PID Constants for M1
// // // // //         SET_POS_PID_CONST_M2 = 62,               // Set Position PID Constants for M2
// // // // //         READ_M1_POS_PID_CONST = 63,              // Read Motor 1 Position PID Constants
// // // // //         READ_M2_POS_PID_CONST = 64,              // Read Motor 2 Position PID Constants
// // // // //         DRIVE_M1_SPD_ACCEL_DECEL_POS = 65,       // Drive M1 with Speed, Accel, Deccel and Position
// // // // //         DRIVE_M2_SPD_ACCEL_DECEL_POS = 66,       // Drive M2 with Speed, Accel, Deccel and Position
// // // // //         DRIVE_M1_M2_SPD_ACCEL_DECEL_POS = 67,    // Drive M1/M2 with Speed, Accel, Deccel and Position
// // // // //         SET_M1_DUTY_ACCEL = 68,                  // Set default duty cycle acceleration for M1
// // // // //         SET_M2_DUTY_ACCEL = 69,                  // Set default duty cycle acceleration for M2
// // // // //         SET_M1_DEF_SPEED = 70,                   // Set Default Speed for M1
// // // // //         SET_M2_DEF_SPEED = 71,                   // Set Default Speed for M2
// // // // //         READ_DEF_SPEED_SET = 72,                 // Read Default Speed Settings
// // // // //         SET_S3S4S5_MODES = 74,                   // Set S3,S4 and S5 Modes
// // // // //         READ_S3S4S5_MODES = 75,                  // Read S3,S4 and S5 Modes
// // // // //         SET_DEADBAND_RC = 76,                    // Set DeadBand for RC/Analog controls
// // // // //         READ_DEADBAND_RC = 77,                   // Read DeadBand for RC/Analog controls
// // // // //         READ_M1_M2_ENC = 78,                     // Read Encoder Counts
// // // // //         READ_M1_M2_RAW_SPD = 79,                 // Read Raw Motor Speeds
// // // // //         RESTORE_DEFAULTS = 80,                   // Restore Defaults
// // // // //         READ_DUTY_ACCELS = 81,                   // Read Default Duty Cycle Accelerations
// // // // //         READ_TEMP = 82,                          // Read Temperature
// // // // //         READ_TEMP_2 = 83,                        // Read Temperature 2

// // // // //         READ_STATUS = 90,         // Read Status
// // // // //         READ_ENCODER_MODES = 91,  // Read Encoder Modes
// // // // //         SET_M1_ENCODER_MODE = 92, // Set Motor 1 Encoder Mode
// // // // //         SET_M2_ENCODER_MODE = 93, // Set Motor 2 Encoder Mode
// // // // //         WRITE_SET_EEPROM = 94,    // Write Settings to EEPROM
// // // // //         READ_SET_EEPROM = 95,     // Read Settings from EEPROM

// // // // //         SET_STD_CONFIG_SET = 98,      // Set Standard Config Settings
// // // // //         READ_STD_CONFIG_SET = 99,     // Read Standard Config Settings
// // // // //         SET_CTRL_MODES = 100,         // Set CTRL Modes
// // // // //         READ_CTRL_MODES = 101,        // Read CTRL Modes
// // // // //         SET_CTRL1 = 102,              // Set CTRL1
// // // // //         SET_CTRL2 = 103,              // Set CTRL2
// // // // //         READ_CTRLS = 104,             // Read CTRLs
// // // // //         SET_M1_AUTO_HOME = 105,       // Set Auto Home Duty/Speed and Timeout M1
// // // // //         SET_M2_AUTO_HOME = 106,       // Set Auto Home Duty/Speed and Timeout M2
// // // // //         READ_AUTO_HOME_SET = 107,     // Read Auto Home Settings
// // // // //         READ_M1_M2_AVG_SPD = 108,     // Read Motor Average Speeds
// // // // //         SET_SPEED_ERR_LIMITS = 109,   // Set Speed Error Limits
// // // // //         READ_SPEED_ERR_LIMITS = 110,  // Read Speed Error Limits
// // // // //         READ_SPD_ERR = 111,           // Read Speed Errors
// // // // //         SET_POS_ERR_LIMITS = 112,     // Set Position Error Limits
// // // // //         READ_POS_ERR_LIMITS = 113,    // Read Position Error Limits
// // // // //         READ_POS_ERR = 114,           // Read Position Errors
// // // // //         SET_BATT_VOLT_OFFSETS = 115,  // Set Battery Voltage Offsets
// // // // //         READ_BATT_VOLT_OFFSETS = 116, // Read Battery Voltage Offsets
// // // // //         SET_CUR_BLANKING_PCTS = 117,  // Set Current Blanking Percentages
// // // // //         READ_CUR_BLANKING_PCTS = 118, // Read Current Blanking Percentages
// // // // //         DRIVE_M1_POS = 119,           // Drive M1 with Position
// // // // //         DRIVE_M2_POS = 120,           // Drive M2 with Position
// // // // //         DRIVE_M1_M2_POS = 121,        // Drive M1/M2 with Position
// // // // //         DRIVE_M1_SPD_POS = 122,       // Drive M1 with Speed and Position
// // // // //         DRIVE_M2_SPD_POS = 123,       // Drive M2 with Speed and Position
// // // // //         DRIVE_M1_M2_SPD_POS = 124,    // Drive M1/M2 with Speed and Position

// // // // //         SET_M1_MAX_CURRENT = 133,  // Set M1 Maximum Current
// // // // //         SET_M2_MAX_CURRENT = 134,  // Set M2 Maximum Current
// // // // //         READ_M1_MAX_CURRENT = 135, // Read M1 Maximum Current
// // // // //         READ_M2_MAX_CURRENT = 136, // Read M2 Maximum Current

// // // // //         SET_PWM_MODE = 148,  // Set PWM Mode
// // // // //         READ_PWM_MODE = 149, // Read PWM Mode

// // // // //         READ_USER_EEPROM_LOC = 252,  // Read User EEPROM Memory Location
// // // // //         WRITE_USER_EEPROM_LOC = 253, // Write User EEPROM Memory Location

// // // // //         NONE = 255 // No command
// // // // //     };

// // // // //     class SerialDevice
// // // // //     {
// // // // //     public:
// // // // //         typedef std::shared_ptr<SerialDevice> SharedPtr;

// // // // //         SerialDevice() = default;

// // // // //         explicit SerialDevice(const std::string device) { connect(device); }
// // // // //         virtual ~SerialDevice() { disconnect(); }

// // // // //         virtual bool connect(const std::string &device)
// // // // //         {
// // // // //             // RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "Writing...");
// // // // //             RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "connect(%s)...", device.c_str());
// // // // //             fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
// // // // //             connected_ = fd_ != -1;

// // // // //             if (connected_)
// // // // //             {
// // // // //                 setSerialDeviceOptions();
// // // // //             }
// // // // //             else
// // // // //             {
// // // // //                 std::cerr << "Failed to open serial device: " << device << std::endl;
// // // // //                 perror("Error");
// // // // //             }
// // // // //             RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "connect(%s) == %d", device.c_str(), connected_);
// // // // //             return connected_;
// // // // //         }

// // // // //         virtual void disconnect()
// // // // //         {
// // // // //             if (connected_)
// // // // //             {
// // // // //                 close(fd_);
// // // // //                 connected_ = false;
// // // // //             }
// // // // //         }

// // // // //         bool connected() const { return connected_; }

// // // // //         virtual std::size_t write(const std::byte *buffer, std::size_t count)
// // // // //         {
// // // // //             RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "write(..,%d)", (int)count);
// // // // //             std::string str;
// // // // //             for (std::size_t i = 0; i < count; i++)
// // // // //             {
// // // // //                 char buf[8];
// // // // //                 sprintf(buf, "%02X ", (int)buffer[i]);
// // // // //                 str += buf;
// // // // //             }
// // // // //             RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "write( %s ,%d)", str.c_str(), (int)count);

// // // // //             ssize_t result = ::write(fd_, buffer, count);
// // // // //             if (result < 0)
// // // // //             {
// // // // //                 // Error writing to device
// // // // //                 throw std::range_error("Error writing to the device!");
// // // // //             }
// // // // //             return static_cast<std::size_t>(result) == count;
// // // // //         }

// // // // //         virtual std::size_t read(std::byte *buffer, std::size_t count)
// // // // //         {
// // // // //             RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "read(..,%d)", (int)count);

// // // // //             // fd_set set;
// // // // //             // struct timeval timeout;

// // // // //             // /* Initialize the file descriptor set. */
// // // // //             // FD_ZERO(&set);
// // // // //             // FD_SET(fd_, &set);

// // // // //             // /* Initialize the timeout data structure. */
// // // // //             // timeout.tv_sec = 1;
// // // // //             // timeout.tv_usec = 0; // was 10ms

// // // // //             // /* select returns 0 if timeout, 1 if input available, -1 if error. */
// // // // //             // int res = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
// // // // //             // if (res < 0)
// // // // //             // {
// // // // //             //     throw std::range_error("Error reading from the serial device!");
// // // // //             // }
// // // // //             // else if (res == 0)
// // // // //             // {
// // // // //             //     throw std::runtime_error("Read timeout!");
// // // // //             // }
// // // // //             // ssize_t result = ::read(fd_, buffer, count);
// // // // //             // if (result < 0)
// // // // //             // {
// // // // //             //     // Error reading from the device
// // // // //             //     throw std::range_error("Error reading from the serial device!");
// // // // //             // }

// // // // //             int bytes_available = 0;
// // // // //             ioctl(fd_, FIONREAD, &bytes_available);

// // // // //             ssize_t result = ::read(fd_, buffer, count);

// // // // //             std::string str;
// // // // //             for (int i = 0; i < (int)result; i++)
// // // // //             {
// // // // //                 char buf[8];
// // // // //                 sprintf(buf, "%02X ", (std::uint8_t)buffer[i]);
// // // // //                 str += buf;
// // // // //             }

// // // // //             RCLCPP_INFO(rclcpp::get_logger("RoboclawSerialDevice"), "read(..,%d) avail = %d  text = %s", (int)count, bytes_available, str.c_str());

// // // // //             return static_cast<std::size_t>(result);
// // // // //         }

// // // // //     protected:
// // // // //         bool connected_ = false;

// // // // //     private:
// // // // //         void setSerialDeviceOptions()
// // // // //         {
// // // // //             const int read_timeout_ms = 100;
// // // // //             struct termios options;
// // // // //             int rc = tcgetattr(fd_, &options);

// // // // //             options.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
// // // // //             options.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
// // // // //             options.c_cflag &= ~CSIZE;         // Clear all the size bits
// // // // //             options.c_cflag |= CS8;            // 8 bits per byte (most common)
// // // // //             options.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
// // // // //             options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

// // // // //             options.c_lflag &= ~ICANON; // Disable canonical mode
// // // // //             options.c_lflag &= ~ECHO;   // Disable echo
// // // // //             options.c_lflag &= ~ECHOE;  // Disable erasure
// // // // //             options.c_lflag &= ~ECHONL; // Disable new-line echo
// // // // //             options.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

// // // // //             options.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
// // // // //             options.c_iflag &= ~(ICRNL | INLCR | IGNBRK | BRKINT | PARMRK | ISTRIP | IGNCR); // Turn off translation of carriage return and newline

// // // // //             options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
// // // // //             options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

// // // // //             // Set in/out baud rate to be 9600
// // // // //             cfsetispeed(&options, B38400);
// // // // //             cfsetospeed(&options, B38400);

// // // // //             options.c_cc[VMIN] = 0;
// // // // //             options.c_cc[VTIME] = (cc_t)read_timeout_ms / 100;

// // // // //             tcflush(fd_, TCIFLUSH);
// // // // //             rc = tcsetattr(fd_, TCSANOW, &options);
// // // // //             if (rc!=0){
// // // // //                 perror("tcsetattr");
// // // // //             }

// // // // //             // Set the file descriptor to non-blocking mode
// // // // //             // wg fcntl(fd_, F_SETFL, O_NONBLOCK);
// // // // //         }

// // // // //         int fd_ = -1;
// // // // //     };

// // // // // } // namespace roboclaw_serial
// ==================================================================================================

// // // // // // // Calculates CRC16 of nBytes of data in byte array message
// // // // // // unsigned int crc16(unsigned char *packet, int nBytes)
// // // // // // {
// // // // // //     unsigned int crc = 0;
// // // // // //     for (int byte = 0; byte < nBytes; byte++)
// // // // // //     {
// // // // // //         crc = crc ^ ((unsigned int)packet[byte] << 8);
// // // // // //         for (unsigned char bit = 0; bit < 8; bit++)
// // // // // //         {
// // // // // //             if (crc & 0x8000)
// // // // // //             {
// // // // // //                 crc = (crc << 1) ^ 0x1021;
// // // // // //             }
// // // // // //             else
// // // // // //             {
// // // // // //                 crc = crc << 1;
// // // // // //             }
// // // // // //         }
// // // // // //     }
// // // // // //     return crc;
// // // // // // }

#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/crc.hpp>
#include <roboclaw_serial/device.hpp>

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("hello world vqw_package vqwNav_test_roboclaw\n");

    const std::string device_name = "/dev/ttyAMA0";
    roboclaw_serial::SerialDevice serial_device(device_name);
    // serial_device.connect(device_name);

    if (!serial_device.connected())
    {
        printf("Failed to connect to serial device: %s\n", device_name.c_str());
        return 1;
    }

    std::uint8_t buffer[4] = {(std::uint8_t)0x80, (std::uint8_t)roboclaw_serial::Command::READ_FIRMWARE_VER};
    unsigned int crc = crc16((unsigned char *)buffer, 2);
    buffer[2] = crc >> 8;
    buffer[3] = crc & 0xFF;

    for (int x = 0; x < 30; x++)
    {

        serial_device.write((const std::byte *)buffer, 4);

        sleep(1);

        char response[64]{0};

        auto sz = serial_device.read((std::byte *)response, 60);

        printf("response size: %d\n", (int)sz);
        if (sz != 0)
            printf("response: \"%s\"\n", response);
        sleep(3);
    }

    // sleep(5);
    return 0;
}
