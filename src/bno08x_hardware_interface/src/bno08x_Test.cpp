#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <rclcpp/rclcpp.hpp>
#include "elapsedMillis.h"
#include "vqwPipe_Driver_Linux.hpp"
#include "vqwPipe_Channel.hpp"
#include "vqwPipe.hpp"
#include "OneLineBuffer.hpp"
#include "Channel_ROS_msg.hpp"

#include "/usr/include/libserial/SerialPort.h"
#include "/usr/include/libserial/SerialPortConstants.h"
// #include <libserial/SerialPort.h>
// #include <libserial/SerialPortConstants.h

bool shutdown_now = false;

void setSerialDeviceOptions(int serial_port_fd, int serial_port_speed, [[maybe_unused]]int read_timeout_ms = 100)
{
    struct termios options;
    int rc = tcgetattr(serial_port_fd, &options);
    if (rc != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Test_1"), "setSerialDeviceOptions() !! FAILED !!   tcgetattr  rc=%d  errno=%d  %s", rc, errno, strerror(errno));
    }

    cfmakeraw(&options);
    cfsetspeed(&options, serial_port_speed);

    rc = tcsetattr(serial_port_fd, TCSANOW, &options);
    if (rc != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Test_1"), "setSerialDeviceOptions() !! FAILED !!   tcsetattr  rc=%d  errno=%d  %s", rc, errno, strerror(errno));
    }

    return;

    // // // // // from https://manpages.ubuntu.com/manpages/trusty/man3/termios.3.html
    // // // // // IGNBRK Ignore BREAK
    // // // // // IGNPAR Ignore framing errors and parity errors
    // // // // // PARMRK If  IGNPAR is not set, prefix a character with a parity error or framing error with \377 \0.
    // // // // // IGNCR  Ignore carriage return on input

    // // // //     // cfmakeraw() sets the terminal to something like the "raw" mode of the old Version 7 terminal driver:

    // // // // options.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    // // // // options.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    // // // // options.c_cflag &= ~CSIZE;         // Clear all the size bits
    // // // // options.c_cflag |= CS8;            // 8 bits per byte (most common)
    // // // // options.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    // // // // options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // // // // options.c_lflag &= ~ICANON; // Disable canonical mode
    // // // // options.c_lflag &= ~ECHO;   // Disable echo
    // // // // options.c_lflag &= ~ECHOE;  // Disable erasure
    // // // // options.c_lflag &= ~ECHONL; // Disable new-line echo
    // // // // options.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    // // // // options.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    // // // // options.c_iflag &= ~(ICRNL | INLCR | IGNBRK | BRKINT | PARMRK | ISTRIP | IGNCR); // Turn off translation of carriage return and newline

    // // // // options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    // // // // options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // // // // // Set in/out baud rate to be 9600, 38400, 115200
    // // // // cfsetispeed(&options, serial_port_speed);
    // // // // cfsetospeed(&options, serial_port_speed);

    // // // // options.c_cc[VMIN] = 0;
    // // // // options.c_cc[VTIME] = (cc_t)read_timeout_ms / 100;

    // // // // tcflush(serial_port_fd, TCIFLUSH);
    // // // // rc = tcsetattr(serial_port_fd, TCSANOW, &options);
    // // // // if (rc != 0)
    // // // // {
    // // // //     RCLCPP_ERROR(rclcpp::get_logger("Test_1"), "setSerialDeviceOptions() !! FAILED !!   tcsetattr  rc=%d  errno=%d  %s", rc, errno, strerror(errno));
    // // // // }
} // end of: setSerialDeviceOptions

void Test_1(int speed = 115200)
{
    std::string port_name = "/dev/ttyAMA3";
    // int speed = 115200;
    int loop_count = 0;
    int loop_max = 1000;
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Test_1"), "open() !!Error!! errno=%d  %s", errno, strerror(errno));
        return;
    }
    setSerialDeviceOptions(fd, speed, 0);
    for (loop_count = 0; loop_count < loop_max; loop_count++)
    {
        uint8_t buf[1024];

        rclcpp::sleep_for(std::chrono::milliseconds(5));

        int sz = read(fd, buf, sizeof(buf) - 8);
        if (shutdown_now)
            return;
        if (sz == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Test_1"), "read() !!Error!! errno=%d  %s", errno, strerror(errno));
            continue;
            ;
        }
        if (sz == 0)
        {
            continue;
        }
        if (sz > 256)
        {
            sz = 256;
        }
        char outBuf[1024]{0};
        for (int i = 0; i < sz; i++)
        {
            sprintf(outBuf + strlen(outBuf), "%02X ", buf[i]);
        }
        RCLCPP_INFO(rclcpp::get_logger("Test_1"), "read()  %3d bytes: %s", sz, outBuf);
    }
}

void Test_2(int speed = 115200)
{
    vqw::vqwPipe_Driver_Linux dvr;
    std::string port_name = "/dev/ttyAMA3";
    // int speed = 115200;
    bool ok = dvr.connect(port_name, speed);
    if (!ok)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Test_2"), "connect() !!Error!!  errno=%d  %s", errno, strerror(errno));
        return;
    }

    for (int z = 0; z < 100; z++)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        uint8_t buf[256];
        int sz = dvr.readBytes(buf, sizeof(buf) - 8);
        if (shutdown_now)
            return;

        if (sz == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Test_2"), "readBytes() !!Error!! errno=%d  %s", errno, strerror(errno));
            continue;
        }

        if (sz == 0)
        {
            continue;
        }
        if (sz > 256)
        {
            sz = 256;
        }
        char outBuf[1024]{0};
        for (int i = 0; i < sz; i++)
        {
            sprintf(outBuf + strlen(outBuf), "%02X ", buf[i]);
        }
        RCLCPP_INFO(rclcpp::get_logger("Test_1"), "read()  %d bytes: %s", sz, outBuf);
    }
}

void on_vqwPipe_chan_IMU_data_msg(const vqwPipe_chan_IMU_data_msg *pData)
{
    const char *pType = "??";
    switch (pData->data_type_code)
    {
    case vqwPipe_chan_IMU_data_msg::data_type_code_Accel:
        pType = "Accel";
        break;
    case vqwPipe_chan_IMU_data_msg::data_type_code_Gyro:
        pType = "Gyro";
        break;
    case vqwPipe_chan_IMU_data_msg::data_type_code_Mag:
        pType = "Mag";
        break;
    case vqwPipe_chan_IMU_data_msg::data_type_code_LinAccel:
        pType = "LinAccel";
        break;
    case vqwPipe_chan_IMU_data_msg::data_type_code_Orientation:
        pType = "Orientation";
        break;
    default:
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("Test_3"), "IMU:  type=%s   X=%.3f, Y=%.3f, Z=%.3f", pType, pData->X, pData->Y, pData->Z);
}

void on_RougeData_line(const uint8_t *pData, int dataSize)
{
    char buf[(OneLineBuffer_SIZE + 2) * 4];
    if (dataSize >= OneLineBuffer_SIZE)
        dataSize = OneLineBuffer_SIZE - 1;
    buf[0] = 0;
    for (int i = 0; i < dataSize; i++)
    {
        sprintf(buf + strlen(buf), " %02X", pData[i]);
    }
    RCLCPP_INFO(rclcpp::get_logger("Test_3"), "Bno08xHardwareInterface::on_RougeData_line()  %s", buf);
}

void Test_3(int speed = 115200)
{
    vqw::vqwPipe_Driver_Linux pPipeDriver;
    vqw::vqwPipe pPipe(&pPipeDriver, "Test_3");
    vqw::vqwPipe_Channel pChannel((uint8_t)vqw::vqwPipe_Channel_Number_Assignments::Channel_ROS, &pPipe, "ROS");
    vqw::OneLineBuffer pipe_rouge_data_buffer;

    pChannel.addCallback<vqwPipe_chan_IMU_data_msg>(on_vqwPipe_chan_IMU_data_msg);

    pipe_rouge_data_buffer.set_Callback(on_RougeData_line);

    pPipe.set_RougeData_callback(
        [pb = &pipe_rouge_data_buffer](const uint8_t *pData, int dataSize)
        {
            pb->write(pData, dataSize);
        });

    std::string port_name = "/dev/ttyAMA3";
    // int speed = 115200;
    bool ok = pPipeDriver.connect(port_name, speed);
    if (!ok)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Test_3"), "connect() !!Error!!  errno=%d  %s", errno, strerror(errno));
        return;
    }
    for (int z = 0; z < 1000; z++)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        pPipe.loop();
        if (shutdown_now)
            return;
    }
}

void Test_4(int speed = 115200)
{
    std::string port_name = "/dev/ttyAMA3";
    // int speed = 115200;
    int loop_count = 0;
    int loop_max = 1000;
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Test_4"), "open() !!Error!! errno=%d  %s", errno, strerror(errno));
        return;
    }
    setSerialDeviceOptions(fd, speed, 0);
    for (loop_count = 0; loop_count < loop_max; loop_count++)
    {
        char buf[1024];

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        sprintf(buf, "ABCDEFGHIJKLMNOPQRSTUVWXYZ=%d\n", loop_count);

        int len = strlen(buf);
        int sz = write(fd, buf, len);

        if (tcdrain(fd) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Test_4"), "tcdrain() !!Error!! errno=%d  %s", errno, strerror(errno));
        }
        //flush(fd);

        if (shutdown_now)
            return;
        if (sz == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Test_4"), "write() !!Error!! errno=%d  %s", errno, strerror(errno));
            continue;
        }
        if (sz != len)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Test_4"), "write() length misMatch  sz=%d  len=%d", sz, len);
        }
    }
    close(fd);
}

void Test_5([[maybe_unused]]int speed = 115200)
{
    const std::string port_name = "/dev/ttyAMA3";
    int loop_count = 0;
    int loop_max = 1000;

    LibSerial::SerialPort serial_port;

    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port.SetVMin(0);
    serial_port.SetVTime(0);
    serial_port.Open(port_name.c_str());

    // // int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    // // if (fd == -1)
    // // {
    // //     RCLCPP_ERROR(rclcpp::get_logger("Test_4"), "open() !!Error!! errno=%d  %s", errno, strerror(errno));
    // //     return;
    // // }
    // // setSerialDeviceOptions(fd, speed, 0);
    for (loop_count = 0; loop_count < loop_max; loop_count++)
    {
        char buf[1024];

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        sprintf(buf, "ABCDEFGHIJKLMNOPQRSTUVWXYZ=%d\n", loop_count);

        // int len = strlen(buf);
        std::string buf_str = buf;

        try
        {
            serial_port.Write(buf_str);
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Test_5"), "std::exception  %s", e.what());
        }

        if (shutdown_now)
        {
            serial_port.Close();
            return;
        }
    }
    serial_port.Close();
}

void sigintHandler([[maybe_unused]] int sig)
{
    shutdown_now = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, sigintHandler);

    int speed = 115200;

    // rclcpp::spin(std::make_shared<Test_1>());
    if (argc > 1)
    {
        int z = atoi(argv[1]);
        if (z == 1)
        {
            Test_1(speed);
        }
        else if (z == 2)
        {
            Test_2(speed);
        }
        else if (z == 3)
        {
            Test_3(speed);
        }
        else if (z == 4)
        {
            Test_4(speed);
        }
        else if (z == 5)
        {
            Test_5(speed);
        }
    }

    rclcpp::shutdown();
    return 0;
}
