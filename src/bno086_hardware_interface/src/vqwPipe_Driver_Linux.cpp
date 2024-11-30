#include "vqwPipe_Driver_Linux.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "hardware_interface/system_interface.hpp"


namespace vqw
{


  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        // Returns the number of bytes which have been received and
        // can be fetched with readBytes().
        int vqwPipe_Driver_Linux::available(void)
        {
            int bytes_available = 0;
            ioctl(fd_, FIONREAD, &bytes_available);
            return bytes_available;
        }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        // Returns the number of bytes which may be transmitted by writeBytes() without waiting.
        int vqwPipe_Driver_Linux::availableForWrite(void)
        {
            int bytes_available = 0;
            ioctl(fd_, TIOCOUTQ, &bytes_available);
            return bytes_available;
        }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        size_t vqwPipe_Driver_Linux::readBytes(uint8_t *buffer, size_t length)
        {
            return read(fd_, buffer, length);
        }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        size_t vqwPipe_Driver_Linux::writeBytes(uint8_t *buffer, size_t length)
        {
            return write(fd_, buffer, length);
        }


  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        void vqwPipe_Driver_Linux::loop()
        {
        }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  bool vqwPipe_Driver_Linux::connect(const std::string &serial_port_name_ , int speed_baud)
  {
    // RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "connect(%s)...", device.c_str());
    if (serial_port_fd != -1)
    {
      close(serial_port_fd);
    }
    serial_port_fd = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY);
    connected_ = serial_port_fd != -1;

    if (connected_)
    {
      serial_port_name = serial_port_name_;
      serial_port_speed = speed_baud;
      setSerialDeviceOptions();
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "connect(%s) !! FAILED !!", serial_port_name_.c_str());
      //std::cerr << "Failed to open serial device: " << device << std::endl;
      //perror("Error");
    }
    RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "connect(%s) == %s", serial_port_name_.c_str(), (connected_)? "true" : "false"); 
    return connected_;
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  void vqwPipe_Driver_Linux::disconnect()
  {
    if (connected_)
    {
      close(serial_port_fd);
      connected_ = false;
      serial_port_fd = -1;
    }
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  bool vqwPipe_Driver_Linux::restart()
  {
    // RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "ReStarting...........%s", device_name_.c_str());
    disconnect();
    return connect(serial_port_name,serial_port_speed);
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  void vqwPipe_Driver_Linux::setSerialDeviceOptions()
  {
    const int read_timeout_ms = 100;
    struct termios options;
    int rc = tcgetattr(serial_port_fd, &options);

    options.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    options.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    options.c_cflag &= ~CSIZE;         // Clear all the size bits
    options.c_cflag |= CS8;            // 8 bits per byte (most common)
    options.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    options.c_lflag &= ~ICANON; // Disable canonical mode
    options.c_lflag &= ~ECHO;   // Disable echo
    options.c_lflag &= ~ECHOE;  // Disable erasure
    options.c_lflag &= ~ECHONL; // Disable new-line echo
    options.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    options.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    options.c_iflag &= ~(ICRNL | INLCR | IGNBRK | BRKINT | PARMRK | ISTRIP | IGNCR); // Turn off translation of carriage return and newline

    options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set in/out baud rate to be 9600, 38400, 115200
    cfsetispeed(&options, serial_port_speed);
    cfsetospeed(&options, serial_port_speed);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = (cc_t)read_timeout_ms / 100;

    tcflush(serial_port_fd, TCIFLUSH);
    rc = tcsetattr(serial_port_fd, TCSANOW, &options);
    if (rc != 0)
    {
      perror("tcsetattr");
    }
  } // end of: vqwPipe_Driver_Linux::setSerialDeviceOptions

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
} // end of: namespace vqw
