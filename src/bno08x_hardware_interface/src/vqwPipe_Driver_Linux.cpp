#include "vqwPipe_Driver_Linux.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
// #include "hardware_interface/system_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace vqw
{

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  // Returns the number of bytes which have been received and
  // can be fetched with readBytes().
  int vqwPipe_Driver_Linux::available(void)
  {
    int bytes_available = 0;
    int rc = ioctl(serial_port_fd, FIONREAD, &bytes_available);
    if (rc == -1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "available()  ioctl(..) !!Error!! errno=%d  %s  fd=%d", errno, strerror(errno), serial_port_fd);
      if (errno == EBADF)
      {
        if (restart())
        {
          rc = ioctl(serial_port_fd, FIONREAD, &bytes_available);
          if (rc == -1)
          {
            RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "available()  ioctl(..) !!Error!! again errno=%d  %s   fd=%d", errno, strerror(errno), serial_port_fd);
            return 0;
          }
        }
        else
          return 0;
      }
    }
    if (bytes_available)
      count_of_available_true++;
    else
      count_of_available_false++;

    bool show_info = false;
    if ((count_of_available_true + count_of_available_false) < 10)
    {
      show_info = true;
    }
    else if (((count_of_available_true + count_of_available_false) % count_of_available_MOD) == 0)
    {
      show_info = true;
      count_of_available_MOD = count_of_available_MOD * 2;
    }

    if (show_info)
    {
      RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "available() == %d    true_count=%ld  false_count=%ld  ", bytes_available, count_of_available_true, count_of_available_false);
    }

    return bytes_available;
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  // Returns the number of bytes which may be transmitted by writeBytes() without waiting.
  int vqwPipe_Driver_Linux::availableForWrite(void)
  {
    int bytes_available = 0;
    int bytes_in_use = 0;
    int rc = ioctl(serial_port_fd, TIOCOUTQ, &bytes_in_use);
    if (rc == -1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "availableForWrite()  ioctl(..) !!Error!! errno=%d  %s   fd=%d", errno, strerror(errno), serial_port_fd);
      return 0;
    }

    bytes_available = linux_serial_port_output_buffer_size - bytes_in_use;

    if (bytes_available)
      count_of_available_for_write_true++;
    else
      count_of_available_for_write_false++;

    bool show_info = false;
    if (bytes_in_use > 1024)
      show_info = true;
    // if ((count_of_available_for_write_true + count_of_available_for_write_false) < 10)
    // {
    //   show_info = true;
    // }
    // else if (((count_of_available_for_write_true + count_of_available_for_write_false) % count_of_available_for_write_MOD) == 0)
    // {
    //   show_info = true;
    //   count_of_available_for_write_MOD = count_of_available_for_write_MOD * 2;
    // }

    if (show_info)
    {
      RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "availableForWrite() == %d    true_count=%ld  false_count=%ld", bytes_available, count_of_available_for_write_true, count_of_available_for_write_false);
      //      RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "availableForWrite() == %d", bytes_available);
    }

    return bytes_available;
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  // // // void vqwPipe_Driver_Linux::read_data_into_incomming_data()
  // // // {
  // // //     if (serial_port_fd == -1)
  // // //       return;
  // // //     uint8_t buffer[1024];
  // // //     int rc = read(serial_port_fd, buffer, 1020);
  // // //     if (rc > 0)
  // // //     {
  // // //       count_of_bytes_read += rc;
  // // //       if ((count_of_bytes_read % (1024 * 1024)) == 0)
  // // //       {
  // // //         long Mbytes_read = count_of_bytes_read / (1024 * 1024);
  // // //         RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "readBytes()  total = %ld Megabytes", Mbytes_read);
  // // //       }
  // // //       for (int i = 0; i < rc; i++)
  // // //       {
  // // //         incomming_data.push_back(buffer[i]);
  // // //       }
  // // //       return;
  // // //     }
  // // //     if (rc == -1)
  // // //     {
  // // //       count_of_read_errors++;
  // // //       int err_no = errno;
  // // //       if ((count_of_read_errors < 5) || (count_of_read_errors % count_of_read_errors_MOD) == 0)
  // // //       {
  // // //         long Kbytes_read = count_of_bytes_read / 1024;
  // // //         RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "readBytes() !!Error!! errno=%d  %s   fd=%d  errCount = %ld   total data read %ld Kbytes",
  // // //                      err_no, strerror(err_no), serial_port_fd, count_of_read_errors, Kbytes_read);
  // // //         if (count_of_read_errors > 5)
  // // //         {
  // // //           count_of_read_errors_MOD = count_of_read_errors_MOD * 2;
  // // //         }
  // // //       }
  // // //       if (err_no == EBADF)
  // // //       {
  // // //         if (restart())
  // // //         {
  // // //           rc = read(serial_port_fd, buffer, length);
  // // //         }
  // // //         else
  // // //         {
  // // //           rc = -1;
  // // //         }
  // // //       }
  // // //     }
  // // //     return rc;

  // // // }
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  bool vqwPipe_Driver_Linux::readByte(uint8_t &pByte)
  {
    if (InboundDataBufferCount == 0)
    {
      InboundDataBufferIndex = 0;
      if (serial_port_fd == -1)
      {
        if (serial_port_name.empty() || serial_port_speed == 0)
          return false;
      }
      int count = -1;
      count = read(serial_port_fd, InboundDataBuffer, InboundDataBufferSize);
      if (count == -1)
      {
        int err_no = errno;

        if (err_no == EAGAIN)
        {
          return false;
        }
        count_of_read_errors++;
        if ((count_of_read_errors > 1000) && (count_of_read_errors % 7) == 0)
        {
          count_of_read_errors_MOD++;
        }
        {
          RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "readByte() !!Error!! errno=%d  %s", errno, strerror(errno));
        }
        if ((count_of_read_errors < 15) || (count_of_read_errors % count_of_read_errors_MOD) == 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "readByte() !!Error!! errno=%d  %s", err_no, strerror(err_no));
        }
        if (err_no == EINTR)
        {
          disconnect();
          return false;
        }
        if (err_no == EBADF)
        {
          if (restart())
          {
            count = read(serial_port_fd, InboundDataBuffer, InboundDataBufferSize);
            if (count == -1)
            {
              return false;
            }
          }
        } // end of: if (err_no == EBADF)
        return false;
      } /// end of: if (count == -1)
      if (count < 1)
      {
        return false;
      }
      // RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "readByte() got %d bytes", count);

      InboundDataBufferCount = count;
    } // end of: if (InboundDataBufferCount == 0)

    uint8_t the_byte = InboundDataBuffer[InboundDataBufferIndex++];
    if (InboundDataBufferIndex >= InboundDataBufferCount)
    {
      InboundDataBufferCount = 0;
      InboundDataBufferIndex = 0;
    }
    pByte = the_byte;
    return true;
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  size_t vqwPipe_Driver_Linux::readBytes(uint8_t *buffer, size_t length)
  {
    if (serial_port_fd == -1)
      return 0;
    int rc = read(serial_port_fd, buffer, length);
    if (rc > 0)
    {
      count_of_bytes_read += rc;
      if ((count_of_bytes_read % (1024 * 1024)) == 0)
      {
        long Mbytes_read = count_of_bytes_read / (1024 * 1024);
        RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "readBytes()  total = %ld Megabytes", Mbytes_read);
      }
      return rc;
    }
    if (rc == -1)
    {
      count_of_read_errors++;
      int err_no = errno;
      if ((count_of_read_errors < 5) || (count_of_read_errors % count_of_read_errors_MOD) == 0)
      {
        long Kbytes_read = count_of_bytes_read / 1024;
        RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "readBytes() !!Error!! errno=%d  %s   fd=%d  errCount = %ld   total data read %ld Kbytes",
                     err_no, strerror(err_no), serial_port_fd, count_of_read_errors, Kbytes_read);
        if (count_of_read_errors > 5)
        {
          count_of_read_errors_MOD = count_of_read_errors_MOD * 2;
        }
      }
      if (err_no == EBADF)
      {
        if (restart())
        {
          rc = read(serial_port_fd, buffer, length);
        }
        else
        {
          rc = -1;
        }
      }
    }
    return rc;
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  size_t vqwPipe_Driver_Linux::writeBytes(uint8_t *buffer, size_t length)
  {
    if (serial_port_fd == -1)
      return 0;
    int rc = write(serial_port_fd, buffer, length);
    if (rc == (int)length)
      return rc;
    if (rc == -1)
    {
      int err_no = errno;
      RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "writeBytes() !!Error!! errno=%d  %s", err_no, strerror(err_no));
      if (err_no == EBADF)
      {
        if (restart())
        {
          rc = write(serial_port_fd, buffer, length);
          if (rc == (int)length)
            return rc;
        }
      }
    }
    return rc;
  }
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  void vqwPipe_Driver_Linux::loop()
  {
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  bool vqwPipe_Driver_Linux::connect(const std::string &serial_port_name_, int speed_baud)
  {
    // RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "connect(%s)...", device.c_str());
    if (serial_port_fd != -1)
    {
      close(serial_port_fd);
    }
    serial_port_fd = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port_fd != -1)
    {
      serial_port_name = serial_port_name_;
      serial_port_speed = speed_baud;
      setSerialDeviceOptions();
      RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "connect(%s, %d) == %s   fd=%d", serial_port_name_.c_str(), speed_baud, (connected()) ? "true" : "false", serial_port_fd);
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "connect(%s) !! FAILED !!   errno = %d  %s", serial_port_name_.c_str(), errno, strerror(errno));
    }
    return serial_port_fd != -1;
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  void vqwPipe_Driver_Linux::disconnect()
  {
    if (serial_port_fd != -1)
    {
      close(serial_port_fd);
      serial_port_fd = -1;
    }
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  bool vqwPipe_Driver_Linux::restart()
  {
    RCLCPP_INFO(rclcpp::get_logger("vqwPipe_Driver_Linux"), "ReStarting...........\"%s\"", serial_port_name.c_str());
    disconnect();
    if (serial_port_name.empty())
    {
      return false;
    }
    return connect(serial_port_name, serial_port_speed);
  }

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  void vqwPipe_Driver_Linux::setSerialDeviceOptions()
  {
    struct termios options;
    int rc = tcgetattr(serial_port_fd, &options);
    if (rc != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "setSerialDeviceOptions() !! FAILED !!   tcgetattr  rc=%d  errno=%d  %s", rc, errno, strerror(errno));
    }

    cfmakeraw(&options);
    cfsetspeed(&options, serial_port_speed);

    rc = tcsetattr(serial_port_fd, TCSANOW, &options);
    if (rc != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "setSerialDeviceOptions() !! FAILED !!   tcsetattr  rc=%d  errno=%d  %s", rc, errno, strerror(errno));
    }

    return;

    // // // // // const int read_timeout_ms = 100;
    // // // // // struct termios options;
    // // // // // int rc = tcgetattr(serial_port_fd, &options);
    // // // // // if (rc != 0)
    // // // // // {
    // // // // //   RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "setSerialDeviceOptions() !! FAILED !!   tcgetattr  rc=%d  errno=%d  %s", rc, errno, strerror(errno));
    // // // // // }

    // // // // // options.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    // // // // // options.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    // // // // // options.c_cflag &= ~CSIZE;         // Clear all the size bits
    // // // // // options.c_cflag |= CS8;            // 8 bits per byte (most common)
    // // // // // options.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    // // // // // options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // // // // // options.c_lflag &= ~ICANON; // Disable canonical mode
    // // // // // options.c_lflag &= ~ECHO;   // Disable echo
    // // // // // options.c_lflag &= ~ECHOE;  // Disable erasure
    // // // // // options.c_lflag &= ~ECHONL; // Disable new-line echo
    // // // // // options.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    // // // // // options.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    // // // // // options.c_iflag &= ~(ICRNL | INLCR | IGNBRK | BRKINT | PARMRK | ISTRIP | IGNCR); // Turn off translation of carriage return and newline

    // // // // // options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    // // // // // options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // // // // // // Set in/out baud rate to be 9600, 38400, 115200
    // // // // // cfsetispeed(&options, serial_port_speed);
    // // // // // cfsetospeed(&options, serial_port_speed);

    // // // // // options.c_cc[VMIN] = 0;
    // // // // // options.c_cc[VTIME] = (cc_t)read_timeout_ms / 100;

    // // // // // tcflush(serial_port_fd, TCIFLUSH);
    // // // // // rc = tcsetattr(serial_port_fd, TCSANOW, &options);
    // // // // // if (rc != 0)
    // // // // // {
    // // // // //   RCLCPP_ERROR(rclcpp::get_logger("vqwPipe_Driver_Linux"), "setSerialDeviceOptions() !! FAILED !!   tcsetattr  rc=%d  errno=%d  %s", rc, errno, strerror(errno));
    // // // // // }
  } // end of: vqwPipe_Driver_Linux::setSerialDeviceOptions

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
} // end of: namespace vqw
