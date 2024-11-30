
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
// #include <string.h>
#include <signal.h>
#include <fcntl.h>
// #include <time.h>
#include <sys/ioctl.h>
// #include <linux/spi/spidev.h>

#include "SPI.hpp"

namespace bno086_hardware_interface
{

  int SPIClass::Open(const char *dev, uint32_t spi_speed_)
  {
    int rc=0;
    int32_t spi__Speed = spi_speed_;
    uint8_t spi__Mode = SPI_MODE_3;
    uint8_t spi__BPW = 8;

    if ((spi_fd = open(dev, O_RDWR)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Open(..) error opening %s  errno = %d - %s", dev, errno, strerror(errno));
      return -1;
    }
    if ((rc=ioctl(spi_fd, SPI_IOC_WR_MODE, &spi__Mode)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Open(..) can't set spi mode,  rc=%d", rc);
      return -1;
    }
    if ((rc=ioctl(spi_fd, SPI_IOC_RD_MODE, &spi__Mode)) < 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Open(..) can't get spi mode,  rc=%d", rc);
      //return -1;
    }
    if ((rc=ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi__BPW)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Open(..) can't set bits per word,  rc=%d", rc);
      return -1;
    }
    if ((rc=ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spi__BPW)) < 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Open(..) can't get bits per word,  rc=%d", rc);
      //return -1;
    }

    if ((rc=ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi__Speed)) < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Open(..) can't set max speed hz, %d  rc=%d", spi__Speed, rc);
      return -1;
    }
    if ((rc=ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi__Speed)) < 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Open(..) can't get max speed hz, %d  rc=%d", spi__Speed, rc);
      //return -1;
    }
    spiDeviceName = dev;
    return 0;
  }

  void SPIClass::Close()
  {
    if (spi_fd >= 0)
    {
      close(spi_fd);
      spi_fd = -1;
    }
  }

  int SPIClass::Read(std::uint8_t *pBuffer, int bufferSize) // returns # of bytes read, or < 0 for error
  {
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = 0;
    spi.rx_buf = (unsigned long)pBuffer;
    spi.len = bufferSize;
    spi.delay_usecs = spiDelay;
    spi.speed_hz = spiSpeed;
    spi.bits_per_word = spiBPW;
    errno=0;
    int rc = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
    if (rc != bufferSize)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Read(..,%d)  %s ioctl() failed  rc=%d  errno=%d - %s", bufferSize, spiDeviceName.c_str(), rc, errno, strerror(errno));
    }
    if (errno) return 0-errno;
    return bufferSize;
  }

  int SPIClass::Write(const std::uint8_t *pBuffer, int bufferSize) // returns < 0 for error, zero for OK
  {
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long)pBuffer;
    spi.rx_buf = 0;
    spi.len = bufferSize;
    spi.delay_usecs = spiDelay;
    spi.speed_hz = spiSpeed;
    spi.bits_per_word = spiBPW;
    errno=0;
    int rc = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
    if (rc != bufferSize)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "SPI::Write(..,%d)  %s  ioctl() failed  rc=%d  errno=%d - %s", bufferSize, spiDeviceName.c_str(), rc, errno, strerror(errno));
    }
    return 0-errno;
  }


} // end of namespace bno086_hardware_interface
