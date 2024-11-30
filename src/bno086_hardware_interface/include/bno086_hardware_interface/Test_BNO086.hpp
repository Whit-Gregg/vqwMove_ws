#pragma once
#include <limits>
#include <vector>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"

#include "bno086_hardware_interface/BNO08x.hpp"
#include "bno086_hardware_interface/SPI.hpp"

namespace bno086_hardware_interface
{

  class Test_BNO086
  {
  public:
    int Initialize();
    void test();
    int set_reports();
    void test_0();
    void test_1();
    void test_2();
    void test_3();

    int twittle_bit(int gpio_number, uint32_t timeout_ms);
    void Delay(int delat_ms);

  private:
    std::shared_ptr<SPIClass> spiPort;
    std::shared_ptr<BNO08x> my_bno086;

    int IMU_CS_pin = 8;
    int IMU_INT_pin = 23;
    int IMU_RESET_pin = 22;
    int IMU_WAKE_pin = 17;
    int IMU_BOOT_pin = 16;

    uint16_t IMU_Report_Interval = 14;
    std::string spi_dev_name = "/dev/spidev0.0";
    int32_t spi_speed = 500000; // 300000;
    int sensor_rate_percent = 100;

  }; // end of class Test_BNO086

} // end of namespace bno086_hardware_interface