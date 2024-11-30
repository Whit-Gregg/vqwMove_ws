// File: Test_BNO086.cpp
#include "bno086_hardware_interface/Test_BNO086.hpp"

namespace bno086_hardware_interface
{
  //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
  int Test_BNO086::Initialize()
  {
    RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::Initialize() ############   Starting  #########");

    spiPort = std::make_shared<SPIClass>();

    int spi_rc = spiPort->Open(spi_dev_name.c_str(), spi_speed);

    if (spi_rc != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("test_bno086"), "Test_BNO086::Initialize()  spiPort->Open(\'%s\", %d) failed !!!!!", spi_dev_name.c_str(), spi_speed);
      spiPort->Close();
      return -1;
    }

    //===================================================================================================
    my_bno086 = std::make_shared<BNO08x>();

    // my_bno086->hardwareReset();

    bool ok = my_bno086->beginSPI(IMU_INT_pin, IMU_RESET_pin, IMU_WAKE_pin, spiPort);

    RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::Initialize() ############   bno086.beginSPI(int=%d, reset=%d, wake=%d)  %s  #########", IMU_INT_pin, IMU_RESET_pin, IMU_WAKE_pin, (ok) ? "OK" : "Failed !!!!!");
    if (ok)
    {
      uint32_t swPartNumber = my_bno086->prodIds.entry[0].swPartNumber;
      uint32_t swBuildNumber = my_bno086->prodIds.entry[0].swBuildNumber;
      uint32_t swVersionMajor = my_bno086->prodIds.entry[0].swVersionMajor;
      uint32_t swVersionMinor = my_bno086->prodIds.entry[0].swVersionMinor;
      uint32_t swVersionPatch = my_bno086->prodIds.entry[0].swVersionPatch;

      RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::Initialize()  swPartNumber=%d 0x%X     swBuildNumber=%d 0x%X ", swPartNumber, swPartNumber, swBuildNumber, swBuildNumber);
      RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::Initialize()  swVersionMajor=%d 0x%X    swVersionMinor=%d 0x%X   swVersionPatch=%d 0x%X ", swVersionMajor, swVersionMajor, swVersionMinor, swVersionMinor, swVersionPatch, swVersionPatch);
      // rclcpp::sleep_for(std::chrono::milliseconds(5000));
      // return -1;
    }

    sensor_rate_percent = 20;
    // set_reports();
    return 0;
  }

  //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
  int Test_BNO086::set_reports()
  {
    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "Test_BNO086::set_reports() ++++++++++++++++");
    bool ok = false;
#define LinearAccelerometer_MAX_Hz 400
#define Accelerometer_MAX_Hz 500
#define Gyro_MAX_Hz 400
#define Magnetometer_MAX_Hz 100
#define RotationVector_MAX_Hz 400
#define Hz_to_uS(Hz) (((1000000 / Hz) * 100) / sensor_rate_percent)

    uint16_t report_interval_LinearAccelerometer_us = Hz_to_uS(LinearAccelerometer_MAX_Hz);
    uint16_t report_interval_Accelerometer_us = Hz_to_uS(Accelerometer_MAX_Hz);
    uint16_t report_interval_Gyro_us = Hz_to_uS(Gyro_MAX_Hz);
    uint16_t report_interval_Magnetometer_us = Hz_to_uS(Magnetometer_MAX_Hz);
    uint16_t report_interval_RotationVector_us = Hz_to_uS(RotationVector_MAX_Hz);

    //....................................................................................................
    ok = my_bno086->enableLinearAccelerometer(report_interval_LinearAccelerometer_us); // LinearAccelerometer max rate: 400 Hz
    if (!ok)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
      ok = my_bno086->enableLinearAccelerometer(report_interval_LinearAccelerometer_us);
    }
    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "Test_BNO086::set_reports()  my_bno086->enableLinearAccelerometer(%d)  %s\n", report_interval_LinearAccelerometer_us, (ok) ? "ok" : "FAIL !!!!");
    if (!ok)
      return -9;

    //....................................................................................................
    ok = my_bno086->enableAccelerometer(report_interval_Accelerometer_us); // Accelerometer max rate: 500 Hz
    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "Test_BNO086::set_reports()  my_bno086->enableAccelerometer(%d)  %s\n", report_interval_Accelerometer_us, (ok) ? "ok" : "FAIL !!!!");
    if (!ok)
      return -9;

    //....................................................................................................
    ok = my_bno086->enableGyro(report_interval_Gyro_us); // Gyro max rate: 400 Hz
    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "Test_BNO086::set_reports()  my_bno086->enableGyro(%d)  %s\n", report_interval_Gyro_us, (ok) ? "ok" : "FAIL !!!!");
    if (!ok)
      return -9;

    //....................................................................................................
    ok = my_bno086->enableMagnetometer(report_interval_Magnetometer_us); // Magnetometer max rate: 100 Hz
    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "Test_BNO086::set_reports()  my_bno086->enableMagnetometer(%d)  %s\n", report_interval_Magnetometer_us, (ok) ? "ok" : "FAIL !!!!");
    if (!ok)
      return -9;

    //....................................................................................................
    ok = my_bno086->enableRotationVector(report_interval_RotationVector_us); // RotationVector max rate: 400 Hz
    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "Test_BNO086::set_reports()  my_bno086->enableRotationVector(%d)  %s\n", report_interval_RotationVector_us, (ok) ? "ok" : "FAIL !!!!");
    if (!ok)
      return -9;
    return 0;
  }

  //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
  void Test_BNO086::test()
  {
    // my_bno086->set_SystemOrientation({0, 0, -1, -1});
    my_bno086->serviceBus();

    uint32_t xyzw[4];
    my_bno086->get_SystemOrientation(xyzw);
    RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::test()  get_SystemOrientation()  0x%X  0x%X  0x%X  0x%X ", xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
    my_bno086->serviceBus();

    my_bno086->clear_SystemOrientation();
    my_bno086->serviceBus();
    my_bno086->get_SystemOrientation(xyzw);
    my_bno086->serviceBus();
    RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::test()  get_SystemOrientation()  0x%X  0x%X  0x%X  0x%X ", xyzw[0], xyzw[1], xyzw[2], xyzw[3]);

    // rclcpp::sleep_for(std::chrono::seconds(5));
    int reports_OK = set_reports();

    if (reports_OK != 0)
    { // this should be done on a seperate thread
      elapsedMillis elap;
      while (elap < 1000 * 10)
      {
        my_bno086->serviceBus();
        // rclcpp::spin_some();
      }
    }
  }

  void Test_BNO086::Delay(int delay_ms)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));
  }

#define GPIO_MOSI 10
#define GPIO_MISO 9
#define GPIO_SCK 11
#define GPIO_CS 8
#define GPIO_INT 23
#define GPIO_RESET 22
#define GPIO_WAKE 17
#define GPIO_BOOT 16

  int Test_BNO086::twittle_bit(int gpio_number, uint32_t timeout_ms)
  {
    gpio gpio_A(gpio_number, pin_direction::OUTPUT);
    gpio_A.Initialize();
    bool is_High = true;
    elapsedMillis elap;
    while (elap < timeout_ms)
    {
      gpio_A.set_pin_value(is_High ? pin_value::HIGH : pin_value::LOW);
      is_High = !is_High;
      Delay(50);
    }
    gpio_A.Close();
    return 0;
  }

  void Test_BNO086::test_0()
  {
    gpio gpio_wake(GPIO_WAKE, pin_direction::OUTPUT);
    gpio_wake.Initialize();
    gpio gpio_reset(GPIO_RESET, pin_direction::OUTPUT);
    gpio_reset.Initialize();

    gpio_wake.set_pin_value(pin_value::HIGH);
    Delay(50);
    gpio_reset.set_pin_value(pin_value::LOW);
    Delay(50);
    gpio_reset.set_pin_value(pin_value::HIGH);
    Delay(50);
    gpio_wake.set_pin_value(pin_value::LOW);
  }

  void Test_BNO086::test_1()
  {
    gpio gpio_reset(GPIO_RESET, pin_direction::OUTPUT);
    gpio_reset.Initialize();
    gpio gpio_wake(GPIO_WAKE, pin_direction::OUTPUT);
    gpio_wake.Initialize();
    gpio gpio_int(GPIO_INT, pin_direction::INPUT);
    gpio_int.Initialize();

    gpio_reset.set_pin_value(pin_value::HIGH);
    gpio_wake.set_pin_value(pin_value::HIGH);

    gpio_wake.set_pin_value(pin_value::HIGH);
    gpio_reset.set_pin_value(pin_value::LOW);
    Delay(100);
    gpio_reset.set_pin_value(pin_value::HIGH);
    Delay(100);
    if (gpio_int.wait_until_LOW(1000))
    {
      RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1() int is HIGH after reset");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1()  NO INT after reset");
    }
    gpio_wake.set_pin_value(pin_value::LOW);

    spiPort = std::make_shared<SPIClass>();
    int spi__speed = 1000000;

    //---------------------------------------------------------------------------------------------------
    int spi_rc = spiPort->Open(spi_dev_name.c_str(), spi__speed);
    if (spi_rc != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1()  spiPort->Open(\'%s\", %d) failed !!!!! rc=%d", spi_dev_name.c_str(), spi__speed, spi_rc);
      spiPort->Close();
      return;
    }

    const char *buf_out = "ABCEFGHIJKLMNOPQRSTUVWXYZ";
    uint8_t buf_in[512];
    int write_size = strlen(buf_out);
    int read_size = 4;
    int count_count = 0;

    elapsedMillis elap_int_wait;
    elapsedMillis elap;
    int int_wait = 0;
    int avg_int_wait_read = -1;
    int avg_int_wait_write = -1;
    int avg_int_wait_span = 5;
    int read_count = 0;
    int write_count = 0;

    while (elap < (1000 * 60))
    {
      count_count++;
      elap_int_wait = 0;
      if (((count_count %2))&&(gpio_int.wait_until_LOW(50)))
      {
        int_wait = elap_int_wait;
        if (avg_int_wait_read == -1)
          avg_int_wait_read = int_wait;
        avg_int_wait_read = (avg_int_wait_read * (avg_int_wait_span - 1) + int_wait) / avg_int_wait_span;
        memset(buf_in, 0, sizeof(buf_in));
        read_count++;
        buf_in[0] = 0xEE;
        buf_in[1] = 0xEE;
        buf_in[2] = 0xEE;
        buf_in[3] = 0xEE;
        
        int read_rc = spiPort->Read(buf_in, read_size);
        if (read_rc == read_size)
        {
          int read2_size = buf_in[0] + ((buf_in[1] << 8));
          RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1()  read2_size = %d   read1 data -->   %02X %02X %02X %02X ", read2_size, buf_in[0], buf_in[1], buf_in[2], buf_in[3]);
          if ((read2_size < (int)sizeof(buf_in)) && (read2_size > 0))
          {
            read_rc = spiPort->Read(buf_in, read2_size);
            if (read_rc == read2_size)
            {
              RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1()  read2_size = %d   read2 data --->   %02X %02X %02X %02X ", read2_size, buf_in[0], buf_in[1], buf_in[2], buf_in[3]);
            }
          }
          else
          {
            RCLCPP_ERROR(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1()  read2_size = %d ", read2_size);
          }
        }
      }
      else
      {
        int_wait = elap_int_wait;
        if (avg_int_wait_write == -1)
          avg_int_wait_write = int_wait;
        avg_int_wait_write = (avg_int_wait_write * (avg_int_wait_span - 1) + int_wait) / avg_int_wait_span;
        write_count++;
        int write_rc = spiPort->Write((const uint8_t *)buf_out, write_size);
        if (write_rc)
        {
          RCLCPP_ERROR(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1()  spiPort->Write() failed !!!!!  rc = %d  errno=%d - %s", write_rc, errno, strerror(errno));
        }
      }
      Delay(1000);
    } // end of while loop
    spiPort->Close();
    RCLCPP_INFO(rclcpp::get_logger("test_bno086"), "Test_BNO086::test_1()  read_count=%d  write_count=%d  avg_int_wait_read = %d    avg_int_wait_write = %d", read_count, write_count, avg_int_wait_read, avg_int_wait_write);
  }

  void Test_BNO086::test_2()
  {
    gpio gpio_reset(GPIO_RESET, pin_direction::OUTPUT);
    gpio_reset.Initialize();
    gpio gpio_wake(GPIO_WAKE, pin_direction::OUTPUT);
    gpio_wake.Initialize();
    gpio gpio_boot(GPIO_BOOT, pin_direction::OUTPUT);
    gpio_boot.Initialize();
    //    gpio gpio_cs(GPIO_CS, pin_direction::OUTPUT);
    //    gpio_cs.Initialize();
    gpio gpio_int(GPIO_INT, pin_direction::OUTPUT);
    gpio_int.Initialize();
    gpio gpio_sck(GPIO_SCK, pin_direction::OUTPUT);
    gpio_sck.Initialize();
    gpio gpio_mosi(GPIO_MOSI, pin_direction::OUTPUT);
    gpio_mosi.Initialize();
    gpio gpio_miso(GPIO_MISO, pin_direction::OUTPUT);
    gpio_miso.Initialize();

    uint32_t flip_ms = 100;
    elapsedMillis elap;
    while (elap < (1000 * 60))
    {
      gpio_reset.set_pin_value(pin_value::HIGH);
      gpio_wake.set_pin_value(pin_value::HIGH);
      gpio_boot.set_pin_value(pin_value::HIGH);
      gpio_int.set_pin_value(pin_value::HIGH);

      // gpio_cs.set_pin_value(pin_value::HIGH);
      gpio_sck.set_pin_value(pin_value::HIGH);
      gpio_mosi.set_pin_value(pin_value::HIGH);
      gpio_miso.set_pin_value(pin_value::HIGH);

      Delay(flip_ms);

      gpio_reset.set_pin_value(pin_value::LOW);
      gpio_wake.set_pin_value(pin_value::LOW);
      gpio_boot.set_pin_value(pin_value::LOW);
      gpio_int.set_pin_value(pin_value::LOW);

      // gpio_cs.set_pin_value(pin_value::LOW);
      gpio_sck.set_pin_value(pin_value::LOW);
      gpio_mosi.set_pin_value(pin_value::LOW);
      gpio_miso.set_pin_value(pin_value::LOW);

      Delay(flip_ms);
    }
  }

  void Test_BNO086::test_3()
  {
  }

} // end of namespace bno086_hardware_interface

//-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto tst = std::make_shared<bno086_hardware_interface::Test_BNO086>();

  tst->test_1();
  return 0;

  // tst->twittle_bit(GPIO_BOOT, 1000 * 60);
  // tst->test_0();
  // return 0;

  int rc_init = tst->Initialize();
  if (rc_init != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("test_bno086"), "main()  tst->Initialize() failed !!!!!");
    return -1;
  }
  tst->test();
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}