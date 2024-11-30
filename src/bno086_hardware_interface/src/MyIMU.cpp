// MyIMU.cpp

#include <cstdint>
#include "MyIMU.hpp"
#include "rclcpp/rclcpp.hpp"


namespace bno086_hardware_interface
{

    //~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=
    //MyIMU *pThis = nullptr;
    //~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=

    // bool MyIMU::begin(uint8_t irq_pin_)
    // {
    //     irq_pin = irq_pin_;
    //     pThis = this;   //ToDo:
        

    //           //=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~
    //           //=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~

    //         // pinMode(IMU_WAKE_pin, OUTPUT_OPENDRAIN);
    //         // digitalWrite(IMU_WAKE_pin, 1); // IMU_WAKE_pin == HIGH at start-up, selects SPI interface

    //         // pinMode(IMU_RESET_pin, OUTPUT);
    //         // digitalWrite(IMU_RESET_pin, HIGH);

    //     int spi_rc = spiPort.Open(SPIClass::spiDevice_default, SPIClass::spiSpeed_default);

    //     if (spi_rc != 0)
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::begin(%d)  spiPort.Open() failed !!!!!", irq_pin);
    //         return false;
    //     }

    //     my_bno086.hardwareReset();

    //     bool ok = my_bno086.beginSPI(IMU_CS_pin, IMU_INT_pin, IMU_RESET_pin, IMU_WAKE_pin, spiPort);
    //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::begin(%d) ############   bno086.beginSPI(%d,%d,%d,%d)  %s  #########\n", irq_pin_, IMU_CS_pin,IMU_INT_pin,IMU_RESET_pin, IMU_WAKE_pin, (ok) ? "OK" : "Failed !!!!!");
    //     if (!ok)
    //     {
    //         rclcpp::sleep_for(std::chrono::milliseconds(5000));
    //         return false;
    //     }

    //     uint32_t swPartNumber = my_bno086.prodIds.entry[0].swPartNumber;
    //     uint32_t swBuildNumber = my_bno086.prodIds.entry[0].swBuildNumber;
    //     uint32_t swVersionMajor = my_bno086.prodIds.entry[0].swVersionMajor;
    //     uint32_t swVersionMinor = my_bno086.prodIds.entry[0].swVersionMinor;
    //     uint32_t swVersionPatch = my_bno086.prodIds.entry[0].swVersionPatch;

    //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::begin(%d)  swPartNumber=%d 0x%X     swBuildNumber=%d 0x%X ", irq_pin, swPartNumber, swPartNumber, swBuildNumber, swBuildNumber);
    //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::begin(%d)  swVersionMajor=%d 0x%X    swVersionMinor=%d 0x%X   swVersionPatch=%d 0x%X ", irq_pin, swVersionMajor, swVersionMajor, swVersionMinor, swVersionMinor, swVersionPatch, swVersionPatch);

    //     //my_bno086.enableDebugging(DebugLog);
    //     BNO086_set_reports();

    //     return true;
    //     // #endif
    //     //=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~
    // }

    //=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~
    //=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~=-+~

    // void MyIMU::spin()
    // {
    //     loop_count++;

    //     BNO086_loop();

    //     // // // // if (elap_since_DumpIntCounts > elap_since_DumpIntCounts_span) {
    //     // // // //     elap_since_DumpIntCounts=0;
    //     // // // //     my_bno086.Dump_WaitForInt_counts();
    //     // // // // }
    // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // // void MyIMU::BNO086_set_reports()
    // // {
    // //     // BACK_TRACE;
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::BNO086_set_reports() ++++++++++++++++");
    // //     bool ok = false;

    // //     //....................................................................................................

    // //     //....................................................................................................
    // //     ok = my_bno086.enableLinearAccelerometer(IMU_REPORT_INTERVAL);
    // //     if (!ok)
    // //     {
    // //         Delay(10);
    // //         ok = my_bno086.enableLinearAccelerometer(IMU_REPORT_INTERVAL);
    // //     }
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "!!!!!!!  my_bno086.enableLinearAccelerometer(%d)  %s\n", IMU_REPORT_INTERVAL, (ok) ? "ok" : "FAIL !!!!");

    // //     //....................................................................................................
    // //     ok = my_bno086.enableAccelerometer(IMU_REPORT_INTERVAL);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "!!!!!!!  my_bno086.enableAccelerometer(%d)  %s\n", IMU_REPORT_INTERVAL, (ok) ? "ok" : "FAIL !!!!");

    // //     //....................................................................................................
    // //     ok = my_bno086.enableGyro(IMU_REPORT_INTERVAL);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "!!!!!!!  my_bno086.enableGyro(%d)  %s\n", IMU_REPORT_INTERVAL, (ok) ? "ok" : "FAIL !!!!");

    // //     //....................................................................................................
    // //     ok = my_bno086.enableMagnetometer(IMU_REPORT_INTERVAL);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "!!!!!!!  my_bno086.enableMagnetometer(%d)  %s\n", IMU_REPORT_INTERVAL, (ok) ? "ok" : "FAIL !!!!");

    // //     //....................................................................................................
    // //     uint16_t RotVec_Interval = (IMU_REPORT_INTERVAL * 5) / 2;
    // //     ok = my_bno086.enableRotationVector(RotVec_Interval);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "!!!!!!!  my_bno086.enableRotationVector(%d)  %s\n", RotVec_Interval, (ok) ? "ok" : "FAIL !!!!");
    // // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // const char *MyIMU::reset_reason_description(uint8_t reason)
    // {
    //     const char *p = "unknown";
    //     switch (reason)
    //     {
    //     case 1:
    //         p = "Power On";
    //         break;
    //     case 2:
    //         p = "Internal";
    //         break;
    //     case 3:
    //         p = "Watch Dog";
    //         break;
    //     case 4:
    //         p = "External";
    //         break;
    //     case 5:
    //         p = "Other";
    //         break;
    //     }
    //     return p;
    // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // // bool MyIMU::tareNow() { return my_bno086.tareNow(); }
    // // bool MyIMU::saveTare() { return my_bno086.saveTare(); }
    // // bool MyIMU::clearTare() { return my_bno086.clearTare(); }
    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    //  sqrt(2)/2 == 0.7071067811865475
    // Q30 of  that is 0x2D413CCD
    // Q30 of -that is 0xD2BEC333

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^

    // // // // // static const uint32_t Sqrt_2_div_2 = 0x2D413CCD;
    // // // // // static const uint32_t minus_Sqrt_2_div_2 = 0xD2BEC333;

    // // // // // int MyIMU::set_SystemOrientation(int8_t xyzw[4]) // Should use Down South West --> 0 0 -1 -1
    // // // // // {
    // // // // //     uint32_t orientation_quaturnion[4]{0, 0, 0, 0};
    // // // // //     for (int i = 0; i < 4; i++)
    // // // // //     {
    // // // // //         if (xyzw[i] == -1)
    // // // // //             orientation_quaturnion[i] = minus_Sqrt_2_div_2;
    // // // // //         if (xyzw[i] == 1)
    // // // // //             orientation_quaturnion[i] = Sqrt_2_div_2;
    // // // // //     }
    // // // // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::set_SystemOrientation(..) (wxyz) params=  0x%X  0x%X  0x%X  0x%X \n", orientation_quaturnion[0], orientation_quaturnion[1], orientation_quaturnion[2], orientation_quaturnion[3]);
    // // // // //     int rc = my_bno086.setFrs(SYSTEM_ORIENTATION, orientation_quaturnion, 4);
    // // // // //     return rc;
    // // // // // }

    // // // // // int MyIMU::get_SystemOrientation(uint32_t xyzw[4])
    // // // // // {
    // // // // //     uint32_t orientation_quaturnion[4]{0, 0, 0, 0};
    // // // // //     uint16_t count = 4;
    // // // // //     int rc = my_bno086.getFrs(SYSTEM_ORIENTATION, orientation_quaturnion, &count);
    // // // // //     if (rc != 0)
    // // // // //     {
    // // // // //         RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::get_SystemOrientation(..) !ERROR!  call to my_bno086.getFrs(..) FAIL. rc = %d \n", rc);
    // // // // //         return rc;
    // // // // //     }
    // // // // //     if (count != 4)
    // // // // //     {
    // // // // //         RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::get_SystemOrientation(..) !ERROR!  call to my_bno086.getFrs(..) rc=OK, but Count = %d \n", count);
    // // // // //         return -1;
    // // // // //     }

    // // // // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::get_SystemOrientation(..) Quaternion=  0x%X  0x%X  0x%X  0x%X \n", orientation_quaturnion[0], orientation_quaturnion[1], orientation_quaturnion[2], orientation_quaturnion[3]);
    // // // // //     for (int i = 0; i < 4; i++)
    // // // // //     {
    // // // // //         xyzw[i] = orientation_quaturnion[i];
    // // // // //     }
    // // // // //     return rc;
    // // // // // }

    // // // // // int MyIMU::clear_SystemOrientation()
    // // // // // {
    // // // // //     uint32_t orientation_quaturnion[4]{0, 0, 0, 0};
    // // // // //     int rc = my_bno086.setFrs(SYSTEM_ORIENTATION, orientation_quaturnion, 0); // Zero length means delete the record
    // // // // //     return rc;
    // // // // // }
    // // // // // //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // void MyIMU::BNO086_loop()
    // {
    //     if (my_bno086.wasReset())
    //     {
    //         uint8_t reason = my_bno086.getResetReason();
    //         RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO086 sensor was reset.....reason = %d  %s \n", reason, reset_reason_description(reason));
    //         BNO086_set_reports();
    //     }
    //     else
    //     {
    //         if (BNO086_get_data())
    //         {
    //             BNO086_check_send();
    //         }
    //     }

    //     my_bno086.serviceBus();

    //     if (elap_since_RateReport > elap_since_RateReport_span)
    //     {
    //         DisplayRateReport(elap_since_RateReport);
    //         elap_since_RateReport = 0;
    //     }
    //     if (elap_since_ConfigReport > elap_since_ConfigReport_span)
    //     {
    //         DisplayConfigReport();
    //         elap_since_ConfigReport = 0;
    //     }
    //     if (elap_since_MetaDataReport > elap_since_MetaDataReport_span)
    //     {
    //         DisplayMetaDataReport();
    //         elap_since_MetaDataReport = 0;
    //     }
    // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // void MyIMU::DisplayMetaDataReport()
    // {
    //     if (MetaDataReport_ndx >= sensor_ids_for_MetaDataReport_count)
    //     {
    //         elap_since_MetaDataReport_span = 1000 * 60 * 29; // 29 minutes
    //         return;
    //     }
    //     elap_since_MetaDataReport_span = 2000;
    //     uint8_t sensor_id = sensor_ids_for_MetaDataReport[MetaDataReport_ndx++];
    //     const char *pName = my_bno086.get_SensorName(sensor_id);
    //     sh2_SensorMetadata_t data;
    //     memset(&data, 0, sizeof(data));
    //     data.revision = 4;
    //     int rc = my_bno086.getMetadata(sensor_id, &data);
    //     if (rc == 0)
    //     {
    //         uint8_t meVersion = data.meVersion;
    //         uint8_t mhVersion = data.mhVersion;
    //         uint8_t shVersion = data.shVersion;
    //         uint16_t Q1 = data.qPoint1;
    //         uint32_t minPeriod_uS = data.minPeriod_uS;
    //         float range = my_bno086.qToFloat(data.range, Q1);
    //         float resolution = my_bno086.qToFloat(data.resolution, Q1);

    //         char vendor_id[50];
    //         memset(vendor_id, 0, sizeof(vendor_id));
    //         int sz = data.vendorIdLen;
    //         if (sz > (((int)sizeof(vendor_id)) - 2))
    //             sz = (int)(sizeof(vendor_id) - 2);
    //         memcpy(vendor_id, data.vendorId, sz);

    //         RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::DisplayMetaDataReport()  Sensor %27s -->  minPeriod_uS = %5d   range=%f   resolution=%f  meVer=%d mhVer=%d shVer=%d  Vendor=%s \n", pName, minPeriod_uS, range, resolution, meVersion, mhVersion, shVersion, vendor_id);
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::DisplayMetaDataReport() !!!ERROR!!! calling my_bno086.getMetadata(%d,..)  %s   Error_code = %d \n", sensor_id, pName, rc);
    //     }
    // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^ MAGNETIC_FIELD_CALIBRATED
    // // void MyIMU::DisplayConfigReport()
    // // {
    // //     if (ConfigReport_ndx >= sensor_ids_for_ConfigReport_count)
    // //     {
    // //         elap_since_ConfigReport_span = 1000 * 60 * 30; // 30 minutes
    // //         return;
    // //     }
    // //     elap_since_ConfigReport_span = 2000;
    // //     uint8_t sensor_id = sensor_ids_for_ConfigReport[ConfigReport_ndx++];
    // //     sh2_SensorConfig_t config;
    // //     memset(&config, 0, sizeof(config));
    // //     int rc = my_bno086.getSensorConfig(sensor_id, &config);
    // //     if (rc == 0)
    // //     {
    // //         const char *pName = my_bno086.get_SensorName(sensor_id);
    // //         RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::DisplayConfigReport()  Sensor %26s -->  reportInterval_us = %d   alwaysOnEnabled=%d   wakeupEnabled=%d \n", pName, config.reportInterval_us, config.alwaysOnEnabled, config.wakeupEnabled);
    // //     }
    // //     else
    // //     {
    // //         RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::DisplayConfigReport() !!!ERROR!!! calling my_bno086.getSensorConfig(%d,..)   Error_code = %d \n", sensor_id, rc);
    // //     }
    // // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // // void MyIMU::DisplayRateReport(uint32_t elap_ms)
    // // {
    // //     float elap_sec = ((float)elap_ms) / 1000.0F;

    // //     float rate_RotationVector = count_of_RotationVector / elap_sec;
    // //     float rate_LinearAccelerometer = count_of_LinearAccelerometer / elap_sec;
    // //     float rate_Accelerometer = count_of_Accelerometer / elap_sec;
    // //     float rate_Gyro = count_of_Gyro / elap_sec;
    // //     float rate_Magnetometer = count_of_Magnetometer / elap_sec;

    // //     int wave_RotationVector = elap_ms / count_of_RotationVector;
    // //     int wave_LinearAccelerometer = elap_ms / count_of_LinearAccelerometer;
    // //     int wave_Accelerometer = elap_ms / count_of_Accelerometer;
    // //     int wave_Gyro = elap_ms / count_of_Gyro;
    // //     int wave_Magnetometer = elap_ms / count_of_Magnetometer;

    // //     float rate_imu_callback = count_of_imu_callback / elap_sec;
    // //     float rate_imu2_callback = count_of_imu2_callback / elap_sec;

    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~ Data Rates per Second over the last %.1f seconds ~~~~~\n", elap_sec);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~       RotationVector:  %6.2f    %5d ms\n", rate_RotationVector, wave_RotationVector);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~  LinearAccelerometer:  %6.2f    %5d ms\n", rate_LinearAccelerometer, wave_LinearAccelerometer);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~        Accelerometer:  %6.2f    %5d ms\n", rate_Accelerometer, wave_Accelerometer);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~                 Gyro:  %6.2f    %5d ms\n", rate_Gyro, wave_Gyro);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~         Magnetometer:  %6.2f    %5d ms\n", rate_Magnetometer, wave_Magnetometer);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~         IMU Callback:  %6.2f \n", rate_imu_callback);
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "~~~~        IMU2 Callback:  %6.2f \n", rate_imu2_callback);
    // //     count_of_RotationVector = 0;
    // //     count_of_LinearAccelerometer = 0;
    // //     count_of_Accelerometer = 0;
    // //     count_of_Gyro = 0;
    // //     count_of_Magnetometer = 0;
    // //     count_of_imu_callback = 0;
    // //     count_of_imu2_callback = 0;
    // // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // // bool MyIMU::BNO086_get_data()
    // // {
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::BNO086_get_data() +++++++++++++++  starting ++++++++++++++++");
    // //     bool some_data_is_new = my_bno086.getSensorEvent(); // Has a new event come in on the Sensor Hub Bus?

    // //     if (some_data_is_new)
    // //     {
    // //         uint8_t event_id = my_bno086.getSensorEventID();
    // //         switch (event_id)
    // //         {
    // //         case SENSOR_REPORTID_ROTATION_VECTOR:
    // //             current_RotationVec_Quat[0] = my_bno086.getQuatI();
    // //             current_RotationVec_Quat[1] = my_bno086.getQuatJ();
    // //             current_RotationVec_Quat[2] = my_bno086.getQuatK();
    // //             current_RotationVec_Quat[3] = my_bno086.getQuatReal();
    // //             Rotation_Quaternion_Accuracy = my_bno086.getQuatRadianAccuracy();
    // //             rot_vec_is_dirty = true;
    // //             count_of_RotationVector++;
    // //             break;
    // //         case SENSOR_REPORTID_LINEAR_ACCELERATION:
    // //             current_LinearAccel_3D[0] = my_bno086.getLinAccelX();
    // //             current_LinearAccel_3D[1] = my_bno086.getLinAccelY();
    // //             current_LinearAccel_3D[2] = my_bno086.getLinAccelZ();
    // //             Linear_Acceleration_Accuracy = my_bno086.getLinAccelAccuracy();
    // //             lin_accel_is_dirty = true;
    // //             count_of_LinearAccelerometer++;
    // //             break;
    // //         case SENSOR_REPORTID_ACCELEROMETER:
    // //             current_Accel_3D[0] = my_bno086.getAccelX();
    // //             current_Accel_3D[1] = my_bno086.getAccelY();
    // //             current_Accel_3D[2] = my_bno086.getAccelZ();
    // //             Acceleration_Accuracy = my_bno086.getAccelAccuracy();
    // //             count_of_Accelerometer++;
    // //             accel_is_dirty = true;
    // //             break;
    // //         case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
    // //             current_Gyro_3D[0] = my_bno086.getGyroX();
    // //             current_Gyro_3D[1] = my_bno086.getGyroY();
    // //             current_Gyro_3D[2] = my_bno086.getGyroZ();
    // //             Gyro_Accuracy = my_bno086.getGyroAccuracy();
    // //             gyro_is_dirty = true;
    // //             count_of_Gyro++;
    // //             break;
    // //         case SENSOR_REPORTID_MAGNETIC_FIELD:
    // //             current_Mag_3D[0] = my_bno086.getMagX();
    // //             current_Mag_3D[1] = my_bno086.getMagY();
    // //             current_Mag_3D[2] = my_bno086.getMagZ();
    // //             Mag_Accuracy = my_bno086.getMagAccuracy();
    // //             mag_is_dirty = true;
    // //             count_of_Magnetometer++;
    // //             break;
    // //         default:
    // //             break;
    // //         }
    // //         count_of_BNO086_get_data++;
    // //     }
    // //     return some_data_is_new;
    // // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // // void MyIMU::BNO086_check_send()
    // // {
    // //     if (/*(accel_is_dirty) && (gyro_is_dirty) && (mag_is_dirty) &&*/ (lin_accel_is_dirty))
    // //     {
    // //         Do_Callback_IMU();
    // //         accel_is_dirty = false;
    // //         gyro_is_dirty = false;
    // //         mag_is_dirty = false;
    // //         lin_accel_is_dirty = false;
    // //     }
    // //     if (rot_vec_is_dirty)
    // //     {
    // //         Do_Callback_IMU2();
    // //         rot_vec_is_dirty = false;
    // //     }
    // // }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    // // void MyIMU::Do_Callback_IMU()
    // // {
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::Do_Callback_IMU() ++++++++++++++++");
    // //     if (imu_callback)
    // //     {
    // //         count_of_imu_callback++;
    // //         imu_callback(current_Accel_3D, current_Gyro_3D, current_Mag_3D, current_LinearAccel_3D);
    // //     }
    // // }

    // // void MyIMU::Do_Callback_IMU2()
    // // {
    // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "MyIMU::Do_Callback_IMU2() ++++++++++++++++");
    // //     if (imu2_callback)
    // //     {
    // //         count_of_imu2_callback++;
    // //         imu2_callback(current_RotationVec_Quat, Rotation_Quaternion_Accuracy);
    // //     }
    // // }

    // void MyIMU::time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns)
    // {
    //     uint64_t timestamp = time_ticks; /* Store the last timestamp */

    //     timestamp = timestamp * 15625; /* timestamp is now in nanoseconds */
    //     *s = (uint32_t)(timestamp / UINT64_C(1000000000));
    //     *ns = (uint32_t)(timestamp - ((*s) * UINT64_C(1000000000)));
    // }

    //~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=
    // MyIMU *pThis = nullptr;
    //~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=~-+=

} // end of namespace bno086_hardware_interface