// MyIMU.h

#pragma once

#include <functional>
#include "elapsedMillis.h"
#include "SPI.hpp"
#include "BNO08x.hpp"

namespace bno086_hardware_interface
{

  //void Delay(uint32_t delay_ms);


#define IMU_BOOTN_pin 4
#define IMU_WAKE_pin 5
#define IMU_RESET_pin 6
#define IMU_INT_pin 9
#define IMU_CS_pin 10
#define IMU_REPORT_INTERVAL 14

//............................
//...
//... Slot_IMU1c pin numbers  //ToDo:
//...
#ifndef CS_PIN
#define CS_PIN 10
#endif

#ifndef IMU_INT1_pin
#define IMU_INT1_pin 9
#endif

#ifndef IMU_INT2_pin
#define IMU_INT2_pin 6
#endif

  //............................
  // // using IMU_Callback_t = std::function<void(const float Accel_3D[3], const float Gyro_3D[3], const float Mag_3D[3], const float LinearAccel_3D[3])>;
  // // using IMU2_Callback_t = std::function<void(const float orientation_xyzw[4], float Accuracy)>;

  //............................
  // class MyIMU
  // {
  // public:
  //   // bool begin(uint8_t irq_pin = IMU_INT1_pin); // return TRUE if ok
  //   // void spin();
  //   // uint8_t setDebugLevel(uint8_t lvl)
  //   // {
  //   //   uint8_t old = debugLevel;
  //   //   debugLevel = lvl;
  //   //   return old;
  //   // }

  //   // void set_IMU_callback(IMU_Callback_t cb) { imu_callback = cb; }
  //   // void set_IMU2_callback(IMU2_Callback_t cb) { imu2_callback = cb; }

  //   // bool tareNow();
  //   // bool saveTare();
  //   // bool clearTare();

  //   // int set_SystemOrientation(int8_t xyzw[4]);
  //   // int clear_SystemOrientation();
  //   // int get_SystemOrientation(uint32_t xyzw[4]);

  //   // // // float qToFloat(int32_t fixedPointValue, uint8_t qPoint) { return my_bno086.qToFloat(fixedPointValue, qPoint); } // Given a Q value, converts fixed point floating to regular floating point number
  //   // // // uint32_t Float_to_Q(float floatValue, uint8_t qPoint) { return my_bno086.Float_to_Q(floatValue, qPoint); }

  // private:
  //   SPIClass spiPort;
  //   BNO08x my_bno086;
  //   // bool lin_accel_is_dirty = false;
  //   // bool accel_is_dirty = false;
  //   // bool gyro_is_dirty = false;
  //   // bool mag_is_dirty = false;
  //   // bool rot_vec_is_dirty = false;
  //   // void BNO086_set_reports();
  //   // void BNO086_loop();
  //   // bool BNO086_get_data();
  //   // void BNO086_check_send();
  //   // const char *reset_reason_description(uint8_t reason);
  //   // long count_of_BNO086_get_data = 0;

  //   // elapsedMillis elap_since_DumpIntCounts;
  //   // uint32_t elap_since_DumpIntCounts_span = 1000 * 75;

  //   // elapsedMillis elap_since_RateReport;
  //   // const uint32_t elap_since_RateReport_span = 1000 * 90;
    
  //   // int count_of_RotationVector = 0;
  //   // int count_of_Accelerometer = 0;
  //   // int count_of_LinearAccelerometer = 0;
  //   // int count_of_Gyro = 0;
  //   // int count_of_Magnetometer = 0;
  //   // void DisplayRateReport(uint32_t elap_ms);
  //   // //.................................................................
  //   // elapsedMillis elap_since_ConfigReport;
  //   // uint32_t elap_since_ConfigReport_span = 1000 * 40;
  //   // int ConfigReport_ndx = 0;
  //   // const uint8_t sensor_ids_for_ConfigReport[5]{SH2_ACCELEROMETER, SH2_LINEAR_ACCELERATION, SH2_GYROSCOPE_CALIBRATED, SH2_MAGNETIC_FIELD_CALIBRATED,
  //   //                                              SH2_ROTATION_VECTOR};
  //   // const int sensor_ids_for_ConfigReport_count = sizeof(sensor_ids_for_ConfigReport);
  //   // void DisplayConfigReport();

  //   // //.................................................................
  //   // elapsedMillis elap_since_MetaDataReport;
  //   // uint32_t elap_since_MetaDataReport_span = 1000 * 75;
  //   // int MetaDataReport_ndx = 0;
  //   // const uint8_t sensor_ids_for_MetaDataReport[5]{SH2_ACCELEROMETER, SH2_LINEAR_ACCELERATION, SH2_GYROSCOPE_CALIBRATED, SH2_MAGNETIC_FIELD_CALIBRATED,
  //   //                                                SH2_ROTATION_VECTOR};
  //   // const int sensor_ids_for_MetaDataReport_count = sizeof(sensor_ids_for_MetaDataReport);
  //   // void DisplayMetaDataReport();

  //   // uint8_t irq_pin;
  //   // uint8_t debugLevel = 3;

  //   // // // IMU_Callback_t imu_callback = nullptr;
  //   // // // IMU2_Callback_t imu2_callback = nullptr;

  //   // // // int count_of_imu_callback = 0;
  //   // // // int count_of_imu2_callback = 0;

  //   // void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns);

  //   // uint32_t loop_count = 0;

  //   //~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=
  //   //~-=~-=
  //   //~-=~-= current values
  //   //~-=~-=
  //   float current_Accel_3D[3];
  //   float current_Gyro_3D[3];
  //   float current_Mag_3D[3]{0, 0, 0};
  //   float current_LinearAccel_3D[3]{0, 0, 0};
  //   float current_RotationVec_Quat[4]{0, 0, 0, 0}; // I,J,K,real

  // public:
  //   float Rotation_Quaternion_Accuracy = 0;
  //   float Linear_Acceleration_Accuracy = 0;
  //   float Acceleration_Accuracy = 0;
  //   float Gyro_Accuracy = 0;
  //   float Mag_Accuracy = 0;

  // private:
  //   //~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=
  //   // void Do_Callback_IMU();  // maybe
  //   // void Do_Callback_IMU2(); // maybe
  //                            //~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=~-=
  // };

} // end of namespace bno086_hardware_interface