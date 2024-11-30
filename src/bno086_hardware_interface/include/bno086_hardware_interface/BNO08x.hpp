/*
  This is a library written for the BNO08x
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x Arduino Library by Bryan Siepert
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  The BNO08x IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO08x and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library

  Development environment specifics:
  Arduino IDE 2.1.1

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Some of this library was based off of the Adafruit BNO08x Arduino Library.
  More specifically, the code layers connecting to the HillCrest/Ceva Driver.
  Their original work can be found here:
  https://github.com/adafruit/Adafruit_BNO08x
  Thank you Adafruit and your developers for all your hard work put into your Library!
*/

#pragma once

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#include "SPI.hpp"
#include "gpio.hpp"

namespace bno086_hardware_interface
{

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// All the ways we can configure or talk to the BNO08x, figure 34, page 36 reference manual
// These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// All the different sensors and features we can get reports from
// These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER SH2_ACCELEROMETER
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED SH2_GYROSCOPE_CALIBRATED
#define SENSOR_REPORTID_MAGNETIC_FIELD SH2_MAGNETIC_FIELD_CALIBRATED
#define SENSOR_REPORTID_LINEAR_ACCELERATION SH2_LINEAR_ACCELERATION
#define SENSOR_REPORTID_ROTATION_VECTOR SH2_ROTATION_VECTOR
#define SENSOR_REPORTID_GRAVITY SH2_GRAVITY
#define SENSOR_REPORTID_UNCALIBRATED_GYRO SH2_GYROSCOPE_UNCALIBRATED
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR SH2_GYRO_INTEGRATED_RV
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER SH2_STEP_COUNTER
#define SENSOR_REPORTID_STABILITY_CLASSIFIER SH2_STABILITY_CLASSIFIER
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER SH2_PERSONAL_ACTIVITY_CLASSIFIER
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

// Command IDs from section 6.4, page 42
// These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z 0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

  class BNO08x
  {
  public:
    bool beginSPI(uint8_t user_INTPin,
                  uint8_t user_RSTPin,
                  uint8_t user_WAKEPin,
                  std::shared_ptr<SPIClass> spiPort);

    //-------- HAL interface for the SH2 library ---------
    int spihal_open(sh2_Hal_t *self);
    void spihal_close(sh2_Hal_t *self);
    int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    uint32_t hal_getTimeUs(sh2_Hal_t *self);
    bool spi_read(uint8_t *buffer, size_t len, uint8_t sendvalue);
    bool spi_write(const uint8_t *buffer, size_t len, const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
    bool hal_wait_for_int(void);
    void hal_hardwareReset();
    //----------------------------------------------------

    void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
    void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);

    sh2_ProductIds_t prodIds;      ///< The product IDs returned by the sensor
    sh2_SensorValue_t sensorValue; // data may be overwritten by the next sensor event

    bool modeOn();    // Use the executable channel to turn the BNO on
    bool modeSleep(); // Use the executable channel to put the BNO to sleep
    bool wake_up();  // Use the WAKE pin to wake up the BNO from sleep
    void drain_spi(); // Drain the SPI bus of any data

    bool softReset(); // Try to reset the IMU via software
    void hardwareReset();
    bool wasReset();          // Returns true if the sensor has reported a reset. Reading this will unflag the reset.
    uint8_t resetReason();    // Query the IMU for the reason it last reset
    uint8_t getResetReason(); // returns prodIds->resetCause

    bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000, uint32_t sensorSpecific = 0);

    bool getSensorEvent();      //?????
    uint8_t getSensorEventID(); //???????

    const char *get_SensorName(sh2_SensorId_t sensor_id);

    bool serviceBus(void); // Service the SH2 device, reading any data that is available and dispatching callbacks.  This function should be called periodically by the host system to service an open sensor hub.
    void save_sensor_data();

    bool enableRotationVector(uint16_t timeBetweenReports = 10);
    bool enableAccelerometer(uint16_t timeBetweenReports = 10);
    bool enableLinearAccelerometer(uint16_t timeBetweenReports = 10);
    bool enableGravity(uint16_t timeBetweenReports = 10);
    bool enableGyro(uint16_t timeBetweenReports = 10);
    bool enableMagnetometer(uint16_t timeBetweenReports = 10);

    void getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy);
    float getQuatI();
    float getQuatJ();
    float getQuatK();
    float getQuatReal();
    float getQuatRadianAccuracy();
    uint8_t getQuatAccuracy();

    void getAccel(float &x, float &y, float &z, uint8_t &accuracy);
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    uint8_t getAccelAccuracy();

    void getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);
    float getLinAccelX();
    float getLinAccelY();
    float getLinAccelZ();
    uint8_t getLinAccelAccuracy();

    void getGyro(float &x, float &y, float &z, uint8_t &accuracy);
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    uint8_t getGyroAccuracy();

    void getMag(float &x, float &y, float &z, uint8_t &accuracy);
    float getMagX();
    float getMagY();
    float getMagZ();
    uint8_t getMagAccuracy();

    void getGravity(float &x, float &y, float &z, uint8_t &accuracy);
    float getGravityX();
    float getGravityY();
    float getGravityZ();
    uint8_t getGravityAccuracy();

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    /**
     * @brief Set an FRS record
     *
     * @param  recordId Which FRS Record to set.
     * @param  pData pointer to buffer containing the new data.
     * @param  words number of 32-bit words to write.  (0 to delete record.)
     * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
     */
    int setFrs(uint16_t recordId, uint32_t *pData, uint16_t words);

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    /**
     * @brief Get an FRS record.
     *
     * @param  recordId Which FRS Record to retrieve.
     * @param  pData pointer to buffer to receive the results
     * @param[in] words Size of pData buffer, in 32-bit words.
     * @param[out] words Number of 32-bit words retrieved.
     * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
     */
    int getFrs(uint16_t recordId, uint32_t *pData, uint16_t *words);

    bool tareNow(bool zAxis = false, sh2_TareBasis_t basis = SH2_TARE_BASIS_ROTATION_VECTOR);
    bool saveTare();
    bool clearTare();
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    float getRoll();
    float getPitch();
    float getYaw();

    /**
     * @brief Get sensor configuration.
     *
     * @param  sensorId Which sensor to query.
     * @param  config SensorConfig structure to store results.
     * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
     */
    int getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *config);

    /**
     * @brief Get metadata related to a sensor.
     *
     * @param  sensorId Which sensor to query.
     * @param  pData Pointer to structure to receive the results.
     * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
     */
    int getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData);

    int set_SystemOrientation(int8_t xyzw[4]);
    int clear_SystemOrientation();
    int get_SystemOrientation(uint32_t xyzw[4]);

  private:
    int _int_pin;
    int _reset_pin;
    int _wake_pin;

    gpio gpio_int;
    gpio gpio_reset;
    gpio gpio_wake;

    std::shared_ptr<SPIClass> spiPort;

    uint32_t spiPortSpeed; // Optional user defined port speed

    sh2_RotationVectorWAcc_t rotationVector;
    sh2_Accelerometer_t accelerometer;
    sh2_Accelerometer_t linearAcceleration;
    sh2_Accelerometer_t gravity;
    sh2_Gyroscope_t gyroscope;
    sh2_MagneticField_t magneticField;

    uint8_t accuracy_from_status(uint8_t status)
    {
      return (status & 0x03);
    }
    uint8_t rotationVector_status;
    uint8_t accelerometer_status;
    uint8_t linearAcceleration_status;
    uint8_t gravity_status;
    uint8_t gyroscope_status;
    uint8_t magneticField_status;

    uint64_t rotationVector_timestamp;
    uint64_t accelerometer_timestamp;
    uint64_t linearAcceleration_timestamp;
    uint64_t gravity_timestamp;
    uint64_t gyroscope_timestamp;
    uint64_t magneticField_timestamp;

    float qToFloat(int32_t fixedPointValue, uint8_t qPoint); // Given a Q value, converts fixed point floating to regular floating point number
    uint32_t Float_to_Q(float floatValue, uint8_t qPoint);
    // These Q values are defined in the datasheet but can also be obtained by querying the meta data records
    // See the read metadata example for more info
    int16_t rotationVector_Q1 = 14;
    int16_t rotationVectorAccuracy_Q1 = 12; // Heading accuracy estimate in radians. The Q point is 12.
    int16_t accelerometer_Q1 = 8;
    int16_t linear_accelerometer_Q1 = 8;
    int16_t gyro_Q1 = 9;
    int16_t magnetometer_Q1 = 4;
    int16_t angular_velocity_Q1 = 10;
    int16_t gravity_Q1 = 8;

    const char *SensorName(sh2_SensorId_t sensor_id);
    // These are the raw sensor values (without Q applied) pulled from the user requested Input Report
    // // uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
    // // uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
    // // uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
    // // uint16_t rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
    // // uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
    // // uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
    // // uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
    // // uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
    // // uint8_t tapDetector;
    // // uint16_t stepCount;
    uint32_t timeStamp;
    // // uint8_t stabilityClassifier;
    // // uint8_t activityClassifier;
    // // uint8_t calibrationStatus;                            // Byte R0 of ME Calibration Response
    // // uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; // Raw readings from MEMS sensor
    // // uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;    // Raw readings from MEMS sensor
    // // uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;       // Raw readings from MEMS sensor

  protected:
    virtual bool _init(int32_t sensor_id = 0);
    sh2_Hal_t _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
  };

} // end of namespace bno086_hardware_interface