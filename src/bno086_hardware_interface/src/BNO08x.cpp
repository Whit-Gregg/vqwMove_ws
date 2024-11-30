// hal_callback and sensorHandler and sh2_decodeSensorEvent

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
  Arduino IDE 1.8.5

  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  Some of this library was based off of the Adafruit BNO08x Arduino Library.
  More specifically, the code layers connecting to the HillCrest/Ceva Driver.
  Their original work can be found here:
  https://github.com/adafruit/Adafruit_BNO08x
  Thank you Adafruit and your developers for all your hard work put into your Library!
*/

#include <cstdint>
#include <chrono>
#include "BNO08x.hpp"
#include "gpio.hpp"
// #include <DebugLog.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

namespace bno086_hardware_interface
{

    static sh2_SensorValue_t *_sensor_value = NULL;
    static bool _reset_occurred = false;

    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+

    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
    bool BNO08x::hal_wait_for_int(void)
    {
        if (_int_pin == -1)
            return true;

        const int timeout_ms = 1000;
        bool isOK = gpio_int.wait_until_LOW(timeout_ms);
        // // if (!isOK)
        // // {
        // //     RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "hal_wait_for_int() !!!! Timed out!  _int_pin=%d\n", _int_pin);
        // // }
        return isOK;
    }
    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+

    // Initializes the sensor with basic settings using SPI
    // Returns false if sensor is not detected
    bool BNO08x::beginSPI(uint8_t user_INTPin, uint8_t user_RSTPin, uint8_t user_WAKEPin, std::shared_ptr<SPIClass> spiPort_)
    {
        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::beginSPI(int=%d, reset=%d, wake=%d, spi=%s) called", user_INTPin, user_RSTPin, user_WAKEPin, spiPort_->get_spiDeviceName().c_str());
        // Get user settings
        spiPort = spiPort_;

        _int_pin = user_INTPin;
        _reset_pin = user_RSTPin;
        _wake_pin = user_WAKEPin;

        gpio_int.Initialize(_int_pin, pin_direction::INPUT);
        gpio_reset.Initialize(_reset_pin, pin_direction::OUTPUT);
        gpio_wake.Initialize(_wake_pin, pin_direction::OUTPUT);

        _HAL.cookie = this;

        _HAL.open = [](sh2_Hal_t *self) -> int
        {
            return static_cast<BNO08x *>(self->cookie)->spihal_open(self);
        };

        _HAL.close = [](sh2_Hal_t *self) -> void
        {
            static_cast<BNO08x *>(self->cookie)->spihal_close(self);
        };

        _HAL.read = [](sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) -> int
        {
            return static_cast<BNO08x *>(self->cookie)->spihal_read(self, pBuffer, len, t_us);
        };

        _HAL.write = [](sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) -> int
        {
            return static_cast<BNO08x *>(self->cookie)->spihal_write(self, pBuffer, len);
        };

        _HAL.getTimeUs = [](sh2_Hal_t *self) -> uint32_t
        {
            return static_cast<BNO08x *>(self->cookie)->hal_getTimeUs(self);
        };

        _sensor_value = &sensorValue;

        return _init();
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /**
     * @brief Get sensor configuration.
     *
     * @param  sensorId Which sensor to query.
     * @param  config SensorConfig structure to store results.
     * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
     */
    int BNO08x::getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *config) { return sh2_getSensorConfig(sensorId, config); }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /**
     * @brief Get metadata related to a sensor.
     *
     * @param  sensorId Which sensor to query.
     * @param  pData Pointer to structure to receive the results.
     * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
     */
    int BNO08x::getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData) { return sh2_getMetadata(sensorId, pData); }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Quaternion to Euler conversion
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440
    // Return the roll (rotation around the x-axis) in Radians
    float BNO08x::getRoll()
    {
        float dqw = getQuatReal();
        float dqx = getQuatI();
        float dqy = getQuatJ();
        float dqz = getQuatK();

        float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
        dqw = dqw / norm;
        dqx = dqx / norm;
        dqy = dqy / norm;
        dqz = dqz / norm;

        float ysqr = dqy * dqy;

        // roll (x-axis rotation)
        float t0 = +2.0 * (dqw * dqx + dqy * dqz);
        float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
        float roll = atan2(t0, t1);

        return (roll);
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Return the pitch (rotation around the y-axis) in Radians
    float BNO08x::getPitch()
    {
        float dqw = getQuatReal();
        float dqx = getQuatI();
        float dqy = getQuatJ();
        float dqz = getQuatK();

        float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
        dqw = dqw / norm;
        dqx = dqx / norm;
        dqy = dqy / norm;
        dqz = dqz / norm;

        // float ysqr = dqy * dqy;

        // pitch (y-axis rotation)
        float t2 = +2.0 * (dqw * dqy - dqz * dqx);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        float pitch = asin(t2);

        return (pitch);
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Return the yaw / heading (rotation around the z-axis) in Radians
    float BNO08x::getYaw()
    {
        float dqw = getQuatReal();
        float dqx = getQuatI();
        float dqy = getQuatJ();
        float dqz = getQuatK();

        float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
        dqw = dqw / norm;
        dqx = dqx / norm;
        dqy = dqy / norm;
        dqz = dqz / norm;

        float ysqr = dqy * dqy;

        // yaw (z-axis rotation)
        float t3 = +2.0 * (dqw * dqz + dqx * dqy);
        float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
        float yaw = atan2(t3, t4);

        return (yaw);
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Gets the full quaternion
    // i,j,k,real output floats
    void BNO08x::getQuat(float &i, float &j, float &k, float &real, float &radAccuracy, uint8_t &accuracy)
    {
        i = rotationVector.i;
        j = rotationVector.j;
        k = rotationVector.k;
        real = rotationVector.real;
        accuracy = getQuatAccuracy();
        radAccuracy = getQuatRadianAccuracy();
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Return the rotation vector quaternion I
    float BNO08x::getQuatI()
    {
        // float quat = qToFloat(rawQuatI, rotationVector_Q1);
        // if (_printDebug == true)
        // {
        // 	if ((quat < -1.0) || (quat > 1.0))
        // 	{
        // 		_debugPort->print(F("getQuatI: quat: ")); // Debug the occasional non-unitary Quat
        // 		_debugPort->print(quat, 2);
        // 		_debugPort->print(F(" rawQuatI: "));
        // 		_debugPort->print(rawQuatI);
        // 		_debugPort->print(F(" rotationVector_Q1: "));
        // 		_debugPort->println(rotationVector_Q1);
        // 	}
        // }
        // return (quat);
        return rotationVector.i;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Return the rotation vector quaternion J
    float BNO08x::getQuatJ()
    {
        // float quat = qToFloat(rawQuatJ, rotationVector_Q1);
        //  // if (_printDebug == true)
        //  //     {
        //  //         if ((quat < -1.0) || (quat > 1.0))       // Debug the occasional non-unitary Quat
        //  //             {
        //  //                 _debugPort->print(F("getQuatJ: quat: "));
        //  //                 _debugPort->print(quat, 2);
        //  //                 _debugPort->print(F(" rawQuatJ: "));
        //  //                 _debugPort->print(rawQuatJ);
        //  //                 _debugPort->print(F(" rotationVector_Q1: "));
        //  //                 _debugPort->println(rotationVector_Q1);
        //  //             }
        //  //     }
        //  return (quat);
        return rotationVector.j;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Return the rotation vector quaternion K
    float BNO08x::getQuatK()
    {
        // float quat = qToFloat(rawQuatK, rotationVector_Q1);
        //  // if (_printDebug == true)
        //  //     {
        //  //         if ((quat < -1.0) || (quat > 1.0))       // Debug the occasional non-unitary Quat
        //  //             {
        //  //                 _debugPort->print(F("getQuatK: quat: "));
        //  //                 _debugPort->print(quat, 2);
        //  //                 _debugPort->print(F(" rawQuatK: "));
        //  //                 _debugPort->print(rawQuatK);
        //  //                 _debugPort->print(F(" rotationVector_Q1: "));
        //  //                 _debugPort->println(rotationVector_Q1);
        //  //             }
        //  //     }
        //  return (quat);
        return rotationVector.k;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Return the rotation vector quaternion Real
    float BNO08x::getQuatReal() { return rotationVector.real; }

    // Return the rotation vector radian accuracy
    float BNO08x::getQuatRadianAccuracy() { return 0.5; } // ToDo:

    // Return the rotation vector sensor event report status accuracy
    uint8_t BNO08x::getQuatAccuracy() { return accuracy_from_status(rotationVector_status); } // ToDo:

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Gets the full acceleration
    // x,y,z output floats
    void BNO08x::getAccel(float &x, float &y, float &z, uint8_t &accuracy)
    {
        x = accelerometer.x;
        y = accelerometer.y;
        z = accelerometer.z;
        accuracy = getAccelAccuracy();
    }

    // Return the acceleration component
    float BNO08x::getAccelX() { return accelerometer.x; }

    // Return the acceleration component
    float BNO08x::getAccelY() { return accelerometer.y; }

    // Return the acceleration component
    float BNO08x::getAccelZ() { return accelerometer.z; }

    // Return the acceleration component
    uint8_t BNO08x::getAccelAccuracy() { return accuracy_from_status(accelerometer_status); } // ToDo:

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // linear acceleration, i.e. minus gravity
    // Gets the full lin acceleration
    // x,y,z output floats
    void BNO08x::getLinAccel(float &x, float &y, float &z, uint8_t &accuracy)
    {
        x = linearAcceleration.x;
        y = linearAcceleration.y;
        z = linearAcceleration.z;
        accuracy = getLinAccelAccuracy();
    }

    // Return the acceleration component
    float BNO08x::getLinAccelX() { return linearAcceleration.x; }

    // Return the acceleration component
    float BNO08x::getLinAccelY() { return linearAcceleration.y; }

    // Return the acceleration component
    float BNO08x::getLinAccelZ() { return linearAcceleration.z; }

    // Return the acceleration component
    uint8_t BNO08x::getLinAccelAccuracy() { return accuracy_from_status(linearAcceleration_status); }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Gets the full gyro vector
    // x,y,z output floats
    void BNO08x::getGyro(float &x, float &y, float &z, uint8_t &accuracy)
    {
        x = gyroscope.x;
        y = gyroscope.y;
        z = gyroscope.z;
        accuracy = getGyroAccuracy();
    }

    // Return the gyro component
    float BNO08x::getGyroX() { return gyroscope.x; }

    // Return the gyro component
    float BNO08x::getGyroY() { return gyroscope.y; }

    // Return the gyro component
    float BNO08x::getGyroZ() { return gyroscope.z; }

    // Return the gyro component
    uint8_t BNO08x::getGyroAccuracy() { return accuracy_from_status(gyroscope_status); }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Gets the full gravity vector
    // x,y,z output floats
    void BNO08x::getGravity(float &x, float &y, float &z, uint8_t &accuracy)
    {
        x = gravity.x;
        y = gravity.y;
        z = gravity.z;
        accuracy = getGravityAccuracy();
    }

    float BNO08x::getGravityX() { return gravity.x; }

    // Return the gravity component
    float BNO08x::getGravityY() { return gravity.y; }

    // Return the gravity component
    float BNO08x::getGravityZ() { return gravity.z; }

    uint8_t BNO08x::getGravityAccuracy() { return accuracy_from_status(gravity_status); } // ToDo:

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Gets the full mag vector
    // x,y,z output floats
    void BNO08x::getMag(float &x, float &y, float &z, uint8_t &accuracy)
    {
        x = magneticField.x;
        y = magneticField.y;
        z = magneticField.z;
        accuracy = getMagAccuracy();
    }

    // Return the magnetometer component
    float BNO08x::getMagX() { return magneticField.x; }

    // Return the magnetometer component
    float BNO08x::getMagY() { return magneticField.y; }

    // Return the magnetometer component
    float BNO08x::getMagZ() { return magneticField.z; }

    // Return the mag component
    uint8_t BNO08x::getMagAccuracy() { return accuracy_from_status(magneticField_status); } // ToDo:

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // this should be called periodically to update the sensor data, perhaps from a timer setup on this node...
    bool BNO08x::serviceBus(void)
    {
        sh2_service();
        return true;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Send command to reset IC
    bool BNO08x::softReset(void)
    {
        int status = sh2_devReset();

        if (status != SH2_OK)
        {
            return false;
        }
        return true;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Set the operating mode to "On"
    //(This one is for @jerabaul29)
    bool BNO08x::modeOn(void)
    {
        int status = sh2_devOn();

        if (status != SH2_OK)
        {
            return false;
        }
        return true;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Set the operating mode to "Sleep"
    //(This one is for @jerabaul29)
    bool BNO08x::modeSleep(void)
    {
        int status = sh2_devSleep();

        if (status != SH2_OK)
        {
            return false;
        }
        return true;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Get the reason for the last reset
    // 1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
    uint8_t BNO08x::getResetReason() { return prodIds.entry[0].resetCause; }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Given a register value and a Q point, convert to float
    // See https://en.wikipedia.org/wiki/Q_(number_format)
    float BNO08x::qToFloat(int32_t fixedPointValue, uint8_t qPoint)
    {
        double qFloat = fixedPointValue;
        double div = pow(2, qPoint);
        qFloat = qFloat / div;
        return (float)(qFloat);
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    uint32_t BNO08x::Float_to_Q(float floatValue, uint8_t qPoint)
    {
        double qFloat = floatValue;
        double ppooww = pow(2, qPoint);
        // RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"),"-------  ppooww = %.12f \n", ppooww);
        qFloat *= ppooww;
        // RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"),"-------  floatValue * ppooww = %.12f \n", qFloat);
        int64_t result64 = (int64_t)qFloat;
        // RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"),"-------  result64 = %lld  0x%llX \n", result64, result64);
        uint32_t result = (uint32_t)result64;
        return result;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Sends the packet to enable the rotation vector
    bool BNO08x::enableRotationVector(uint16_t timeBetweenReports)
    {
        timeBetweenReports = timeBetweenReports * 1000; // ms to us
        return enableReport(SH2_ROTATION_VECTOR, timeBetweenReports);
    }

    // Sends the packet to enable the accelerometer
    bool BNO08x::enableAccelerometer(uint16_t timeBetweenReports)
    {
        timeBetweenReports = timeBetweenReports * 1000; // ms to us
        return enableReport(SH2_ACCELEROMETER, timeBetweenReports);
    }

    // Sends the packet to enable the accelerometer
    bool BNO08x::enableLinearAccelerometer(uint16_t timeBetweenReports)
    {
        timeBetweenReports = timeBetweenReports * 1000; // ms to us
        return enableReport(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
    }

    // Sends the packet to enable the gravity vector
    bool BNO08x::enableGravity(uint16_t timeBetweenReports)
    {
        timeBetweenReports = timeBetweenReports * 1000; // ms to us
        return enableReport(SENSOR_REPORTID_GRAVITY, timeBetweenReports);
    }

    // Sends the packet to enable the gyro
    bool BNO08x::enableGyro(uint16_t timeBetweenReports)
    {
        timeBetweenReports = timeBetweenReports * 1000; // ms to us
        return enableReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, timeBetweenReports);
    }

    // Sends the packet to enable the magnetometer
    bool BNO08x::enableMagnetometer(uint16_t timeBetweenReports)
    {
        timeBetweenReports = timeBetweenReports * 1000; // ms to us
        return enableReport(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    bool BNO08x::tareNow(bool zAxis, sh2_TareBasis_t basis)
    {
        int status = sh2_setTareNow(zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL, basis);

        if (status != SH2_OK)
        {
            return false;
        }

        return true;
    }

    bool BNO08x::saveTare()
    {
        int status = sh2_persistTare();

        if (status != SH2_OK)
        {
            return false;
        }

        return true;
    }

    bool BNO08x::clearTare()
    {
        int status = sh2_clearTare();

        if (status != SH2_OK)
        {
            return false;
        }

        return true;
    }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    int BNO08x::getFrs(uint16_t recordId, uint32_t *pData, uint16_t *words) { return sh2_getFrs(recordId, pData, words); }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    int BNO08x::setFrs(uint16_t recordId, uint32_t *pData, uint16_t words) { return sh2_setFrs(recordId, pData, words); }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    // //This tells the BNO08x to begin calibrating
    // //See page 50 of reference manual and the 1000-4044 calibration doc
    // void BNO08x::sendCalibrateCommand(uint8_t thingToCalibrate)
    // {
    // 	/*shtpData[3] = 0; //P0 - Accel Cal Enable
    // 	shtpData[4] = 0; //P1 - Gyro Cal Enable
    // 	shtpData[5] = 0; //P2 - Mag Cal Enable
    // 	shtpData[6] = 0; //P3 - Subcommand 0x00
    // 	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
    // 	shtpData[8] = 0; //P5 - Reserved
    // 	shtpData[9] = 0; //P6 - Reserved
    // 	shtpData[10] = 0; //P7 - Reserved
    // 	shtpData[11] = 0; //P8 - Reserved*/

    // 	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
    // 		shtpData[x] = 0;

    // 	if (thingToCalibrate == CALIBRATE_ACCEL)
    // 		shtpData[3] = 1;
    // 	else if (thingToCalibrate == CALIBRATE_GYRO)
    // 		shtpData[4] = 1;
    // 	else if (thingToCalibrate == CALIBRATE_MAG)
    // 		shtpData[5] = 1;
    // 	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
    // 		shtpData[7] = 1;
    // 	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
    // 	{
    // 		shtpData[3] = 1;
    // 		shtpData[4] = 1;
    // 		shtpData[5] = 1;
    // 	}
    // 	else if (thingToCalibrate == CALIBRATE_STOP)
    // 	{
    // 		; //Do nothing, bytes are set to zero
    // 	}

    // 	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
    // 	calibrationStatus = 1;

    // 	//Using this shtpData packet, send a command
    // 	sendCommand(COMMAND_ME_CALIBRATE);
    // }

    // //Request ME Calibration Status from BNO08x
    // //See page 51 of reference manual
    // void BNO08x::requestCalibrationStatus()
    // {
    // 	/*shtpData[3] = 0; //P0 - Reserved
    // 	shtpData[4] = 0; //P1 - Reserved
    // 	shtpData[5] = 0; //P2 - Reserved
    // 	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
    // 	shtpData[7] = 0; //P4 - Reserved
    // 	shtpData[8] = 0; //P5 - Reserved
    // 	shtpData[9] = 0; //P6 - Reserved
    // 	shtpData[10] = 0; //P7 - Reserved
    // 	shtpData[11] = 0; //P8 - Reserved*/

    // 	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
    // 		shtpData[x] = 0;

    // 	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

    // 	//Using this shtpData packet, send a command
    // 	sendCommand(COMMAND_ME_CALIBRATE);
    // }

    // //This tells the BNO08x to save the Dynamic Calibration Data (DCD) to flash
    // //See page 49 of reference manual and the 1000-4044 calibration doc
    // void BNO08x::saveCalibration()
    // {
    // 	/*shtpData[3] = 0; //P0 - Reserved
    // 	shtpData[4] = 0; //P1 - Reserved
    // 	shtpData[5] = 0; //P2 - Reserved
    // 	shtpData[6] = 0; //P3 - Reserved
    // 	shtpData[7] = 0; //P4 - Reserved
    // 	shtpData[8] = 0; //P5 - Reserved
    // 	shtpData[9] = 0; //P6 - Reserved
    // 	shtpData[10] = 0; //P7 - Reserved
    // 	shtpData[11] = 0; //P8 - Reserved*/

    // 	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
    // 		shtpData[x] = 0;

    // 	//Using this shtpData packet, send a command
    // 	sendCommand(COMMAND_DCD); //Save DCD command
    // }

    void BNO08x::drain_spi()
    {
        if (!hal_wait_for_int())
        {
            if (!hal_wait_for_int())
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "drain_spi(..)  --- No Data available to drain --- ");
                return;
            }
        }

        uint8_t pBuffer[8];
        int read_count = 0;
        bool isDone = false;

        while (!isDone)
        {
            memset(pBuffer, 0, sizeof(pBuffer));

            if (!spi_read(pBuffer, 1, 0x00))
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "drain_spi(..)  spi_read(..) FAIL.!!");
                return;
            }
            if (pBuffer[0] == 0x00)
            {
                if (!hal_wait_for_int())
                    isDone = true;
                if (read_count > 100)
                {
                    isDone = true;
                }
            }
            else
            {
                read_count++;
            }
        }
        if (read_count > 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "drain_spi(..)  --- %d bytes drained --- ", read_count);
        }
    }

    bool BNO08x::wake_up() // returns true if the chip is awake
    {
        const uint32_t time_out_ms = 30; // <<---- time to wait for the chip to wake up

        pin_value val = gpio_int.get_pin_value();
        if (val == pin_value::LOW)
        {
            return true;
        }
        gpio_wake.set_pin_value(pin_value::LOW); // turn on WAKE function
        elapsedMillis elap;
        while (elap < time_out_ms)
        {
            val = gpio_int.get_pin_value();
            if (val == pin_value::LOW)
            {
                gpio_wake.set_pin_value(pin_value::HIGH); // turn off WAKE function
                return true;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }

        gpio_wake.set_pin_value(pin_value::HIGH); // turn off WAKE function
        return false;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /*!  @brief Initializer for post spi init
     *   @param sensor_id Optional unique ID for the sensor set
     *   @returns True if chip identified and initialized
     */
    bool BNO08x::_init(int32_t sensor_id)
    {
        int status;

        gpio_wake.set_pin_value(pin_value::HIGH); // select SPI as the active interface (latched at reset)

        spiPortSpeed = spiPort->get_spiSpeed();

        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::_init() call.....");

        hardwareReset();

        _sensor_value = &sensorValue;

        // Open SH2 interface (also registers non-sensor event handler.)
        auto hal_callback_handler = [](void *cookie, sh2_AsyncEvent_t *pEvent) -> void
        {
            static_cast<BNO08x *>(cookie)->hal_callback(cookie, pEvent);
        };

        status = sh2_open(&_HAL, hal_callback_handler, this);       // this sends to the BNO086, via halspi_write

        if (status != SH2_OK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::_init()  !!!!!!  sh2_open(..) FAIL !!! status = %d  0x%X", status, status);
            return false;
        }

        // Check connection partially by getting the product id's
        bool wakeup_OK = wake_up();
        memset(&prodIds, 0, sizeof(prodIds));
        status = sh2_getProdIds(&prodIds);
        if (status != SH2_OK)
        { // try again....
            drain_spi();
            wakeup_OK = wake_up();
            drain_spi();

            memset(&prodIds, 0, sizeof(prodIds));
            status = sh2_getProdIds(&prodIds);
        }
        if (status != SH2_OK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::_init()   !!!!!!  sh2_getProdIds(..) FAIL !!! status = %d  0x%X   wakeup_OK=%d", status, status, wakeup_OK);
            return false;
        }

        //-------------- Register sensor listener -----------
        auto sensor_callback_handler = [](void *cookie, sh2_SensorEvent_t *pEvent) -> void
        {
            static_cast<BNO08x *>(cookie)->sensorHandler(cookie, pEvent);
        };

        sh2_setSensorCallback(sensor_callback_handler, (void *)this);

        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::_init(%d) ok.", sensor_id);
        return true;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /**
     * @brief Check if a reset has occured
     *
     * @return true: a reset has occured false: no reset has occoured
     */
    bool BNO08x::wasReset(void)
    {
        bool x = _reset_occurred;
        _reset_occurred = false;

        return x;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /**
     * @brief Fill the given sensor value object with a new report
     *
     * @param value Pointer to an sh2_SensorValue_t struct to fil
     * @return true: The report object was filled with a new report
     * @return false: No new report available to fill
     */
    bool BNO08x::getSensorEvent()
    {
        _sensor_value = &sensorValue;

        _sensor_value->timestamp = 0;

        sh2_service();

        if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV)
        {
            // no new events
            return false;
        }

        return true;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /**
     * @brief Enable the given report type
     *
     * @param sensorId The report ID to enable
     * @param interval_us The update interval for reports to be generated, in
     * microseconds
     * @param sensorSpecific config settings specific to sensor/reportID.
     * (e.g. enabling/disabling possible activities in personal activity classifier)
     * @return true: success false: failure
     */
    bool BNO08x::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us, uint32_t sensorSpecific)
    {
        sh2_SensorConfig_t config;

        // These sensor options are disabled or not used in most cases
        config.changeSensitivityEnabled = false;
        config.wakeupEnabled = false;
        config.changeSensitivityRelative = false;
        config.alwaysOnEnabled = false;
        config.changeSensitivity = 0;
        config.batchInterval_us = 0;
        config.sensorSpecific = sensorSpecific;

        config.reportInterval_us = interval_us;

        // if (_int_pin != -1)
        //     {
        //         if (!hal_wait_for_int()) { return false; }
        //     }

        int status = sh2_setSensorConfig(sensorId, &config);

        if (status != SH2_OK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::enableReport(%d, %d %d) call to sh2_setSensorConfig(..) FAIL !!!! status=%d 0x%X", sensorId, interval_us, sensorSpecific,
                         status, status);
            return false;
        }

        return true;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    static void delay(uint32_t ms)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(ms));
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    void BNO08x::hal_hardwareReset(void)
    {
        // BNO086 NRST (reset) pin is active LOW

        elapsedMillis elap;

        if (_reset_pin != -1)
        {
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::hal_hardwareReset    Hardware reset..........");

            gpio_reset.set_pin_value(pin_value::HIGH);
            delay(30);
            gpio_wake.set_pin_value(pin_value::HIGH);

            gpio_reset.set_pin_value(pin_value::LOW);
            delay(100);
            gpio_reset.set_pin_value(pin_value::HIGH);
            delay(30);

            elap = 0;
            enum pin_value pin__value = pin_value::LOW;
            uint32_t elap_ms = 0;
            while (pin__value == pin_value::LOW && elap < 5000)
            {
                delay(1);
                elap_ms = elap;
                pin__value = gpio_int.get_pin_value();
            }

            gpio_wake.set_pin_value(pin_value::LOW);

            if (pin__value == pin_value::LOW)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::hal_hardwareReset    Hardware reset failed.");
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::hal_hardwareReset    Hardware reset complete.  elap = %d", elap_ms);
            }
        }
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    uint32_t BNO08x::hal_getTimeUs([[maybe_unused]] sh2_Hal_t *self)
    {
        auto ros_time = rclcpp::Clock(RCL_ROS_TIME).now();
        unsigned long long time_us = (unsigned long long)(ros_time.nanoseconds());
        uint32_t t = time_us & 0x7FFFFFFF;
        return t;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    static const char *get_shtp_event_name(enum sh2_ShtpEvent_e evt)
    {
        const char *p = "?";
        switch (evt)
        {
        case SH2_SHTP_TX_DISCARD:
            p = "TX_DISCARD";
            break;
        case SH2_SHTP_SHORT_FRAGMENT:
            p = "SHORT_FRAGMENT";
            break;
        case SH2_SHTP_TOO_LARGE_PAYLOADS:
            p = "TOO_LARGE_PAYLOADS";
            break;
        case SH2_SHTP_BAD_RX_CHAN:
            p = "BAD_RX_CHAN";
            break;
        case SH2_SHTP_BAD_TX_CHAN:
            p = "BAD_TX_CHAN";
            break;
        }
        return p;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    void BNO08x::hal_callback([[maybe_unused]] void *cookie, sh2_AsyncEvent_t *pEvent)
    {
        // If we see a reset, set a flag so that sensors will be reconfigured.
        if (pEvent->eventId == SH2_RESET)
        {
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "hal_callback(..) !Reset!");
            _reset_occurred = true;
            return;
        }
        if (pEvent->eventId == SH2_GET_FEATURE_RESP)
        {
            sh2_SensorId_t sensorId = pEvent->sh2SensorConfigResp.sensorId;
            uint32_t reportInterval_us = pEvent->sh2SensorConfigResp.sensorConfig.reportInterval_us;
            int reportInterval_ms = reportInterval_us / 1000;
            const char *pName = SensorName(sensorId);
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::hal_callback(..)   Sensor %s  #%d    reportInterval_ms = %d \n", pName, sensorId, reportInterval_ms);
        }
        if (pEvent->eventId == SH2_SHTP_EVENT)
        {
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::hal_callback(..)   SHTP Event:  %s", get_shtp_event_name((enum sh2_ShtpEvent_e)pEvent->shtpEvent));
        }
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Handle sensor events.
    void BNO08x::sensorHandler([[maybe_unused]] void *cookie, sh2_SensorEvent_t *event)
    {
        int rc;

        // Serial.println("Got an event!");

        rc = sh2_decodeSensorEvent(_sensor_value, event);
        if (rc != SH2_OK)
        {
            RCLCPP_WARN(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::sensorHandler() - Error decoding sensor event");
            _sensor_value->timestamp = 0;
            return;
        }
        save_sensor_data();
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    void BNO08x::save_sensor_data()
    {
        // RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"),"BNO08x::save_sensor_data()  _sensor_value->sensorId = %d \n", _sensor_value->sensorId);
        switch (_sensor_value->sensorId)
        {
        case SH2_ROTATION_VECTOR:
            rotationVector = _sensor_value->un.rotationVector;
            rotationVector_status = _sensor_value->status;
            rotationVector_timestamp = _sensor_value->timestamp;
            break;
        case SH2_ACCELEROMETER:
            accelerometer = _sensor_value->un.accelerometer;
            accelerometer_status = _sensor_value->status;
            accelerometer_timestamp = _sensor_value->timestamp;
            break;
        case SENSOR_REPORTID_LINEAR_ACCELERATION:
            linearAcceleration = _sensor_value->un.linearAcceleration;
            linearAcceleration_status = _sensor_value->status;
            linearAcceleration_timestamp = _sensor_value->timestamp;
            break;
        case SENSOR_REPORTID_GRAVITY:
            gravity = _sensor_value->un.gravity;
            gravity_status = _sensor_value->status;
            gravity_timestamp = _sensor_value->timestamp;
            break;
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
            gyroscope = _sensor_value->un.gyroscope;
            gyroscope_status = _sensor_value->status;
            gyroscope_timestamp = _sensor_value->timestamp;
            break;
        case SENSOR_REPORTID_MAGNETIC_FIELD:
            magneticField = _sensor_value->un.magneticField;
            magneticField_status = _sensor_value->status;
            magneticField_timestamp = _sensor_value->timestamp;
            break;
        default:
            break;
        }
    }

    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^
    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^

    static const uint32_t Sqrt_2_div_2 = 0x2D413CCD;
    static const uint32_t minus_Sqrt_2_div_2 = 0xD2BEC333;

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    int BNO08x::set_SystemOrientation(int8_t xyzw[4]) // Should use Down South West --> 0 0 -1 -1
    {
        uint32_t orientation_quaturnion[4]{0, 0, 0, 0};
        for (int i = 0; i < 4; i++)
        {
            if (xyzw[i] == -1)
                orientation_quaturnion[i] = minus_Sqrt_2_div_2;
            if (xyzw[i] == 1)
                orientation_quaturnion[i] = Sqrt_2_div_2;
        }
        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "set_SystemOrientation(..) (wxyz) params=  0x%X  0x%X  0x%X  0x%X \n", orientation_quaturnion[0], orientation_quaturnion[1], orientation_quaturnion[2], orientation_quaturnion[3]);
        int rc = setFrs(SYSTEM_ORIENTATION, orientation_quaturnion, 4);
        return rc;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    int BNO08x::get_SystemOrientation(uint32_t xyzw[4])
    {
        uint32_t orientation_quaturnion[4]{0, 0, 0, 0};
        uint16_t count = 4;
        int rc = getFrs(SYSTEM_ORIENTATION, orientation_quaturnion, &count);
        if (rc != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::get_SystemOrientation(..) !ERROR!  call to getFrs(..) FAIL. rc = %d \n", rc);
            return rc;
        }
        if (count != 4)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::get_SystemOrientation(..) !ERROR!  call to getFrs(..) rc=OK, but Count = %d \n", count);
            return -1;
        }

        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "BNO08x::get_SystemOrientation(..) Quaternion=  0x%X  0x%X  0x%X  0x%X \n", orientation_quaturnion[0], orientation_quaturnion[1], orientation_quaturnion[2], orientation_quaturnion[3]);
        for (int i = 0; i < 4; i++)
        {
            xyzw[i] = orientation_quaturnion[i];
        }
        return rc;
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    int BNO08x::clear_SystemOrientation()
    {
        uint32_t orientation_quaturnion[4]{0, 0, 0, 0};
        int rc = setFrs(SYSTEM_ORIENTATION, orientation_quaturnion, 0); // Zero length means delete the record
        return rc;
    }
    //=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^=-+~^

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /**
     * @brief Reset the device using the Reset pin
     *
     */
    void BNO08x::hardwareReset(void) { hal_hardwareReset(); }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    // Return the sensorID
    uint8_t BNO08x::getSensorEventID()
    {
        _sensor_value = &sensorValue;
        return _sensor_value->sensorId;
    }

    /****************************************
    ***************************************** HAL SPI interface
    *****************************************
    *****************************************/

    //.........................................................................
    int BNO08x::spihal_open([[maybe_unused]] sh2_Hal_t *self)
    {
        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "SPI HAL open");

        // hal_wait_for_int();

        return 0;
    }

    //.........................................................................
    void BNO08x::spihal_close([[maybe_unused]] sh2_Hal_t *self)
    {
        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "SPI HAL close");
    }

    //.........................................................................
    int BNO08x::spihal_read([[maybe_unused]] sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, [[maybe_unused]] uint32_t *t_us)
    {
        // RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"),"SPI HAL read");
        uint16_t packet_size = 0;

        if (!hal_wait_for_int())
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "spihal_read(..)  hal_wait_for_int TIMEOUT.!!");
            return 0;
        }

        memset(pBuffer, 0, len);

        // read the first 4 bytes to get the packet size`
        if (!spi_read(pBuffer, 4, 0x00))
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "spihal_read(..)  spi_read(..) FAIL.!!");
            return 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "spihal_read(..) %02X %02X %02X %02X", pBuffer[0], pBuffer[1], pBuffer[2], pBuffer[3]);

        // Determine amount to read
        packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;

        // Unset the "continue" bit
        packet_size &= ~0x8000;

        if (packet_size > len)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "spihal_read(..) packet_size[%d] > len[%d] !!", packet_size, len);
            return 0;
        }

        if (!hal_wait_for_int())
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "spihal_read(..)  hal_wait_for_int TIMEOUT.2.!!");
            return 0;
        }

        if (!spi_read(pBuffer, packet_size, 0x00))
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "spihal_read(..)  spi_read(..).2  FAIL.!!");
            return 0;
        }

        return packet_size;
    }

    //.........................................................................
    int BNO08x::spihal_write([[maybe_unused]] sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
    {
        if (!hal_wait_for_int())
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "spihal_write(..)  hal_wait_for_int TIMEOUT.2.!!");
            return 0;
        }

        spi_write(pBuffer, len);

        return len;
    }

    /****************************************
    ***************************************** SPI WRITE/READ Functions
    *****************************************
    *****************************************/

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /*!
     *    @brief  Read from SPI into a buffer from the SPI device, with transaction management.
     *    @param  buffer Pointer to buffer of data to read into
     *    @param  len Number of bytes from buffer to read.
     *    @param  sendvalue The 8-bits of data to write when doing the data read,
     * defaults to 0xFF
     *    @return Always returns true because there's no way to test success of SPI
     * writes
     */
    bool BNO08x::spi_read(uint8_t *buffer, size_t len, uint8_t sendvalue)
    {
        memset(buffer, sendvalue, len); // clear out existing buffer

        int rc = BNO08x::spiPort->Read(buffer, len);

        return (rc >= 0);
    }

    //-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*-=+~*
    /*!
     *    @brief  Write a buffer or two to the SPI device, with transaction management.
     *    @param  buffer Pointer to buffer of data to write
     *    @param  len Number of bytes from buffer to write
     *    @param  prefix_buffer Pointer to optional array of data to write before buffer.
     *    @param  prefix_len Number of bytes from prefix buffer to write
     *    @return Always returns true because there's no way to test success of SPI writes
     */
    bool BNO08x::spi_write(const uint8_t *buffer, size_t len, [[maybe_unused]] const uint8_t *prefix_buffer, [[maybe_unused]] size_t prefix_len)
    {
        int rc = BNO08x::spiPort->Write(buffer, len);
        return (rc >= 0);
    }

    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
    const char *BNO08x::SensorName(uint8_t sensor_id)
    {
        const char *p = "?";
        switch (sensor_id)
        {
        case SH2_RAW_ACCELEROMETER:
            p = "RAW_ACCELEROMETER";
            break;
        case SH2_ACCELEROMETER:
            p = "ACCELEROMETER";
            break;
        case SH2_LINEAR_ACCELERATION:
            p = "LINEAR_ACCELERATION";
            break;
        case SH2_GRAVITY:
            p = "GRAVITY";
            break;
        case SH2_RAW_GYROSCOPE:
            p = "RAW_GYROSCOPE";
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            p = "GYROSCOPE_CALIBRATED";
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            p = "GYROSCOPE_UNCALIBRATED";
            break;
        case SH2_RAW_MAGNETOMETER:
            p = "RAW_MAGNETOMETER";
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            p = "MAGNETIC_FIELD_CALIBRATED";
            break;
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            p = "MAGNETIC_FIELD_UNCALIBRATED";
            break;
        case SH2_ROTATION_VECTOR:
            p = "ROTATION_VECTOR";
            break;
        case SH2_GAME_ROTATION_VECTOR:
            p = "GAME_ROTATION_VECTOR";
            break;
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            p = "GEOMAGNETIC_ROTATION_VECTOR";
            break;
        case SH2_PRESSURE:
            p = "PRESSURE";
            break;
        case SH2_AMBIENT_LIGHT:
            p = "AMBIENT_LIGHT";
            break;
        case SH2_HUMIDITY:
            p = "HUMIDITY";
            break;
        case SH2_PROXIMITY:
            p = "PROXIMITY";
            break;
        case SH2_TEMPERATURE:
            p = "TEMPERATURE";
            break;
        case SH2_RESERVED:
            p = "RESERVED";
            break;
        case SH2_TAP_DETECTOR:
            p = "TAP_DETECTOR";
            break;
        case SH2_STEP_DETECTOR:
            p = "STEP_DETECTOR";
            break;
        case SH2_STEP_COUNTER:
            p = "STEP_COUNTER";
            break;
        case SH2_SIGNIFICANT_MOTION:
            p = "SIGNIFICANT_MOTION";
            break;
        case SH2_STABILITY_CLASSIFIER:
            p = "STABILITY_CLASSIFIER";
            break;
        case SH2_SHAKE_DETECTOR:
            p = "SHAKE_DETECTOR";
            break;
        case SH2_FLIP_DETECTOR:
            p = "FLIP_DETECTOR";
            break;
        case SH2_PICKUP_DETECTOR:
            p = "PICKUP_DETECTOR";
            break;
        case SH2_STABILITY_DETECTOR:
            p = "STABILITY_DETECTOR";
            break;
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
            p = "PERSONAL_ACTIVITY_CLASSIFIER";
            break;
        case SH2_SLEEP_DETECTOR:
            p = "SLEEP_DETECTOR";
            break;
        case SH2_TILT_DETECTOR:
            p = "TILT_DETECTOR";
            break;
        case SH2_POCKET_DETECTOR:
            p = "POCKET_DETECTOR";
            break;
        case SH2_CIRCLE_DETECTOR:
            p = "CIRCLE_DETECTOR";
            break;
        case SH2_HEART_RATE_MONITOR:
            p = "HEART_RATE_MONITOR";
            break;
        case SH2_ARVR_STABILIZED_RV:
            p = "ARVR_STABILIZED_RV";
            break;
        case SH2_ARVR_STABILIZED_GRV:
            p = "ARVR_STABILIZED_GRV";
            break;
        case SH2_GYRO_INTEGRATED_RV:
            p = "GYRO_INTEGRATED_RV";
            break;
        case SH2_IZRO_MOTION_REQUEST:
            p = "IZRO_MOTION_REQUEST";
            break;
        }
        return p;
    }

    const char *BNO08x::get_SensorName(sh2_SensorId_t sensor_id) { return SensorName(sensor_id); }
    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+

} // end of namespace bno086_hardware_interface