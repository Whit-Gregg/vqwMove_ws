/*!
 *  @file PWMServoDriver.h
 *
 *  This is a library for our Adafruit 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit 16-channel PWM & Servo
 * driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These driver use I2C to communicate, 2 pins are required to interface.
 *  For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */
#ifndef _PWMServoDriver_H
#define _PWMServoDriver_H

#include <cam_pan_tilt_hardware_interface/i2c_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>


namespace cam_pan_tilt_hardware_interface
{

// REGISTER ADDRESSES
#define PCA9685_MODE1      0x00 /**< Mode Register 1 */
#define PCA9685_MODE2      0x01 /**< Mode Register 2 */
#define PCA9685_SUBADR1    0x02 /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2    0x03 /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3    0x04 /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L  0x06 /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H  0x07 /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L  0xFA /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H  0xFB /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE     0xFE /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE     0xFF /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL  0x01 /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3    0x02 /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2    0x04 /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1    0x08 /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP   0x10 /**< Low power mode. Oscillator off */
#define MODE1_AI      0x20 /**< Auto-Increment enabled */
#define MODE1_EXTCLK  0x40 /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV  0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH     0x08 /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT   0x10 /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40 /**< Default PCA9685 I2C Slave Address */
// #define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */
#define FREQUENCY_OSCILLATOR 26800000 /**< Int. osc. frequency as tested with oscilloscope */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN   150        // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX   600        // This is the 'maximum' pulse length count (out of 4096)
#define USMIN      600        // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX      2400       // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50         // Analog servos run at ~50 Hz updates

    /*!
     *  @brief  Class that stores state and functions for interacting with PCA9685 PWM chip
     */
    class PWMServoDriver
    {
      public:
        PWMServoDriver();
        PWMServoDriver(const std::string i2c_device_name, const uint8_t i2c_address);
        PWMServoDriver(I2CController::SharedPtr i2c_ctl);
        bool     begin();
        void     close();
        void     reset();
        void     sleep();
        void     wakeup();
        void     setExtClk(uint8_t prescale);
        void     setPWMFreq(float freq);
        void     setOutputMode(bool totempole);
        uint16_t getPWM(uint8_t num, bool off = false);
        uint8_t  setPWM(uint8_t num, uint16_t on, uint16_t off);
        uint8_t  setPWM(uint8_t num, float pulse_width_ms);
        void     setPin(uint8_t num, uint16_t val, bool invert = false);
        uint8_t  readPrescale(void);
        // void     writeMicroseconds(uint8_t num, double Microseconds);

        void     setOscillatorFrequency(uint32_t freq);
        uint32_t getOscillatorFrequency(void);

        using SharedPtr = std::shared_ptr<PWMServoDriver>;

      private:
        uint8_t                  _i2caddr = PCA9685_I2C_ADDRESS;
        I2CController::SharedPtr _i2c_ctl;
        uint8_t                  prescale;
        uint32_t                 _oscillator_freq = 0;
        float                    pwm_period_ms    = 1000.0f / SERVO_FREQ;
        void                     delay(uint32_t delay_ms) { std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms)); }
        void                     write8(uint8_t reg, uint8_t data);
        void                     writeN(uint8_t reg, uint8_t *data, int len);
        uint8_t                  read8(uint8_t reg);
        void                     readN(uint8_t reg, uint8_t *data, int len);
        int                      prescale_value  = -1;
        long                     count_of_setPWM = 0;

    };       // class PWMServoDriver

}       // namespace cam_pan_tilt_hardware_interface
#endif
