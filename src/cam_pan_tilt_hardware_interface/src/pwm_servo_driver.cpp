/*!
 *  @file PWMServoDriver.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "cam_pan_tilt_hardware_interface/pwm_servo_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <string.h>

// #define ENABLE_DEBUG_OUTPUT

namespace cam_pan_tilt_hardware_interface
{

    /*!
     *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
     * TwoWire interface
     *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
     */
    PWMServoDriver::PWMServoDriver(const std::string dev_name, const uint8_t addr) { _i2c_ctl = std::make_shared<I2CController>(dev_name, addr); }

    /*!
     *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2CController to use for communication
     *  @param  i2c  A reference to a 'I2CController' object that we'll use to communicate with
     */
    PWMServoDriver::PWMServoDriver(I2CController::SharedPtr i2c) : _i2c_ctl(i2c)
    {
        //....
    }

    /*!
     *  @brief  Setups the I2C interface and hardware
     *  @return true if successful, otherwise false
     */
    bool PWMServoDriver::begin()
    {
        if (!_i2c_ctl->isOpen())
            {
                int rc = _i2c_ctl->open_port();
                if (rc < 0)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("cam_pan_tilt_hardware_interface"), "Failed to open i2c device: %s   errno=%d  %s",
                                     _i2c_ctl->get_device_name().c_str(), errno, strerror(errno));
                        return false;
                    }
            }

        reset();

        /*
         * In theory the internal oscillator (clock) is 25MHz but it really isn't
         * that precise. You can 'calibrate' this by tweaking this number until
         * you get the PWM update frequency you're expecting!
         * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
         * is used for calculating things like writeMicroseconds()
         * Analog servos run at ~50 Hz updates, It is importaint to use an
         * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
         * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
         *    the I2C PCA9685 chip you are setting the value for.
         * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
         *    expected value (50Hz for most ESCs)
         * Setting the value here is specific to each individual I2C PCA9685 chip and
         * affects the calculations for the PWM update frequency.
         * Failure to correctly set the int.osc value will cause unexpected PWM results
         */
        if (_oscillator_freq == 0) { setOscillatorFrequency(FREQUENCY_OSCILLATOR); }
        setPWMFreq(SERVO_FREQ);       // Analog servos run at ~50 Hz updates
        setOutputMode(true);          // totem pole!

        return true;
    }

    void PWMServoDriver::close()
    {
        if (_i2c_ctl) { 
            reset();
            _i2c_ctl->close_port();
             }
    }

    /*!
     *  @brief  Sends a reset command to the PCA9685 chip over I2C
     */
    void PWMServoDriver::reset()
    {
        write8(PCA9685_MODE1, MODE1_RESTART);
        delay(10);
    }

    /*!
     *  @brief  Puts board into sleep mode
     */
    void PWMServoDriver::sleep()
    {
        uint8_t awake = read8(PCA9685_MODE1);
        uint8_t sleep = awake | MODE1_SLEEP;       // set sleep bit high
        write8(PCA9685_MODE1, sleep);
        delay(5);       // wait until cycle ends for sleep to be active
    }

    /*!
     *  @brief  Wakes board from sleep
     */
    void PWMServoDriver::wakeup()
    {
        uint8_t sleep  = read8(PCA9685_MODE1);
        uint8_t wakeup = sleep & ~MODE1_SLEEP;       // set sleep bit low
        write8(PCA9685_MODE1, wakeup);
    }

    /*!
     *  @brief  Sets EXTCLK pin to use the external clock
     *  @param  prescale
     *          Configures the prescale value to be used by the external clock
     */
    void PWMServoDriver::setExtClk(uint8_t prescale)
    {
        uint8_t oldmode = read8(PCA9685_MODE1);
        uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;       // sleep
        write8(PCA9685_MODE1, newmode);                                   // go to sleep, turn off internal oscillator

        // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
        // use the external clock.
        write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

        write8(PCA9685_PRESCALE, prescale);       // set the prescaler

        delay(5);
        // clear the SLEEP bit to start
        write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
    }

    /*!
     *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
     *  @param  freq Floating point frequency that we will attempt to match
     */
    void PWMServoDriver::setPWMFreq(float freq)
    {
        // Range output modulation frequency is dependant on oscillator
        if (freq < 1) freq = 1;
        if (freq > 3500) freq = 3500;       // Datasheet limit is 3052=50MHz/(4*4096)

        pwm_period_ms = 1000.0f / freq;

        float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
        if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
        if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;
        uint8_t prescale = (uint8_t)prescaleval;
        prescale_value   = prescale;

        RCLCPP_INFO(rclcpp::get_logger("PWMServoDriver"), "setPWMFreq(%.1f) pre-scale to: %d ", freq, prescale_value);

        uint8_t oldmode = read8(PCA9685_MODE1);
        uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;       // sleep
        write8(PCA9685_MODE1, newmode);                                   // go to sleep
        write8(PCA9685_PRESCALE, prescale);                               // set the prescaler
        write8(PCA9685_MODE1, oldmode);
        delay(5);
        // This sets the MODE1 register to turn on auto increment, and Totempole(push/pull)
        write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI | MODE2_OUTDRV);
    }

    /*!
     *  @brief  Sets the output mode of the PCA9685 to either
     *  open drain or push pull / totempole.
     *  Warning: LEDs with integrated zener diodes should
     *  only be driven in open drain mode.
     *  @param  totempole Totempole if true, open drain if false.
     */
    void PWMServoDriver::setOutputMode(bool totempole)
    {
        uint8_t oldmode = read8(PCA9685_MODE2);
        uint8_t newmode;
        if (totempole) { newmode = oldmode | MODE2_OUTDRV; }
        else { newmode = oldmode & ~MODE2_OUTDRV; }
        write8(PCA9685_MODE2, newmode);
    }

    /*!
     *  @brief  Reads set Prescale from PCA9685
     *  @return prescale value
     */
    uint8_t PWMServoDriver::readPrescale(void)
    {
        if (prescale_value < 0) prescale_value = read8(PCA9685_PRESCALE);
        return prescale_value;
    }

    /*!
     *  @brief  Gets the PWM output of one of the PCA9685 pins
     *  @param  num One of the PWM output pins, from 0 to 15
     *  @param  off If true, returns PWM OFF value, otherwise PWM ON
     *  @return requested PWM output value
     */
    uint16_t PWMServoDriver::getPWM(uint8_t num, bool off)
    {
        uint8_t buffer[2] = {uint8_t(PCA9685_LED0_ON_L + 4 * num), 0};
        if (off) buffer[0] += 2;
        uint8_t reg = (uint8_t)(PCA9685_LED0_ON_L + 4 * num);
        _i2c_ctl->read_reg(reg, buffer, 2);
        return uint16_t(buffer[0]) | (uint16_t(buffer[1]) << 8);
    }

    /*!
     *  @brief  Sets the PWM output of one of the PCA9685 pins
     *  @param  num One of the PWM output pins, from 0 to 15
     *  @param  pulse_width_ms The number of milliseconds to turn the PWM output ON
     *  @return 0 if successful, otherwise 1
     */

    uint8_t PWMServoDriver::setPWM(uint8_t num, float pulse_width_ms)
    {
        if (std::isnan(pulse_width_ms)) { return 1; }
        // Clamp value between 0 and 4095 inclusive.
        int val = (uint16_t)(pulse_width_ms * 4096 / pwm_period_ms);
        // uint16_t val = (uint16_t)(pulse_width_ms * 4096 / 20);
        if (val > 4095) val = 4095;
        if (val < 0) val = 0;

        if ((count_of_setPWM < 5)||(count_of_setPWM % 100) == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("PWMServoDriver"), "setPWM(%2d, %.3f)  val=%d", num, pulse_width_ms, val);
            }
        count_of_setPWM++;
        setPin(num, val);
        return 0;
    }

    /*!
     *  @brief  Sets the PWM output of one of the PCA9685 pins
     *  @param  num One of the PWM output pins, from 0 to 15
     *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
     *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
     *  @return 0 if successful, otherwise 1
     */
    uint8_t PWMServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off)
    {
        uint8_t reg = PCA9685_LED0_ON_L + (4 * num);
        uint8_t buffer[5];
        buffer[0] = on;
        buffer[1] = on >> 8;
        buffer[2] = off;
        buffer[3] = off >> 8;

        //RCLCPP_INFO(rclcpp::get_logger("PWMServoDriver"), "setPWM(%2d, %4d, %4d)", num, on, off);

        if (_i2c_ctl->write_reg(reg, buffer, 4)) { return 0; }
        return 1;
    }

    /*!
     *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
     * on/off tick placement and properly handles a zero value as completely off and
     * 4095 as completely on.  Optional invert parameter supports inverting the
     * pulse for sinking to ground.
     *   @param  num One of the PWM output pins, from 0 to 15
     *   @param  val The number of ticks out of 4096 to be active, should be a value from 0 to 4095 inclusive.
     *   @param  invert If true, inverts the output, defaults to 'false'
     */
    void PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert)
    {
        // Clamp value between 0 and 4095 inclusive.
        val = std::min(val, (uint16_t)4095);
        if (invert)
            {
                if (val == 0)
                    {
                        // Special value for signal fully on.
                        setPWM(num, 4096, 0);
                    }
                else if (val == 4095)
                    {
                        // Special value for signal fully off.
                        setPWM(num, 0, 4096);
                    }
                else { setPWM(num, 0, 4095 - val); }
            }
        else
            {
                if (val == 4095)
                    {
                        // Special value for signal fully on.
                        setPWM(num, 4096, 0);
                    }
                else if (val == 0)
                    {
                        // Special value for signal fully off.
                        setPWM(num, 0, 4096);
                    }
                else { setPWM(num, 0, val); }
            }
    }

    /*!
     *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input microseconds, output is not precise
     *  @param  num One of the PWM output pins, from 0 to 15
     *  @param  Microseconds The number of Microseconds to turn the PWM output ON
     */
    // void PWMServoDriver::writeMicroseconds(uint8_t num, double Microseconds)
    // {
    //     double pulse = Microseconds;
    //     double pulselength;
    //     pulselength = 1000000;       // 1,000,000 us per second

    //     // Read prescale
    //     uint16_t prescale = readPrescale();

    //     // Calculate the pulse for PWM based on Equation 1 from the datasheet section 7.3.5
    //     prescale += 1;
    //     pulselength *= prescale;
    //     pulselength /= _oscillator_freq;
    //     pulse /= pulselength;

    //     setPWM(num, 0, pulse);
    // }

    /*!
     *  @brief  Getter for the internally tracked oscillator used for freq
     * calculations
     *  @returns The frequency the PCA9685 thinks it is running at (it cannot introspect)
     */
    uint32_t PWMServoDriver::getOscillatorFrequency(void) { return _oscillator_freq; }

    /*!
     *  @brief Setter for the internally tracked oscillator used for freq calculations
     *  @param freq The frequency the PCA9685 should use for frequency calculations
     */
    void PWMServoDriver::setOscillatorFrequency(uint32_t freq)
    {
        _oscillator_freq = freq;
        RCLCPP_INFO(rclcpp::get_logger("PWMServoDriver"), "setOscillatorFrequency(%d)", freq);
    }

    /******************* Low level I2C interface ****************************/
    uint8_t PWMServoDriver::read8(uint8_t reg)
    {
        uint8_t buffer[2] = {0, 0};
        _i2c_ctl->read_reg(reg, buffer, 1);
        return buffer[0];
    }

    void PWMServoDriver::write8(uint8_t reg, uint8_t d)
    {
        uint8_t buffer[2] = {d, 0};
        _i2c_ctl->write_reg(reg, buffer, 1);
    }

    void PWMServoDriver::readN(uint8_t reg, uint8_t *data, int len) { _i2c_ctl->read_reg(reg, data, len); }

    void PWMServoDriver::writeN(uint8_t reg, uint8_t *data, int len) { _i2c_ctl->write_reg(reg, data, len); }

}       // namespace cam_pan_tilt_hardware_interface