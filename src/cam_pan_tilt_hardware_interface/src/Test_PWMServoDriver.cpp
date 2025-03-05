#include <rclcpp/rclcpp.hpp>

#include "cam_pan_tilt_hardware_interface/i2c_controller.hpp"
#include "cam_pan_tilt_hardware_interface/pwm_servo_driver.hpp"

bool shutdown_now = false;

class Test_PWMServoDriver
{
  public:
    // Test_PWMServoDriver() { pwm_driver->begin(); }

    void delay(int delay_ms)
    {
        for (int i = 0; i < delay_ms; i++)
            {
                if (shutdown_now) return;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
    }

    void set_pwm(int pwm_index, float pwm_value) { pwm_driver->setPWM(pwm_index, pwm_value); }

    std::shared_ptr<cam_pan_tilt_hardware_interface::I2CController>  i2c_controller;
    std::shared_ptr<cam_pan_tilt_hardware_interface::PWMServoDriver> pwm_driver;

    void Test1()
    {
        i2c_controller = std::make_shared<cam_pan_tilt_hardware_interface::I2CController>("/dev/i2c-1", 0x40);
        int rc1        = i2c_controller->open_port();
        if (rc1 < 0)
            {
                printf("Failed to open i2c device: %s   errno=%d  %s\n", i2c_controller->get_device_name().c_str(), errno, strerror(errno));
                return;
            }
        pwm_driver = std::make_shared<cam_pan_tilt_hardware_interface::PWMServoDriver>(i2c_controller);
        pwm_driver->setOscillatorFrequency(26800000);
        bool ok = pwm_driver->begin();
        if (!ok)
            {
                printf("Failed to initialize PWM driver\n");
                return;
            }

        float pulse_width_ms = 1.0;

        for (int i = 0; i < 10; i++)
            {
                pwm_driver->setPWM(0, pulse_width_ms);
                delay(1000 * 30);
                pulse_width_ms += 0.1;
                if (shutdown_now) return;
            }
        // pwm_driver->setPWM(0, 0.5);
        // delay(1000 * 60 * 3);
        // if (shutdown_now) return;

        // pwm_driver->setPWM(0, 0.5);
        // delay(1000 * 60 * 3);
        // if (shutdown_now) return;

        // pwm_driver->writeMicroseconds(0, 1.0);
        // delay(1000 * 60 * 10);
        // if (shutdown_now) return;
    }
};

void sigintHandler([[maybe_unused]] int sig) { shutdown_now = true; }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);

    Test_PWMServoDriver testPWM;

    testPWM.Test1();

    // // // rclcpp::init(argc, argv);
    // // // auto node = std::make_shared<VqwServoDriverComponent>();
    // // // rclcpp::spin(node);
    // // // rclcpp::shutdown();
    return 0;
}