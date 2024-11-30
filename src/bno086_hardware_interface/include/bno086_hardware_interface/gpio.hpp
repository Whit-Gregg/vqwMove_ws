#ifndef GPIO_HPP
#define GPIO_HPP

// use gpiod library instead of this class
// int chip = gpiod.Chip("gpiochip4");

#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "elapsedMillis.h"

#define use_LINUX_GPIO 1
// #define use_GPIOD 1
// #define use_GPIO_SYSFS 1

#ifdef use_GPIO_SYSFS
// high priority or an error occured in SW
#define POLL_GPIO POLLPRI | POLLERR
#define TIMEOUT 60
#define NSAMPLES 1000
#define EXPORT_PATH "/sys/class/gpio/export"
#define UNEXPORT_PATH "/sys/class/gpio/unexport"
#define SW_VAL_PATH_FORMAT "/sys/class/gpio/gpio%d/value"
#define SW_INT_PATH_FORMAT "/sys/class/gpio/gpio%d/edge"
#define SW_DIR_PATH_FORMAT "/sys/class/gpio/gpio%d/direction"
#endif

#ifdef use_GPIOD
#include <gpiod.h>
#endif

#ifdef use_LINUX_GPIO
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <string.h>
#endif

namespace bno086_hardware_interface
{

    enum class pin_direction
    {
        UNKNOWN,
        INPUT,
        OUTPUT
    };
    enum class pin_value
    {
        LOW = 0,
        HIGH = 1,
        UNKNOWN = 2
    };

    class gpio
    {
        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    private:
        int gpio_pin_number = -1;
        enum pin_direction direction = pin_direction::UNKNOWN;
#ifdef use_LINUX_GPIO
        int chip_fd;
        int req_fd;
        int avg_millis_in_loop = -1;
        int avg_millis_in_loop_span = 25;
        int wait_until_LOW_count = 0;
        int wait_until_LOW_timeout_count = 0;
        elapsedMillis elap_since_loop_wait_report;
        uint32_t loop_wait_report_interval = 5000;

#endif
#ifdef use_GPIO_SYSFS
        int fd = -1;
        char sw[4]; // Linux GPIO pin number (as a C string)
        char sw_val_path[48];
        char sw_int_path[48];
        char sw_dir_path[48];
        char value;
        struct pollfd poll_gpio;
#endif
#ifdef use_GPIOD
        gpiod::chip chip("gpiochip0");
        struct gpiod_line *line = NULL;
#endif
        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    public:
        gpio() {}
        gpio(int gpio_pin_number_, pin_direction direction_)
        {
            gpio_pin_number = gpio_pin_number_;
            direction = direction_;
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        ~gpio()
        {
            Close();
        }
        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        int get_avg_millis_in__wait_until_LOW()
        {
            return avg_millis_in_loop;
        }
        int get_wait_until_LOW_count()
        {
            return wait_until_LOW_count;
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        int Initialize(int gpio_pin_number_, pin_direction direction_)
        {
            gpio_pin_number = gpio_pin_number_;
            direction = direction_;
            return init();
        }
        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        int Initialize()
        {
            if (gpio_pin_number == -1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::Initialize() GPIO pin number is not set");
                return -1;
            }
            if ((gpio_pin_number < 0) || (gpio_pin_number > 40))
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::Initialize() GPIO=%d,  pin number is out of range", gpio_pin_number);
                return -1;
            }
            if (direction == pin_direction::UNKNOWN)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::Initialize() GPIO=%d,  direction is not set", gpio_pin_number);
                return -1;
            }
            return init();
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        void Close()
        {
#ifdef use_LINUX_GPIO
            if (chip_fd >= 0)
            {
                close(chip_fd);
                chip_fd = -1;
            }
            if (req_fd >= 0)
            {
                close(req_fd);
                req_fd = -1;
            }
#endif
#ifdef use_GPIOD
            if (line)
                gpiod_line_release(line);
#endif
#ifdef use_GPIO_SYSFS
            if (fd >= 0)
                close_fd(fd);
            unconfigure_pins();
#endif
        }
        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        pin_value get_pin_value() // if error, returns pin_value::UNKNOWN
        {
            if (direction == pin_direction::OUTPUT)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::get_pin_value() GPIO=%d is not configured as INPUT", gpio_pin_number);
                return pin_value::UNKNOWN;
            }
#ifdef use_LINUX_GPIO
            struct gpio_v2_line_values values;
            values.bits = 0;
            values.mask = 1;
            int rc = ioctl(req_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values);
            if (rc == 0)
            {
                return (values.bits & 1) ? pin_value::HIGH : pin_value::LOW;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::get_pin_value() gpio=%d ioctl() failed with error %d = %s", gpio_pin_number, errno, strerror(errno));
                return pin_value::UNKNOWN;
            }
#endif
#ifdef use_GPIOD
            if (!line)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::get_pin_value() %s  is not open, call Initialize() before this method", sw_val_path);
                return pin_value::UNKNOWN;
            }
            int val = gpiod_line_get_value(line);
            return (val == 0) ? pin_value::LOW : pin_value::HIGH;
#endif
#ifdef use_GPIO_SYSFS

            if (fd <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::get_pin_value() %s  is not open, call Initialize() before this method", sw_val_path);
                return pin_value::UNKNOWN;
            }
            lseek(fd, 0, SEEK_SET);
            read(fd, &value, 1); // read GPIO value
            return (value == '0') ? pin_value::LOW : pin_value::HIGH;
#endif
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        int set_pin_value(pin_value val) // returns 0 if success, -1 if failed
        {
            if (direction == pin_direction::INPUT)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::set_pin_value() gpio=%d GPIO is not configured as OUTPUT", gpio_pin_number);
                return -1;
            }
#ifdef use_LINUX_GPIO
            struct gpio_v2_line_values values;
            values.bits = (val == pin_value::HIGH) ? 1 : 0;
            values.mask = 1;
            int rc = ioctl(req_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values);
            if (rc != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::set_pin_value() gpio=%d ioctl() failed with error %d -%s", gpio_pin_number, errno, strerror(errno));
                return -1;
            }
            return 0;
#endif
#ifdef use_GPIOD
            if (!line)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::set_pin_value() %s  is not open, call Initialize() before this method", sw_val_path);
                return -1;
            }
            int valuee = (val == pin_value::HIGH) ? 1 : 0;
            return gpiod_line_set_value(line, valuee);
#endif
#ifdef use_GPIO_SYSFS
            if (fd <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::set_pin_value() Failed to open %s", sw_val_path);
                return -1;
            }
            value = (val == pin_value::HIGH) ? '1' : '0';
            write(fd, &value, 1);
            return 0;
#endif
        }

        void record_loop_wait(uint32_t wait_time_ms, uint32_t timeout_ms)
        {
            wait_until_LOW_count++;
            if (wait_time_ms>= timeout_ms)
            {
                wait_until_LOW_timeout_count++;
            }   
            if (avg_millis_in_loop == -1)
            {
                avg_millis_in_loop = (int)wait_time_ms;
            }

            avg_millis_in_loop = (avg_millis_in_loop * (avg_millis_in_loop_span - 1) + ((int)wait_time_ms)) /  avg_millis_in_loop_span;
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        bool wait_until_LOW(uint32_t timeout_ms) // returns true if value is '0', or false if timeout
        {
#ifdef use_LINUX_GPIO
            if (direction == pin_direction::OUTPUT)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() GPIO=%d is not configured as INPUT", gpio_pin_number);
                return false;
            }
            if (req_fd < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() GPIO = %d is not open, call Initialize() before this method", gpio_pin_number);
                return false;
            }

            if (elap_since_loop_wait_report > loop_wait_report_interval)
            {
                elap_since_loop_wait_report = 0;
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() gpio=%d  count=%d   timeouts=%d,   avg_millis_in_loop=%d", gpio_pin_number, wait_until_LOW_count, wait_until_LOW_timeout_count, avg_millis_in_loop);
            }

            pin_value val = get_pin_value();
            if (val == pin_value::LOW)
            {
                record_loop_wait(0, timeout_ms);
                return true;
            }
            elapsedMillis elap_for_loop;
            struct gpio_v2_line_values values;
            while (true)
            {
                values.mask = 1;
                values.bits = 1;
                int rc = ioctl(req_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values);
                if (rc != 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() gpio=%d ioctl() failed with error %d - %s", gpio_pin_number, errno, strerror(errno));
                    // record_loop_wait(elap_for_loop, timeout_ms);     // don't count if this is an error
                    return false;
                }
                if ((values.bits & 1) == 0)
                {
                    record_loop_wait(elap_for_loop, timeout_ms);
                    return true;
                }

                if (elap_for_loop > timeout_ms) {
                    record_loop_wait(elap_for_loop, timeout_ms);
                    return false;
                }

                int millis_to_sleep = timeout_ms / 10;
                millis_to_sleep = std::clamp(millis_to_sleep, 1, 5);
                rclcpp::sleep_for(std::chrono::milliseconds(millis_to_sleep));
            }
#endif
#ifdef use_GPIOD
            if (!line)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() %s  is not open, call Initialize() before this method", sw_val_path);
                return false;
            }
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += timeout_ms / 1000;
            ts.tv_nsec += (timeout_ms % 1000) * 1000000;
            return (gpiod_line_event_wait(line, &ts) == 0);
#endif
#ifdef use_GPIO_SYSFS
            if (fd <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() %s  is not open, call Initialize)() before this method", sw_val_path);
                return false;
            }
            int poll_ret = 0;
            // file descriptor from SW is being polled
            poll_gpio.fd = fd;
            // poll events in GPIO
            poll_gpio.events = POLL_GPIO;
            poll_gpio.revents = 0;

            value = '1';
            while (value != '0')
            {
                lseek(fd, 0, SEEK_SET);
                read(fd, &value, 1); // read GPIO value
                if (value == '0')
                    return true;

                poll_ret = poll(&poll_gpio, 1, timeout_ms);

                if (poll_ret == 0)
                {
                    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() GPIO=%s !! Timeout !!", sw);
                    return false;
                }
                else
                {
                    if (poll_ret == -1)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::wait_until_LOW() GPIO=%s !! poll Failed !!", sw);
                        return false;
                    }
                    if ((poll_gpio.revents) & (POLL_GPIO))
                    {
                        lseek(fd, 0, SEEK_SET);
                        read(fd, &value, 1); // read GPIO value
                    }
                }
            }
            return (value == '0');
#endif
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        int init() // return 0 if good, -1 if error
        {
#ifdef use_LINUX_GPIO
            req_fd = -1;
            char chipname[32];
            sprintf(chipname, "/dev/gpiochip%d", 4);
            chip_fd = open(chipname, O_RDONLY);
            if (chip_fd < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::init() Failed to open %s", chipname);
                return -1;
            }
            struct gpio_v2_line_config config;
            struct gpio_v2_line_request req;
            memset(&config, 0, sizeof(config));
            memset(&req, 0, sizeof(req));

            config.flags = (direction == pin_direction::INPUT) ? GPIO_V2_LINE_FLAG_INPUT : GPIO_V2_LINE_FLAG_OUTPUT;
            config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
            if (direction == pin_direction::INPUT)
            {
                config.flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING;
            }

            strncpy(req.consumer, "bno086_hardware_interface", sizeof(req.consumer) - 1);
            // req.flags = 0;
            req.fd = -1;
            req.config = config;
            req.num_lines = 1;
            req.offsets[0] = gpio_pin_number;
            req_fd = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
            if (req_fd < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::init() Failed to request GPIO %d", gpio_pin_number);
                return -1;
            }
            if (req.fd < 0) {
                close(chip_fd);
                return -1;
            }
            req_fd = req.fd;
            close(chip_fd);
            return 0;
#endif
#ifdef use_GPIOD
            if (direction == pin_direction::INPUT)
            {
                set_realtime_priority();
            }
            int rc = configure_pins();
            return rc;

#endif
#ifdef use_GPIO_SYSFS
            sprintf(sw, "%d", gpio_pin_number);
            sprintf(sw_val_path, SW_VAL_PATH_FORMAT, gpio_pin_number);
            sprintf(sw_int_path, SW_INT_PATH_FORMAT, gpio_pin_number);
            sprintf(sw_dir_path, SW_DIR_PATH_FORMAT, gpio_pin_number);

            if (direction == pin_direction::INPUT)
            {
                set_realtime_priority();
            }
            configure_pins();

            fd = open(sw_val_path, O_RDONLY);
            if (fd <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::init() Failed to open %s", sw_val_path);
                return -1;
            }
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::init() Opened %s value successfully", sw_val_path);
            if (direction == pin_direction::INPUT)
            {
                read(fd, &value, 1);
            }
            if (direction == pin_direction::OUTPUT)
            {
                value = '0';
                write(fd, &value, 1);
            }
            return 0;
#endif
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        void set_realtime_priority()
        {
            pthread_t this_thread = pthread_self(); // operates in the current running thread
            struct sched_param params;
            int ret;

            // set max prio
            params.sched_priority = sched_get_priority_max(SCHED_FIFO);
            ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);

            if (ret != 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::set_realtime_priority() !! Failed !! errno = %d = %s", errno, strerror(errno));
                // perror("Unsuccessful in setting thread realtime prio\n");
            }
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
#ifdef use_GPIO_SYSFS
        void close_fd(int fd)
        {
            // close file from file descriptor
            if (close(fd) < 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::close_fd() Unable to close file correctly");
            }
        }
#endif
        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        int unconfigure_pins()
        {
#ifdef use_GPIOD
            if (line)
                gpiod_line_release(line);
#endif
#ifdef use_GPIO_SYSFS
            int fd_export;
            // open export file
            if ((fd_export = open(UNEXPORT_PATH, O_WRONLY)) <= 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::unconfigure_pins() Unable to open Un-export file");
                return EXIT_FAILURE;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::unconfigure_pins() Opened Un-export file successfully");
            }

            // unexport SW GPIO
            if (write(fd_export, sw, strlen(sw)) < 0)
            {
                if (errno != EBUSY)
                { // does not end if pin is not already exported
                    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::unconfigure_pins() Unable to Un-export SW GPIO:  %s   errno=%d", sw, errno);
                    close_fd(fd_export);
                    return EXIT_FAILURE;
                }
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::unconfigure_pins() Unable to Un-xport SW GPIO:  %s   error == EBUSY", sw);
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::unconfigure_pins() Un-Exported SW=%s successfully", sw);
            }

            // close export file
            close_fd(fd_export);
            return 0;
#endif
            return 0;
        }

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        int configure_pins()
        {
#ifdef use_GPIOD
            line = chip.get_line(gpio_pin_number);
            if (!line)
            {
                RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Failed to get line %d", gpio_pin_number);
                return -1;
            }
            if (direction == pin_direction::INPUT)
            {
                if (gpiod_line_request_input(line, "bno086_hardware_interface") < 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Failed to request input %d", gpio_pin_number);
                    return -1;
                }
                return 0;
            }
            if (direction == pin_direction::OUTPUT)
            {
                if (gpiod_line_request_output(line, "bno086_hardware_interface", 0) < 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Failed to request output %d", gpio_pin_number);
                    return -1;
                }
                return 0;
            }
#endif
#ifdef use_GPIO_SYSFS
            int fd_export, fd_edge, fd_input;

            /******************* EXPORT *******************/
            // open export file
            if ((fd_export = open(EXPORT_PATH, O_WRONLY)) <= 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to open export file");
                return EXIT_FAILURE;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Opened export file successfully");
            }

            // export SW GPIO
            if (write(fd_export, sw, strlen(sw)) < 0)
            {
                if (errno != EBUSY)
                { // does not end if pin is already exported
                    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to export SW GPIO:  %s   errno=%d", sw, errno);
                    close_fd(fd_export);
                    return EXIT_FAILURE;
                }
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to export SW GPIO:  %s   error == EBUSY", sw);
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Exported SW=%s successfully", sw);
            }

            // close export file
            close_fd(fd_export);

            /****************** DIRECTION ******************/
            // open direction file
            if ((fd_input = open(sw_dir_path, O_WRONLY)) <= 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to open direction file for: %s", sw_dir_path);
                return EXIT_FAILURE;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Opened direction file successfully:  %s", sw_dir_path);
            }

            if (write(fd_input, "in", 2) < 0)
            { // configure as input
                if (errno != EBUSY)
                {
                    RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to change direction to 'IN' for:  %s   errno = %d", sw_dir_path, errno);
                    close_fd(fd_input);
                    return EXIT_FAILURE;
                }
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to change direction to 'IN' for:  %s   error == EBUSY", sw_dir_path);
                perror("Warning: unable to change direction from SW\n");
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Changed direction to 'IN' for: %s  successfully", sw_dir_path);
            }

            close_fd(fd_input); // close direction file

            /******************** EDGE *********************/
            if ((fd_edge = open(sw_int_path, O_RDWR)) <= 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to open edge file for: %s", sw_int_path);
                return EXIT_FAILURE;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Opened edge file successfully:  %s", sw_int_path);
            }

            if (write(fd_edge, "rising", 6) < 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Unable to change edge for %s", sw_int_path);
                close_fd(fd_edge);
                return EXIT_FAILURE;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "gpio::configure_pins() Changed edge successfully:  %s", sw_int_path);
            }

            close_fd(fd_edge);

            return EXIT_SUCCESS;
#endif
            return 0;
        }
    }; // end of class gpio

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

} // end of namespace bno086_hardware_interface

#endif // GPIO_HPP