#include "rclcpp/rclcpp.hpp"
#include "class_loader/class_loader.hpp"
#include "hardware_interface/sensor_interface.hpp"

class bno086_test_class_loader
{
public:
    void test1()
    {
        std::string library_path = "libbno086_hardware_interface,so";
        class_loader::ClassLoader class_loader(library_path, false);

        std::vector<std::string> classes;

        try
        {

            classes = class_loader.getAvailableClasses<hardware_interface::SensorInterface>();
        }
        catch (const class_loader::CreateClassException &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bno086_hardware_interface"), "bno086_test_class_loader--> CreateClassException: %s", ex.what());
            return;
        }

        for (auto &class_name : classes)
        {
            RCLCPP_INFO(rclcpp::get_logger("bno086_hardware_interface"), "bno086_test_class_loader--> class_name: %s", class_name.c_str());
        }
    }
}; // end of:  class bno086_test_class_loader

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto test_bno = std::make_shared<bno086_test_class_loader>();

    test_bno->test1();

    rclcpp::shutdown();
    return 0;
}