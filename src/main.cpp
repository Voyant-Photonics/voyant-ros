#include "voyant-ros/sensor_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoyantSensorDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
