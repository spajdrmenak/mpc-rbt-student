#include "KeyboardControl.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControlNode>());
    rclcpp::shutdown();
    return 0;
}
