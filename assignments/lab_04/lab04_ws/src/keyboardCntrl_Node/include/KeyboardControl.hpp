#ifndef KEYBORDECTRL 
#define KEYBORDECTRL

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <termios.h>

class KeyboardControlNode : public rclcpp::Node
{
private:
    
    void timerCallback();

    //Pub
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    struct termios old_termios_;

public:
    KeyboardControlNode(/* args */);
    ~KeyboardControlNode();
};


#endif