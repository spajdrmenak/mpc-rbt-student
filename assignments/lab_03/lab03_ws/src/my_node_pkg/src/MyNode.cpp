// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MyNode : public rclcpp::Node
{
public:

  
  MyNode()
  : Node("MyNode"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    publisher1_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyNode::timer_callback, this));
      
    subscription0_ = create_subscription<std_msgs::msg::Float32>("battery_voltage", 1, std::bind(&MyNode::battCallback, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "MyNode";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
    void battCallback(const std_msgs::msg::Float32::ConstSharedPtr &msg){
	
	auto value = msg->data;
	
	auto percent = (value-32)*10;
	auto resp = std_msgs::msg::Float32();
	resp.data = percent;
	publisher1_->publish(resp);
	
  }
  
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher1_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription0_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
