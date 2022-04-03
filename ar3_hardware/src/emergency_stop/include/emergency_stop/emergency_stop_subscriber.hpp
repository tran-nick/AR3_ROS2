#ifndef EMERGENCY_STOP_HPP
#define EMERGENCY_STOP_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/u_int32.hpp>

using std::placeholders::_1;

namespace emergency_stop
{

class EmergencyStopSubscriber : public rclcpp::Node
{
  public:
  float range_value;
  int button_pressed;
  
  EmergencyStopSubscriber()
  : Node("emergency_stop_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
        "range", 10,std::bind(&EmergencyStopSubscriber::topic_callback, this, _1));
    button_subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
        "estop_clr", 10,std::bind(&EmergencyStopSubscriber::topic2_callback, this, _1));
  }

    private:
  void topic_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
  {
      RCLCPP_INFO(this->get_logger(),"Range heard: '%f'",msg->range);
      range_value = msg->range;
  }

  void topic2_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),"Estop clear pressed: '%s'",msg->data? "TRUE" : "FALSE");
    button_pressed = msg->data;
  }

  
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr button_subscription_;
};

} // namespace emergency_stop

#endif