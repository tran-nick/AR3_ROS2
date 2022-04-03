#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/u_int32.hpp>

using namespace std::chrono_literals;

namespace emergency_stop{

class EmergencyStop : public rclcpp::Node
{
  public:
  //constructor create node called "emergency_stop"
  EmergencyStop()
  : Node("emergency_stop"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("range", 10);
    button_publisher_ = this->create_publisher<std_msgs::msg::UInt32>("estop_clr", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&EmergencyStop::timer_callback, this));
  }

  void init(void);  // open port to driver
  float sensor_read(void);              // read port to get dist measurement
  int button_read(void);                // read if GPIO button was pressed

  private:
  void timer_callback()
  {
      auto message = sensor_msgs::msg::Range();
      auto button = std_msgs::msg::UInt32();
      message.range = sensor_read();
      button.data = button_read();
      RCLCPP_INFO(this->get_logger(),"Range: %f",message.range);
      RCLCPP_INFO(this->get_logger(),"Button: %d",button.data);
      publisher_->publish(message);
      button_publisher_->publish(button);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr button_publisher_;
  size_t count_;
  int fd, fd1;

};

} // namespace emergency_stop

#endif