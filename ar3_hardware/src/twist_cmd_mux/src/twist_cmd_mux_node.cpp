#include "twist_cmd_mux/twist_cmd_mux.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<twist_cmd_mux::MessageMux>());
  rclcpp::shutdown();
  return 0;
}