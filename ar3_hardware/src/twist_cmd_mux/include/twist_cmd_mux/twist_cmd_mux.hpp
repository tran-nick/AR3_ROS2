#ifndef TWIST_CMD_MUX_HPP
#define TWIST_CMD_MUX_HPP


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>


namespace twist_cmd_mux
{

class MessageMux : public rclcpp::Node
{
    public:
    // constructor
    MessageMux();
    // constructor overload
    MessageMux(const rclcpp::NodeOptions& options);

    void initialize_parameters();
    
    // subscriber callbacks, handle messages
    void xboxCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void wacomCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    //publisher callback on a timer
    void combine_message();

    // Variables
    std::string xbox_cmd;
    std::string wacom_cmd;
    std::string servo_server_cmd;
    double queue_size;

    // destructor
    ~MessageMux() = default;

    private:

    // publishers and subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr xbox_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wacom_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_server_pub_;
    
    // parameters 
    rclcpp::Parameter xbox_cmd_topic_;
    rclcpp::Parameter wacom_cmd_topic_;
    rclcpp::Parameter servo_server_topic_;
    rclcpp::Parameter queue_size_;
    rclcpp::TimerBase::SharedPtr timer_;

    //messages
    geometry_msgs::msg::TwistStamped xbox_msg_in;
    geometry_msgs::msg::TwistStamped wacom_msg_in;
        
        
};

} // namespace twist_cmd_mux

#endif