#include <rclcpp/rclcpp.hpp>
#include "twist_cmd_mux/twist_cmd_mux.hpp"
#include <moveit_servo/servo_parameters.h>
#include <string>


using std::placeholders::_1;
using namespace std::chrono_literals;


namespace twist_cmd_mux
{
    
MessageMux::MessageMux()
: Node("twist_cmd_mux")
{
    // ROS2 way, should always declare parameters when you can
    this->declare_parameter<std::string>("xbox_cmd_topic" , "xbox/delta_twist_cmds");
    this->declare_parameter<std::string>("wacom_cmd_topic" , "wacom/delta_twist_cmds");
    this->declare_parameter<std::string>("servo_server_topic" , "servo_server/delta_twist_cmds");
    this->declare_parameter<double>("queue_size" , 10.0);

    // get param values from config passed from launch
    this->initialize_parameters();
    
    // Setup pub / sub
    xbox_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        xbox_cmd, queue_size, std::bind(&MessageMux::xboxCB, this, _1));

    wacom_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        wacom_cmd, queue_size, std::bind(&MessageMux::wacomCB, this, _1));

    servo_server_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(servo_server_cmd, queue_size);

    //publish at 100 Hz -- using timers as hack to control frequency
    // timer_ = this->create_wall_timer(16ms, std::bind(&MessageMux::combine_message, this));
}

MessageMux::MessageMux(const rclcpp::NodeOptions& options)
: Node("twist_cmd_mux", options)
{
    // ROS2 way, should always declare parameters when you can
    this->declare_parameter<std::string>("xbox_cmd_topic" , "xbox/delta_twist_cmds");
    this->declare_parameter<std::string>("wacom_cmd_topic" , "wacom/delta_twist_cmds");
    this->declare_parameter<std::string>("servo_server_topic" , "servo_server/delta_twist_cmds");
    this->declare_parameter<double>("queue_size" , 10.0);

    // get param values from config passed from launch
    this->initialize_parameters();
    
    // Setup pub / sub
    xbox_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        xbox_cmd, queue_size, std::bind(&MessageMux::xboxCB, this, _1));

    wacom_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        wacom_cmd, queue_size, std::bind(&MessageMux::wacomCB, this, _1));

    servo_server_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(servo_server_cmd, queue_size);

    //publish at 60 Hz -- using timers as hack to control frequency
    timer_ = this->create_wall_timer(16ms, std::bind(&MessageMux::combine_message, this));
}

void MessageMux::initialize_parameters() 
{
    // returns a rclcpp::Parameter object
    this->get_parameter("xbox_cmd_topic" , xbox_cmd_topic_);
    this->get_parameter("wacom_cmd_topic" , wacom_cmd_topic_);
    this->get_parameter("servo_server_topic" , servo_server_topic_);
    this->get_parameter("queue_size" , queue_size_);

    // helper functions to return param as correct type
    xbox_cmd = xbox_cmd_topic_.value_to_string();
    wacom_cmd = wacom_cmd_topic_.value_to_string();
    servo_server_cmd = servo_server_topic_.value_to_string();
    queue_size = queue_size_.as_double();

    // Debug prints
    RCLCPP_INFO(this->get_logger(),"xbox_cmd_topic: %s",xbox_cmd_topic_.as_string());
    RCLCPP_INFO(this->get_logger(),"wacom_cmd_topic: %s",wacom_cmd);
    RCLCPP_INFO(this->get_logger(),"servo_server_topic: %s",servo_server_cmd);
    RCLCPP_INFO(this->get_logger(),"queue_size: %f",queue_size);
}

void MessageMux::xboxCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    xbox_msg_in.header = msg->header;
    xbox_msg_in.twist = msg->twist;
    // RCLCPP_INFO(this->get_logger(),"xboxCB");
    MessageMux::combine_message();
}

void MessageMux::wacomCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    wacom_msg_in.header = msg->header;
    wacom_msg_in.twist = msg->twist;
    MessageMux::combine_message();
    // RCLCPP_INFO(this->get_logger(),"wacomCB");
    // Debug print
    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "Linear: X: %f Y: %f Z: %f\n",
    //     wacom_msg_in.twist.linear.x,
    //     wacom_msg_in.twist.linear.y,
    //     wacom_msg_in.twist.linear.z
    // );
}

void MessageMux::combine_message()
{
    auto servo_cmd_out = geometry_msgs::msg::TwistStamped();

    // Grab non-zero command values
    // Twist linear
    if(abs(xbox_msg_in.twist.linear.x) > abs(wacom_msg_in.twist.linear.x))
        servo_cmd_out.twist.linear.x = xbox_msg_in.twist.linear.x;
    else
        servo_cmd_out.twist.linear.x = wacom_msg_in.twist.linear.x;

    if(abs(xbox_msg_in.twist.linear.y) > abs(wacom_msg_in.twist.linear.y))
        servo_cmd_out.twist.linear.y = xbox_msg_in.twist.linear.y;
    else
        servo_cmd_out.twist.linear.y = wacom_msg_in.twist.linear.y; 

    if(abs(xbox_msg_in.twist.linear.z) > abs(wacom_msg_in.twist.linear.z))
        servo_cmd_out.twist.linear.z = xbox_msg_in.twist.linear.z;
    else
        servo_cmd_out.twist.linear.z = wacom_msg_in.twist.linear.z;

    // Twist angular
    if(abs(xbox_msg_in.twist.angular.x) > abs(wacom_msg_in.twist.angular.x))
        servo_cmd_out.twist.angular.x = xbox_msg_in.twist.angular.x;
    else
        servo_cmd_out.twist.angular.x = wacom_msg_in.twist.angular.x;

    if(abs(xbox_msg_in.twist.angular.y) > abs(wacom_msg_in.twist.angular.y))
        servo_cmd_out.twist.angular.y = xbox_msg_in.twist.angular.y;
    else
        servo_cmd_out.twist.angular.y = wacom_msg_in.twist.angular.y;
        
    if(abs(xbox_msg_in.twist.angular.z) > abs(wacom_msg_in.twist.angular.z))
        servo_cmd_out.twist.angular.z = xbox_msg_in.twist.angular.z;
    else
        servo_cmd_out.twist.angular.z = wacom_msg_in.twist.angular.z;

    
    servo_cmd_out.header.stamp = this->now();
    servo_cmd_out.header.frame_id = xbox_msg_in.header.frame_id;

    servo_server_pub_->publish(std::move(servo_cmd_out));

    // Debug print
    RCLCPP_INFO(
        this->get_logger(),
        "\nLinear:\n   x: %0.2f\n   y: %0.2f\n   z: %0.2f\nAngular:\n   x: %0.2f\n   y: %0.2f\n   z: %0.2f\n",
        servo_cmd_out.twist.linear.x,
        servo_cmd_out.twist.linear.y,
        servo_cmd_out.twist.linear.z,
        servo_cmd_out.twist.angular.x,
        servo_cmd_out.twist.angular.y,
        servo_cmd_out.twist.angular.z
    );
}


}   // namespace twist_cmd_mux
