/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
// const std::string TWIST_TOPIC = "/servo_server/delta_twist_cmds";
const std::string TWIST_TOPIC = "/xbox/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_server/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "ar3_link6";
const std::string BASE_FRAME_ID = "ar3_link0";
double velScale = 1;
int command_type = 1;
std::string* command_frame;

enum command_type
{
  JOINT_JOG = 0,
  JOG_TWIST = 1,
};

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  BACK = 6,
  START = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.servo");

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands
  /* 
  Note: No button debouncing, one press will return multiple values
  axes[D_PAD_X] +1 left / -1 right
  axes[D_PAD_Y] +1 up / -1 down
  buttons[A,B,X,Y] 1.0 when pressed otherwise 0.
  buttons[right/left bumper] 1.0 when pressed otherwise 0
  axes[RIGHT_STICK_X & Y] value between [0,1] proportional to stick tilt. +1 left & up / -1 right & down
  axes[LEFT_STICK_X & Y] value between [0,1] proportional to stick tilt. +1 left & up / -1 right & down
 */
  if(command_type == JOG_TWIST)
  {  
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
    {
      // Map the D_PAD to the proximal joints
      joint->joint_names.push_back("ar3_joint1");
      joint->velocities.push_back(velScale*axes[D_PAD_X]);      
      joint->joint_names.push_back("ar3_joint2");
      joint->velocities.push_back(velScale*axes[D_PAD_Y]);

      // Map the diamond to the distal joints
      joint->joint_names.push_back("ar3_joint4");
      joint->velocities.push_back(velScale * (buttons[B] - buttons[X]));
      joint->joint_names.push_back("ar3_joint3");
      joint->velocities.push_back(velScale * (buttons[Y] - buttons[A]));
      return false;
    }

    // The bread and butter: map buttons to twist commands
    // twist->twist.linear.z = axes[RIGHT_STICK_Y];
    // twist->twist.linear.y = axes[RIGHT_STICK_X];

    // double lin_x_right = -1 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    // double lin_x_left = 1 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    // twist->twist.linear.x = lin_x_right + lin_x_left;

    // twist->twist.angular.y = axes[LEFT_STICK_Y];
    // twist->twist.angular.x = axes[LEFT_STICK_X];

    // double roll_positive = buttons[RIGHT_BUMPER];
    // double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    // twist->twist.angular.z = roll_positive + roll_negative;

    // Modified commands
    twist->twist.linear.y = -1 * velScale * axes[RIGHT_STICK_Y];
    twist->twist.linear.x = velScale * axes[RIGHT_STICK_X];

    // axes[RIGHT_TRIGGER / LEFT TRIGGER] have values from 1.0 (not pressed) to -1.0 (fully pressed)
    // max value of twist linear command is +/- 1, need to normalize trigger inputs
    double lin_z_right = 0;
    double lin_z_left = 0;

    if(axes[RIGHT_TRIGGER] < 1 )
      lin_z_right = -1 * abs(axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER)) / 2.0;
    if(axes[LEFT_TRIGGER] < 1 )
      lin_z_left = 1 * abs(axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER)) / 2.0;

    // Final twist command, max value +/- 1
    twist->twist.linear.z = velScale*(lin_z_left + lin_z_right);

    // Debug messages
    // RCLCPP_INFO(LOGGER, "axes[RIGHT_TRIGGER]: '%f'", axes[RIGHT_TRIGGER]);   
    // RCLCPP_INFO(LOGGER, "axes[LEFT_TRIGGER]: '%f'", axes[LEFT_TRIGGER]);   
    // RCLCPP_INFO(LOGGER, "lin_z_right: '%f'", lin_z_right);   
    // RCLCPP_INFO(LOGGER, "lin_z_left: '%f'", lin_z_left);   
    // RCLCPP_INFO(LOGGER, "twist->twist.linear.z: '%f'", twist->twist.linear.z);   

    twist->twist.angular.y = -1 * velScale * axes[LEFT_STICK_X];
    twist->twist.angular.x = velScale * axes[LEFT_STICK_Y];

    double roll_positive = -1 * buttons[RIGHT_BUMPER];
    double roll_negative = (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = velScale * (roll_positive + roll_negative);  

    return true;
  }
  else  // Joint Jog Only
  {
      if (axes[RIGHT_STICK_Y] || axes[RIGHT_STICK_X] || axes[D_PAD_X] || axes[D_PAD_Y] || axes[LEFT_STICK_Y] || axes[LEFT_STICK_X] || buttons[B] || buttons[X] )
    {
      // Map the D_PAD to the proximal joints
      joint->joint_names.push_back("ar3_joint1");
      joint->velocities.push_back(axes[D_PAD_X]);
      joint->joint_names.push_back("ar3_joint2");
      joint->velocities.push_back(-1*axes[D_PAD_Y]);  // Invert, so D_PAD down lowers robot

      // Map the diamond to the distal joints

      joint->joint_names.push_back("ar3_joint4");
      joint->velocities.push_back((buttons[B] - buttons[X]));


      joint->joint_names.push_back("ar3_joint4");
      joint->velocities.push_back(axes[RIGHT_STICK_X]);
      joint->joint_names.push_back("ar3_joint3");
      joint->velocities.push_back(-1*axes[RIGHT_STICK_Y]);  // Invert, so D_PAD down lowers robot

      // Map the diamond to the distal joints
      joint->joint_names.push_back("ar3_joint5");
      joint->velocities.push_back(-1*axes[LEFT_STICK_Y]);   // Invert, so D_PAD down lowers robot
      joint->joint_names.push_back("ar3_joint6");
      joint->velocities.push_back(axes[LEFT_STICK_X]);
      return false;
    }
    return true;
  }

}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
{
  std::string temp = frame_name;

  if (buttons[BACK] && frame_name == EEF_FRAME_ID)
    frame_name = BASE_FRAME_ID;
  else if (buttons[START] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;

  if(temp.compare(frame_name) != 0)
  {
    if(frame_name == BASE_FRAME_ID)
      RCLCPP_INFO(LOGGER, "Command Frame: 'BASE_FRAME_ID'");         
    else
      RCLCPP_INFO(LOGGER, "Command Frame: 'EEF_FRAME_ID'");         
  }
}

/** \brief // This should update the command types to change how we control the robot */
void updateCmdType(const std::vector<int>& buttons)
{
  if(buttons[HOME])
  {
    if(command_type == JOINT_JOG)
      command_type = JOG_TWIST;  
    else 
      command_type = JOINT_JOG;
    
    RCLCPP_INFO(LOGGER, "Command Type: '%s'", command_type? "JOG & TWIST" : "JOINT_JOG ONLY");   
  }
}

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, ROS_QUEUE_SIZE, std::bind(&JoyToServoPub::joyCB, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
    collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Create a service client to start the ServoServer
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // Load the collision scene asynchronously
    collision_pub_thread_ = std::thread([this]() {
      rclcpp::sleep_for(std::chrono::seconds(3));
      // Create collision object, in the way of servoing
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = "ar3_link0";
      collision_object.id = "box";

      shape_msgs::msg::SolidPrimitive table_1;
      table_1.type = table_1.BOX;
      table_1.dimensions = { 0.4, 0.6, 0.03 };

      geometry_msgs::msg::Pose table_1_pose;
      table_1_pose.position.x = 0.6;
      table_1_pose.position.y = 0.0;
      table_1_pose.position.z = 0.4;

      shape_msgs::msg::SolidPrimitive table_2;
      table_2.type = table_2.BOX;
      table_2.dimensions = { 0.6, 0.4, 0.03 };

      geometry_msgs::msg::Pose table_2_pose;
      table_2_pose.position.x = 0.0;
      table_2_pose.position.y = 0.5;
      table_2_pose.position.z = 0.25;

      collision_object.primitives.push_back(table_1);
      collision_object.primitive_poses.push_back(table_1_pose);
      collision_object.primitives.push_back(table_2);
      collision_object.primitive_poses.push_back(table_2_pose);
      collision_object.operation = collision_object.ADD;

      moveit_msgs::msg::PlanningSceneWorld psw;
      //psw.collision_objects.push_back(collision_object);

      auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
      ps->world = psw;
      ps->is_diff = true;
      collision_pub_->publish(std::move(ps));
    });
  }

  ~JoyToServoPub() override
  {
    if (collision_pub_thread_.joinable())
      collision_pub_thread_.join();
  }

  void joyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);

    updateCmdType(msg->buttons);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = "ar3_link3";
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::string frame_to_publish_;

  std::thread collision_pub_thread_;
};  // class JoyToServoPub

}  // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)
