#include "emergency_stop/emergency_stop.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<emergency_stop::EmergencyStop>());
  rclcpp::shutdown();
  return 0;
}


/* nhtran@ubuntu:~/ar3_ros2_control_simulation_only$ ros2 service call ar3_arm_controller/follow_joint_trajectory/_action/cancel_goal action_msgs/srv/CancelGoal
waiting for service to become available...
requester: making request: action_msgs.srv.CancelGoal_Request(goal_info=action_msgs.msg.GoalInfo(goal_id=unique_identifier_msgs.msg.UUID(uuid=array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=uint8)), stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0)))

 */


/* 
Following this example here
https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-service-node 
*/