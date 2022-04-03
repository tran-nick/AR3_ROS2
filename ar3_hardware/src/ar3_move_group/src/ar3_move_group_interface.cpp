/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Handling Quaternion used in geometry_msgs::Pose
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the *JointModelGroup*. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "ar3_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  // 3rd arg markers topic -- need match rviz config topic value.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "ar3_link0", "move_group_markers",
                                                      move_group.getRobotModel());

  
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal -- N.T. Goal is defined on final pose of the end effector
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  
  // Create Quaternion object from tf2 library so we can abstract and use Euler description of orientation
  // POSITION x = red direction(-right/+left)   y = green direction (-forward/+back)  z = blue direction (-down/+up)
  // ORIENT  looking down marker axix x = red-axis(+CW/-CCW)    y = green-axis(+CW/-CCW)  z = blue-axis(+CW/-CCW)
  // WXYZ are interdependent. Therefore we should never modify them in 
  // their values individually.
  // 3.14 rotate 180 deg
  // 1.57 rotate 90 deg
  // 0.78 rotate 45 deg
  tf2::Quaternion orientation;
  orientation.setRPY(-3.14,0,-3.14); // arguments in radian

  geometry_msgs::msg::Pose draw_ready;
  
  draw_ready.orientation.w = orientation.getW();
  draw_ready.orientation.x = orientation.getX();
  draw_ready.orientation.y = orientation.getY();
  draw_ready.orientation.z = orientation.getZ();
  draw_ready.position.x = 0.0;
  draw_ready.position.y = -0.33;
  draw_ready.position.z = 0.4;

  geometry_msgs::msg::Pose pen2paper = draw_ready;
  pen2paper.position.z = 0.33;
  
  move_group.setPoseTarget(draw_ready);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //-----------draw_ready----------------------------------------------------
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(draw_ready, "draw_ready");
  visual_tools.publishText(text_pose, "draw_ready", rvt::WHITE, rvt::XLARGE);
  // For some unknown reason, other function prototype w/o 2'nd argument does not work
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("ar3_link6") ,joint_model_group, rvt::PINK);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  //-----------pen2paper----------------------------------------------------
  move_group.setPoseTarget(pen2paper);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 as trajectory line");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(pen2paper, "pen2paper");
  visual_tools.publishText(text_pose, "pen2paper", rvt::WHITE, rvt::XLARGE);
  // For some unknown reason, other function prototype w/o 2'nd argument does not work
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("ar3_link6") ,joint_model_group, rvt::PINK);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  //-----------test plane----------------------------------------------------
  // std::vector<geometry_msgs::msg::Pose> test_plane_points;
  // geometry_msgs::msg::Pose temp = target_pose1;
  //
  // temp.position.x += -test_step_size;
  // test_plane_points.push_back(temp);
  // temp.position.y += -test_step_size;
  // test_plane_points.push_back(temp);
  // temp.position.y += -test_step_size;
  // test_plane_points.push_back(temp);
  // temp.position.x -= -test_step_size;
  // test_plane_points.push_back(temp);
  // temp.position.x -= -test_step_size;
  // test_plane_points.push_back(temp);
  // temp.position.y -= -test_step_size;
  // test_plane_points.push_back(temp);  
  // temp.position.x += -test_step_size;
  // test_plane_points.push_back(temp);
  //
  // moveit_msgs::msg::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.05;
  // double fraction = move_group.computeCartesianPath(test_plane_points, eef_step, jump_threshold, trajectory);
  // RCLCPP_INFO(LOGGER, "Visualizing plan SMILEY (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  //
  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Test_Plane", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(test_plane_points, rvt::LIME_GREEN, rvt::XXXSMALL);
  // for (std::size_t i = 0; i < test_plane_points.size(); ++i)
  //   visual_tools.publishAxisLabeled(test_plane_points[i], "pt" + std::to_string(i), rvt::XXXSMALL);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  //
  // for (std::size_t i = 0; i < test_plane_points.size(); ++i)
  // {
  //   move_group.setPoseTarget(test_plane_points[i]);
  //   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //   visual_tools.trigger();
  //   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute next point");
  //   move_group.move();
  // }

  // //---------------Joint space--------------------------------------------------------------------------
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  //
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  //
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);
  //
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  //
  // //  Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("ar3_link6") ,joint_model_group, rvt::PINK);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //------------------Constraint-----------------------------------------------------------------------
  // JointConstraint -- limit position a joint can take
  // OrientationConstraint -- used to keep eg. something upright (or mostly upright with respect to some tolerance).
  /* PositionConstraint -- constraints the Cartesian positions allowed for a (position relative to a) link.
  * https://github.com/ros-planning/moveit_msgs/issues/42
  * 
  * The position constraint will keep the center of the specified link frame inside the constraint region. 
  * Optionally, you can specify an offset. Then it is not the center of the link frame, but the offset 
  * point, expressed relative to the link frame, that has to stay inside the constraint region.
  */
  // Use robot current state as start_state
  moveit::core::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::msg::Pose goal_pose = pen2paper;
  goal_pose.position.y = -0.5;
  goal_pose.orientation.w = orientation.getW();
  goal_pose.orientation.x = orientation.getX();
  goal_pose.orientation.y = orientation.getY();
  goal_pose.orientation.z = orientation.getZ();

  // Planning with Path Constraints -----------------------------------------------------------//
  
  // Define primitive shape for bounding volume
  shape_msgs::msg::SolidPrimitive constrain_box;
  constrain_box.type = constrain_box.BOX;
  constrain_box.dimensions.resize(3);
  constrain_box.dimensions[constrain_box.BOX_X] = 0.01;  
  constrain_box.dimensions[constrain_box.BOX_Y] = 1.0;  
  constrain_box.dimensions[constrain_box.BOX_Z] = 0.01;  // If vallue is too small, return error no solution can be found

  // Define constraining volume pose in planning scene
  // geometry_msgs::msg::Pose constrain_box_pose = pen2paper;
  // constrain_box_pose.position.z = pen2paper.position.z; // set same z-height as drawing surface
  geometry_msgs::msg::PoseStamped constrain_box_pose = move_group.getCurrentPose();
  constrain_box_pose.pose.position.y = goal_pose.position.y;

  // Position constraint
  moveit_msgs::msg::PositionConstraint pcm;
  pcm.link_name = "ar3_link6";        // ee_link
  pcm.header.frame_id = "ar3_link0";  // ref_link
  pcm.weight = 1.0;
  // can only use shape_msgs::msg::SolidPrimitives or Meshes
  pcm.constraint_region.primitives.push_back(constrain_box);
  pcm.constraint_region.primitive_poses.push_back(constrain_box_pose.pose);

  // Orientation constraint
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = pcm.link_name;
  ocm.header.frame_id = pcm.header.frame_id;
  ocm.weight = 1.0;
  ocm.orientation = constrain_box_pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = 0.09;   // control J6 tilt forward / back
  ocm.absolute_y_axis_tolerance = 0.1;    // control J6 tilt left / right
  ocm.absolute_z_axis_tolerance = 0.1; 
  

  start_state.setFromIK(joint_model_group, pen2paper);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(goal_pose);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(30.0);
  
  // Set the path constraints last
  moveit_msgs::msg::Constraints test_constraints;
  // test_constraints.position_constraints.push_back(pcm);
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  if(!success)
  {
    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    // visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
  }

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  // visualize constraint plane
  visual_tools.publishCuboid(
    pen2paper,
    constrain_box.dimensions[constrain_box.BOX_X] ,
    constrain_box.dimensions[constrain_box.BOX_Y] ,
    constrain_box.dimensions[constrain_box.BOX_Z] ,
    rvt::TRANSLUCENT_DARK
  );
  visual_tools.publishAxisLabeled(pen2paper, "start");
  visual_tools.publishAxisLabeled(goal_pose, "goal");
  visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("ar3_link6") ,joint_model_group, rvt::PINK);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();


  //-----------draw test circle----------------------------------------------------

  //Move back to starting position
  move_group.setPoseTarget(pen2paper);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing pen2paper %s", success ? "" : "FAILED");
  RCLCPP_INFO(LOGGER, "Visualizing pen2paper as trajectory line");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(pen2paper, "pen2paper");
  visual_tools.publishText(text_pose, "pen2paper", rvt::WHITE, rvt::XLARGE);
  // For some unknown reason, other function prototype w/o 2'nd argument does not work
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("ar3_link6") ,joint_model_group, rvt::PINK);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.move();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(pen2paper);
  
  geometry_msgs::msg::Pose target_pose3 = pen2paper;
  
  // Smaller resolution value generates more points in circle
  double angle_resolution = 10;

  //radius in meters
  double radius = .075;
  double x_center = target_pose3.position.x;
  double y_center = target_pose3.position.y-radius;
  
  // Trajectory parameters (circle)
  double d_angle = angle_resolution*3.14/180;
  double angle= 0;

  //3. Plan for the trajectory
  for (int i= 0; i< (360/angle_resolution); i++)
  {
    //discretize the trajectory
    angle+= d_angle;
    target_pose3.position.x = x_center + radius*cos(angle);
    target_pose3.position.y = y_center + radius*sin(angle);
    target_pose3.orientation.w = orientation.getW();
    target_pose3.orientation.x = orientation.getX();
    target_pose3.orientation.y = orientation.getY();
    target_pose3.orientation.z = orientation.getZ();
    waypoints.push_back(target_pose3);
    //ROS_INFO("%d",i);
  }
  waypoints.push_back(waypoints[1]);  //connect circle with last point


  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01; //cm -- specify cartesian path interpolation step resolution
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan SMILEY (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::XXXSMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::XXXSMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
  // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // Pull requests are welcome.s
  //
  // You can execute a trajectory like this
  move_group.execute(trajectory); 
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");


  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  // visual_tools.trigger();

  rclcpp::shutdown();
  return 0;

}
