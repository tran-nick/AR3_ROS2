#include "ar3_hardware_interface.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "emergency_stop/emergency_stop_subscriber.hpp"

#include <rclcpp/executors.hpp>

/* 
> This is loaded as a plugin when ROS2 loads the urdf robot description

> Fuctions are called by the ros2_control_node. See ros2_control_node logs for debug prints
*/

namespace ar3_hardware_interface
{

/* 
> hardware_interface::HardwareInfo -- This structure stores information about hardware defined in a robot's URDF.

> configure() function for pconfiguring controllers and parameters that were parsed from URDF 

> hardware_paramters are defined under ros2_control.xacro
*/
hardware_interface::return_type AR3HardwareInterface::configure(
    const hardware_interface::HardwareInfo & info
)
{
  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "configure() called");

  if(configure_default(info) != hardware_interface::return_type::OK)
  {
      return hardware_interface::return_type::ERROR;
  }

  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Calling init() to resize vectors ");

  //------ Call initialize function to resize vectors---------
  init(); 

  // Get hw timing params
  hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec"]);

  // Variable info_ defined in base_interface.hpp, of class HardwareInfo
  // For loop is error checking ComponentInfo for each joint
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    /* 
    AR3HardwareInterface has exactly 2 state interfaces (position/velocity)
    and 2 command interfaces (position/velocity) on each joint. These are 
    defined in the ros2_control.xacro
     */
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3HardwareInterfaceHardware"),
        "Joint '%s' has %d command interfaces. 2 expected.", joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3HardwareInterfaceHardware"),
        "Joint '%s' has %s command interface. Expected %s or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3HardwareInterfaceHardware"),
        "Joint '%s'has %d state interfaces. 2 expected.", joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3HardwareInterfaceHardware"),
        "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  // Debug print

  status_ = hardware_interface::status::CONFIGURED;
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "configure() exit");
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> AR3HardwareInterface::export_state_interfaces()
{
  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "export_state_interfaces() called");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
  }

  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "export_command_interfaces() exit");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AR3HardwareInterface::export_command_interfaces()
{
  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "export_command_interfaces() called");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[i]));
  }


  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "export_command_interfaces() exit");

  return command_interfaces;
}


void AR3HardwareInterface::init()
{

  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "init() called");

  // Get number of joints
  num_joints_ = static_cast<int>(info_.joints.size());
	// num_joints_ = static_cast<int>(joint_names_.size());

	if (num_joints_ == 0)
	{
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("AR3HardwareInterface"),"No joints found. Did the hardware_interface::HardwareInfo & info initialze correctly?");
	}


	// resize vectors
  joint_names_.resize(num_joints_);
	actuator_commands_.resize(num_joints_);
	actuator_positions_.resize(num_joints_);
	joint_positions_.resize(num_joints_);
	joint_velocities_.resize(num_joints_);
	joint_efforts_.resize(num_joints_);
	joint_position_commands_.resize(num_joints_);
	joint_velocity_commands_.resize(num_joints_);
	joint_effort_commands_.resize(num_joints_);
	joint_offsets_.resize(num_joints_);
	joint_lower_limits_.resize(num_joints_);
	joint_upper_limits_.resize(num_joints_);
	velocity_limits_.resize(num_joints_);
	acceleration_limits_.resize(num_joints_);
  enc_steps_per_deg.resize(num_joints_);

  // Debug print
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "init() exit");
}


/* 
 8/26/2021 Note:
 This is where you should grab all param values and configure the driver_, 
 create it as pointer object and access functinos by ->. See ROS2 UR robot driver as example
 */
hardware_interface::return_type AR3HardwareInterface::start()
{
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Starting AR3HardwareInterface...please wait...");

  // Give some time to run
  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("AR3HardwareInterface"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Getting enconder calibration...");
  // get encoder calibration
  // num_joints_ value set in init(), set by sizeof(joint_names_) given by parameters
	
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "for loop...");
	for (int i = 0; i < num_joints_; ++i)
	{
    joint_names_[i] = info_.joints[i].name;
    
    enc_steps_per_deg[i] = stod(info_.hardware_parameters["encoder_steps_per_deg_"+joint_names_[i]]);
    joint_offsets_[i] = stod(info_.hardware_parameters["joint_offsets_"+joint_names_[i]]);
	}

  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Initializing motor driver...");
	// init motor driver
	std::string serial_port;
	int baudrate;
  serial_port = info_.hardware_parameters["serial_port"];
  baudrate = stoi(info_.hardware_parameters["baudrate"]);
  driver_ = std::make_unique<ar3_hardware_drivers::TeensyDriver>();

  /* 
  Initialize calls TeensyDriver.cpp, should print serial connection status
  N.T 9/1/2021, if error is permission denied, need to sudo chmod 777 /dev/ttyACM0
  */
	driver_->init(serial_port, baudrate, num_joints_, enc_steps_per_deg);

	// set velocity limits
	for (int i = 0; i < num_joints_; ++i)
	{
    // velocity_limits_[i] = info_.hardware_parameters
    velocity_limits_[i] = stod(info_.hardware_parameters["max_velocity"]);
    acceleration_limits_[i] = stod(info_.hardware_parameters["max_acceleration"]);
    
		velocity_limits_[i] = radToDeg(velocity_limits_[i]);
		acceleration_limits_[i] = radToDeg(acceleration_limits_[i]);
	}

  // RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Debug print 2");
	driver_->setStepperSpeed(velocity_limits_, acceleration_limits_);

	// calibrate joints if needed
	bool use_existing_calibrations = false;
  use_existing_calibrations = (info_.hardware_parameters["use_existing_calibrations"] == "true"); //logic convert from string to bool
	if (!use_existing_calibrations)
	{
		// run calibration
		RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"),"Running joint calibration...");
		driver_->calibrateJoints();
	}

	// init position commands at current positions
	driver_->getJointPositions(actuator_positions_);
	for (int i = 0; i < num_joints_; ++i)
	{
		// apply offsets, convert from deg to rad for moveit
		joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
		joint_position_commands_[i] = joint_positions_[i];
	}

  // start emergency stop subscriber node
  emergency_nh = std::make_shared<emergency_stop::EmergencyStopSubscriber>();

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("AR3HardwareInterface"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AR3HardwareInterface::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Stopping AR3HardwareInterface...please wait...");

  // Give some time to stop
  for (int i = 0; i <= hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("AR3HardwareInterface"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  // Shutdown Emergency Stop subscriber node
  rclcpp::shutdown();

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("AR3HardwareInterface"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}


//---------controller_manager ros2_control_node calls these read/write funcs-------------------
// Function commented out because it depends on retrofitted safety fence hardware not standard to AR3/AR4
/*
hardware_interface::return_type AR3HardwareInterface::read()
{
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Reading...");

  rclcpp::spin_some(emergency_nh);  // spin to read emergency stop range values
  
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Safety Sensor Range: '%f'",emergency_nh->range_value);
    
  switch (status_)
  {
    case hardware_interface::status::STARTED :
      
      RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "In STARTED case");
      
      if(emergency_nh->range_value < 20 && emergency_nh->range_value > 5)    // if range less than 50 cm (20") 
      {
        status_ = hardware_interface::status::STOPPED;
        driver_->emergencyStop();       // send stop commant to Teensy
      }

      //else -- read joint positions
      driver_->getJointPositions(actuator_positions_);

      RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "end STARTED case");
      
      break;
    case (hardware_interface::status::STOPPED) :
      
      RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "In STOPPED case");
      
      // if STOPPED, keep polling range value for when distance is fixed
      while(emergency_nh->range_value < 20 || emergency_nh->button_pressed == 0)
      {
        rclcpp::spin_some(emergency_nh);  

        if(emergency_nh->range_value >= 20 && emergency_nh->button_pressed == 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Clearing Estop");
            driver_->clearEStop();
            status_ = hardware_interface::status::STARTED;
        }    
      }
      RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "end STOPPED case");
      break;

    default:
      RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "default case");
      break;
  }

  // apply offsets, convert from deg to rad for moveit
  for(int i = 0; i < num_joints_; i++)
  {    
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
    
    // change RCLCPP_INFO to report position and commands as degrees. Easier to compare values.
    RCLCPP_INFO(
    rclcpp::get_logger("AR3HardwareInterface"), "Got position %.5f for joint %d!",
    actuator_positions_[i], i+1);
  }

  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Joints successfully read!");
  return hardware_interface::return_type::OK;
} */


hardware_interface::return_type AR3HardwareInterface::read()
{
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Reading...");

  // read joint positions
  driver_->getJointPositions(actuator_positions_);

  // apply offsets, convert from deg to rad for moveit
  for(int i = 0; i < num_joints_; i++)
  {    
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
    
    // change RCLCPP_INFO to report position and commands as degrees. Easier to compare values.
    RCLCPP_INFO(
    rclcpp::get_logger("AR3HardwareInterface"), "Got position %.5f for joint %d!",
    actuator_positions_[i], i+1);
  }

  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Joints successfully read!");
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AR3HardwareInterface::write()
{
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Writing...");
  for(int i = 0; i < num_joints_ ; i++)
  {
    // How are commands getting from rviz / moveit servo to joint_position_commands_ via command interfaces?

    // convert from rad to deg, apply offsets -- ROS/moveit report in radians. Convert to degrees for teensy driver
    // Note: you'll always have small delta between position/command value due to conversion error
    actuator_commands_[i] = radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
    RCLCPP_INFO(
    rclcpp::get_logger("AR3HardwareInterface"), "Got command %.5f for joint %d!",
    actuator_commands_[i], i+1);
  }
  driver_->update(actuator_commands_, actuator_positions_); 
  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "driver->update()");

  RCLCPP_INFO(rclcpp::get_logger("AR3HardwareInterface"), "Joints successfully written!");  
  return hardware_interface::return_type::OK;
}

double AR3HardwareInterface::degToRad(double deg)
{
    return deg / 180.0 * M_PI;
}

double AR3HardwareInterface::radToDeg(double rad)
{
    return rad / M_PI * 180.0;
}

}   //namespace ar3_hardware_interface

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
  ar3_hardware_interface::AR3HardwareInterface , hardware_interface::SystemInterface)
