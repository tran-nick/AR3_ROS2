#ifndef AR3_HARDWARE_INTERACE_HPP_
#define AR3_HARDWARE_INTERACE_HPP_

// Std includes
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <sstream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

// Teensy drivers
#include "ar3_hardware_drivers/TeensyDriver.hpp"
#include "visibility.h"

// Hardware interface
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "emergency_stop/emergency_stop_subscriber.hpp"


namespace ar3_hardware_interface
{
    class AR3HardwareInterface
    : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
    {
            public:
            // For use with logging macros to pass to rclcpp::get_logger
            RCLCPP_SHARED_PTR_DEFINITIONS(AR3HardwareInterface); // Macro creates new node with provided name
            
            hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info);
            std::vector<hardware_interface::StateInterface> export_state_interfaces();
            std::vector<hardware_interface::CommandInterface> export_command_interfaces();

            void init();
            hardware_interface::return_type start();
            hardware_interface::return_type stop();
            // no update() in ROS2 control demo's
            hardware_interface::return_type read();
            hardware_interface::return_type write();


            private:
                // std::shared_ptr<rclcpp::Node> nh_;    
                // rclcpp::Timer non_realtime_loop_;    TO DO -- how implement for ROS2?
                // rclcpp::Duration control_period_;
                // rclcpp::Duration elapsed_time_;

                // TO DO: rewrite these variables to ROS2 equivalent
                // Interfaces no longer available in ROS2
                // PositionJointInterface positionJointInterface;
                // PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;

                double loop_hz_;
                // Boost shared_ptr became part of stdlib, use stdlib instead in <memory>
                // std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
                double p_error_, v_error_, e_error_;

                // Motor driver
                std::unique_ptr<ar3_hardware_drivers::TeensyDriver> driver_;
                std::vector<double> actuator_commands_;     //ros2_control demo -- hw_commands_
                std::vector<double> actuator_positions_;    //ros2_control demo -- hw_states_

                // TO DO -- need to rewrite these API calls to ROS2_control equivalent
                // // Interfaces -- these libs to longer part of ROS2
                // hardware_interface::JointStateInterface joint_state_interface_;
                // hardware_interface::PositionJointInterface position_joint_interface_;

                // joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
                // joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;

                // Emergency Stop
                std::shared_ptr<emergency_stop::EmergencyStopSubscriber> emergency_nh;

                // Shared memory
                int num_joints_;
                double hw_start_sec_;
                double hw_stop_sec_;
                std::vector<std::string> joint_names_;
                std::vector<double> joint_offsets_;
                std::vector<double> joint_positions_;
                std::vector<double> joint_velocities_;
                std::vector<double> joint_efforts_;
                std::vector<double> joint_position_commands_;
                std::vector<double> joint_velocity_commands_;
                std::vector<double> joint_effort_commands_;
                std::vector<double> joint_lower_limits_;
                std::vector<double> joint_upper_limits_;
                std::vector<double> velocity_limits_;
                std::vector<double> acceleration_limits_;
                std::vector<double> enc_steps_per_deg;


                // Misc
                double degToRad(double deg);
                double radToDeg(double rad);
    };


}

#endif