#ifndef MANIPULATOR_ROS_CONTROL_MANIPULATOR_HARDWARE_INTERFACE_H
#define MANIPULATOR_ROS_CONTROL_MANIPULATOR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
// #include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <math.h>
#include "manipulator_driver.h"


namespace ros_control_manipulator {

// For simulation only - determines how fast a trajectory is followed
static const double POSITION_STEP_FACTOR = 1;
static const double VELOCITY_STEP_FACTOR = 1;

/// \brief Hardware interface for a robot
class ManipulatorHardwareInterface: public hardware_interface::RobotHW {
public:

    /**
     * \brief Constructor
     * \param nh - Node handle for topics.
     */
    ManipulatorHardwareInterface(ros::NodeHandle& nh, ManipulatorDriver* robot);

    /// \brief Initialize the hardware interface
    virtual void init();

    /// \brief Read the state from the robot hardware.
    virtual void read();

    /// \brief write the command to the robot hardware.
    virtual void write();

    /// \brief initialize the command variable until the controller starts
    virtual void hold();

    /*bool prepareSwitch(const std::list<ControllerInfo>& start_list,
                       const std::list<ControllerInfo>& stop_list) { return true; }*/

    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                  const std::list<hardware_interface::ControllerInfo>& stop_list);

protected:

    // Startup and shutdown of the internal node inside a roscpp program
    ros::NodeHandle nh_;

    // Interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // Shared memory
    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;

    bool velocity_interface_running_;
    bool position_interface_running_;

    std::size_t num_joints_;
    double robot_force_[3] = { 0., 0., 0. };
    double robot_torque_[3] = { 0., 0., 0. };

    double max_vel_change_;

    // Robot API
    ManipulatorDriver* robot_;
};
// class

}// namespace

#endif
