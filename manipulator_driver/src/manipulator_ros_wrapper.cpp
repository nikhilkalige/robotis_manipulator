#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <chrono>
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/simple_action_server.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>

#include "RobotisController.h"
#include "handler/GroupHandler.h"
#include "manipulator_hardware_interface.h"

#include "manipulator_actions/ControlTorqueAction.h"
#include "manipulator_actions/ControlTableAction.h"

#include "manipulator_msgs/KeyValue.h"
#include "manipulator_driver.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


// Disables read and write commands in the control loop
// #define SIMULATION

// Prints the control loop rate in Hz
// #define CONTROLLOOP_SPEED

// Hold all the address values that are retrieved during the controltable action
static std::map<std::string, int> table_addr = {
    {"BaudRate", 8},
    {"Return Delay Time", 9},
    {"Operating Mode", 11},
    {"Homing Offset", 13},
    {"Moving Threshold", 17},
    {"Temperature Limit", 21},
    {"Max Voltage Limit", 22},
    {"Min Voltage Limit", 24},
    {"Acceleration Limit", 26},
    {"Torque limit", 30},
    {"Velocity Limit", 32},
    {"Max Position Limit", 36},
    {"Min Position Limit", 40},
    {"Torque Enable", 562},
    {"LED Red", 563},
    {"LED Green", 564},
    {"LED Blue", 565},
    {"Velocity I Gain", 586},
    {"Velocity P Gain", 588},
    {"Position P Gain", 594},
    {"Goal Position", 596},
    {"Goal Velocity", 600},
    {"Goal Torque", 604},
    {"Goal Acceleration", 606},
    {"Moving", 610},
    {"Present Position", 611},
    {"Present Velocity", 615},
    {"Present Current", 621},
    {"Present Input Voltage", 623},
    {"Present Temperature", 625}
};

class RosWrapper {
protected:
    ros::NodeHandle nh_;
    std::mutex com_lock_;
    std::string ns_;
    Robotis::RobotisController controller_;
    Robotis::GroupHandler grp_handler_;

    ManipulatorDriver driver_;

    boost::shared_ptr<ros_control_manipulator::ManipulatorHardwareInterface> hardware_interface_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // Actions
    actionlib::SimpleActionServer<manipulator_actions::ControlTorqueAction> control_torque_as_;
    manipulator_actions::ControlTorqueResult control_torque_result_;

    actionlib::SimpleActionServer<manipulator_actions::ControlTableAction> control_table_as_;
    manipulator_actions::ControlTableResult control_table_result_;

    std::thread *ros_control_thread_;

public:
    RosWrapper() :
            controller_(nh_),
            grp_handler_(&controller_),
            driver_(&controller_, &grp_handler_, &com_lock_),
            control_torque_as_(nh_, "control_torque", boost::bind(&RosWrapper::ControlTorqueExecuteCB, this, _1),
                               false),
            control_table_as_(nh_, "control_table", boost::bind(&RosWrapper::ControlTableExecuteCB, this, _1),
                               false) {
        // Get the joint angles
        std::string joint_prefix = "";
        std::vector<std::string> joint_names;
        char buf[256];

        ns_ = ros::this_node::getNamespace();
        ROS_INFO("Running with namespace - %s", ns_.c_str());

        if (ros::param::get("~prefix", joint_prefix)) {
            if (joint_prefix.length() > 0) {
                ROS_INFO("Setting prefix to %s", joint_prefix.c_str());
            }
        }
        joint_names.push_back(joint_prefix + "shoulder_pan_joint");
        joint_names.push_back(joint_prefix + "shoulder_lift_joint");
        joint_names.push_back(joint_prefix + "elbow_joint");
        joint_names.push_back(joint_prefix + "wrist_1_joint");
        joint_names.push_back(joint_prefix + "wrist_2_joint");
        joint_names.push_back(joint_prefix + "wrist_3_joint");

#ifndef SIMULATION
        if (!controller_.initialize())
            exit(-1);
        addGroupRead();
#endif
        hardware_interface_.reset(
                new ros_control_manipulator::ManipulatorHardwareInterface(nh_, &driver_));
        controller_manager_.reset(
                new controller_manager::ControllerManager(
                        hardware_interface_.get(), nh_));

        control_torque_as_.start();
        control_table_as_.start();
        ros_control_thread_ = new std::thread(
                boost::bind(&RosWrapper::rosControlLoop, this));
        ROS_DEBUG("The control thread for this driver has been started");
    }

    void halt() {
        // robot_.halt();
    }

    int get_id_from_name(const std::string& joint_name) {
        for (int i = 0; i < controller_.idList.size(); ++i) {
            int id = controller_.idList[i];
            if (strcmp(joint_name.c_str(), controller_.getDevice(id)->getJointName()) == 0)
                return id;
        }
        return -1;
    }

private:
    void rosControlLoop() {
        ros::Duration elapsed_time;
        struct timespec last_time, current_time;
        static const double BILLION = 1000000000.0;

#ifdef CONTROLLOOP_SPEED
        ros::Time speed;
        int temp = 0;
#endif
        hardware_interface_->hold();
        ROS_INFO("Starting the control loop");

        clock_gettime(CLOCK_MONOTONIC, &last_time);

        while (ros::ok()) {
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            elapsed_time = ros::Duration(
                    current_time.tv_sec - last_time.tv_sec
                    + (current_time.tv_nsec - last_time.tv_nsec)
                      / BILLION);
            last_time = current_time;

#ifndef SIMULATION
            hardware_interface_->read();
#endif
            controller_manager_->update(ros::Time::now(), elapsed_time);

#ifndef SIMULATION
            hardware_interface_->write();
#endif
#ifdef CONTROLLOOP_SPEED
            temp++;
            if (temp == 1)
                speed = ros::Time::now();

            if (temp == 10000) {
                temp = 0;
                ros::Duration hz = ros::Time::now() - speed;
                ROS_DEBUG_STREAM_NAMED(ns_, "Hz " <<  (10000.0 / hz.toSec()));
            }
#endif
            
        }
    }

    void addGroupRead() {
        for (int i = 0; i < controller_.idList.size(); ++i) {
            int id = controller_.idList[i];
            // Use length 8 - position(4) + velocity(4)
            grp_handler_.pushBulkRead(id, controller_.getDevice(id)->ADDR_PRESENT_POSITION, 8);
        }
    }

    void ControlTorqueExecuteCB(const manipulator_actions::ControlTorqueGoalConstPtr &goal) {
        // Validate the message
        if ((goal->joint_names.size() == 0) || (goal->enable.size() != goal->joint_names.size())) {
            ROS_INFO("ControlTorque Action Failed: Invalid Message");
            control_torque_result_.success = false;
            control_torque_as_.setAborted(control_torque_result_);
            return;
        }

        int n = 0, addr, length;
        std::vector<unsigned char> param;
        for (int i = 0; i < goal->joint_names.size(); i++) {
            int id = get_id_from_name(goal->joint_names[i]);
            if(id != -1)
            {
                addr = controller_.getDevice(id)->ADDR_TORQUE_ENABLE;
                length = controller_.getDevice(id)->getAddrLength(addr);
                param.resize(param.size() + length + 1); // 2 : ID(1) + TORQUE_ENABLE(1)
                param[n++] = (unsigned char)id;
                if(goal->enable[i])
                    param[n++]  = 1;
                else
                    param[n++]  = 0;
                ROS_INFO("Torque %sd for %s", goal->enable[i] ? "enable": "disable", goal->joint_names[i].c_str());
            }
        }

        if(param.size() != 0) {
            driver_.write(addr, length, param);
            control_torque_result_.success = true;
        }
        else {
            ROS_INFO("Torque Action failed, Unable to find valid id's for joint name");
            control_torque_result_.success = false;
        }
        control_torque_as_.setSucceeded(control_torque_result_);
    }

    void ControlTableExecuteCB(const manipulator_actions::ControlTableGoalConstPtr &goal) {
        // Validate the message
        if (goal->joint_names.size() == 0) {
            ROS_INFO("ControlTable Action Failed: Invalid Message");
            control_table_result_.success = false;
            control_table_as_.setAborted(control_table_result_);
            return;
        }

        std::vector<int> motor_ids;
        for (int i = 0; i < goal->joint_names.size(); i++) {
            int id = get_id_from_name(goal->joint_names[i]);
            motor_ids.push_back(id);
        }

        if(motor_ids.size() == 0) {
            ROS_INFO("ControlTable Action Failed: Invalid Message");
            control_table_result_.success = false;
            control_table_as_.setAborted(control_table_result_);
            return;
        }

        control_table_result_.table.clear();
        com_lock_.lock();
        for (std::map<std::string, int>::iterator it = table_addr.begin(); it != table_addr.end(); ++it) {
            manipulator_msgs::KeyValue row;
            row.key = it->first;
            for (int i = 0; i < motor_ids.size(); ++i) {
                long value;
                controller_.read(motor_ids[i], it->second, &value);
                row.values.push_back((int)value);
            }
            control_table_result_.table.push_back(row);
        }
        com_lock_.unlock();
        control_table_result_.success = true;
        control_table_as_.setSucceeded(control_table_result_);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "manipulator_driver");
    ros::NodeHandle nh;

    RosWrapper interface;

    ros::AsyncSpinner spinner(3);

    spinner.start();

    ros::waitForShutdown();

    interface.halt();
    exit(0);
}
