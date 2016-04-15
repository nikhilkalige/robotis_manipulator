#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>

#include "RobotisController.h"
#include "handler/GroupHandler.h"
// #include "manipulator_hardware_interface.h"

#include "manipulator_actions/ControlTorqueAction.h"
// #include "mainpulator_msgs
/// TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class RosWrapper {
protected:
    Robotis::RobotisController controller_;
    Robotis::GroupHandler grp_handler_;
    ros::NodeHandle nh_;
    // boost::shared_ptr<ros_control_manipulator::ManipulatorHardwareInterface> hardware_interface_;
    // boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;

    // Actions
    actionlib::SimpleActionServer<manipulator_actions::ControlTorqueAction> control_torque_as_;
    manipulator_actions::ControlTorqueResult control_torque_result_;

    bool has_goal_;
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;
    ros::Subscriber speed_sub_;
    ros::Subscriber urscript_sub_;
    ros::ServiceServer io_srv_;
    ros::ServiceServer payload_srv_;
    std::thread *rt_publish_thread_;
    std::thread *mb_publish_thread_;
    double io_flag_delay_;
    double max_velocity_;
    std::vector<double> joint_offsets_;
    std::string base_frame_;
    std::string tool_frame_;
    bool use_ros_control_;
    std::thread *ros_control_thread_;

public:
    RosWrapper() :
            controller_(nh_),
            grp_handler_(&controller_),
            control_torque_as_(nh_, "control_torque", boost::bind(&RosWrapper::ControlTorqueExecuteCB, this, _1),
                               false) {
        // Get the joint angles
        std::string joint_prefix = "";
        std::vector<std::string> joint_names;
        char buf[256];

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

        if (!controller_.initialize())
            exit(-1);

        addGroupRead();
        /*hardware_interface_.reset(
                new ros_control_manipulator::ManipulatorHardwareInterface(nh_, &controller_));
        controller_manager_.reset(
                new controller_manager::ControllerManager(
                        hardware_interface_.get(), nh_));*/

    }

    void halt() {
        // robot_.halt();
        rt_publish_thread_->join();
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

        clock_gettime(CLOCK_MONOTONIC, &last_time);
        while (ros::ok()) {
            std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
            std::unique_lock<std::mutex> locker(msg_lock);
            while (!robot_.rt_interface_->robot_state_->getControllerUpdated()) {
                rt_msg_cond_.wait(locker);
            }
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            elapsed_time = ros::Duration(
                    current_time.tv_sec - last_time.tv_sec
                    + (current_time.tv_nsec - last_time.tv_nsec)
                      / BILLION);
            last_time = current_time;
            // Input
            hardware_interface_->read();
            robot_.rt_interface_->robot_state_->setControllerUpdated();
            // Control
            controller_manager_->update(
                    ros::Time(current_time.tv_sec, current_time.tv_nsec),
                    elapsed_time);

            // Output
            hardware_interface_->write();
        }
    }

    void addGroupRead() {
        for (int i = 0; i < controller_.idList.size(); ++i) {
            int id = controller_.idList[i];
            grp_handler_.pushBulkRead(id, controller_.getDevice(id)->ADDR_PRESENT_POSITION);
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

        int n = 0;
        for (int i = 0; i < goal->joint_names.size(); i++) {
            int id = get_id_from_name(goal->joint_names[i]);
            if(id != -1)
            {
                // TODO: Address sharing of usb write control
                /*syncwrite_addr = controller->getDevice(id)->ADDR_TORQUE_ENABLE;
                syncwrite_data_length = controller->getDevice(id)->getAddrLength(syncwrite_addr);
                syncwrite_param.resize(syncwrite_param.size() + syncwrite_data_length + 1); // 2 : ID(1) + TORQUE_ENABLE(1)
                syncwrite_param[n++]  = id;
                if(goal->enable[i] == true)
                    syncwrite_param[n++]  = 1;
                else
                    syncwrite_param[n++]  = 0;*/
                ROS_INFO("Torque %sed for %s", goal->enable[i] ? "enable": "disable", goal->joint_names[i].c_str());
            }
        }
        control_torque_result_.success = true;
        control_torque_as_.setSucceeded(control_torque_result_);
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
