#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace position_velocity_controllers {

    class PositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
    public:
        bool init(std::vector <hardware_interface::JointHandle> &joint_handles, ros::NodeHandle &) {
            joint_handles_ptr_ = &joint_handles;
            return true;
        }

        void starting(const ros::Time & /*time*/) {
            if (!joint_handles_ptr_) { return; }

            // Semantic zero for commands
            for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i) {
                (*joint_handles_ptr_)[i].setCommand((*joint_handles_ptr_)[i].getPosition(), (*joint_handles_ptr_)[i].getVelocity);
            }
        }

        void stopping(const ros::Time &time) { }

        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
            // get joint name from the parameter server
            std::string my_joint;
            if (!n.getParam("joint", my_joint)) {
                ROS_ERROR("Could not find joint name");
                return false;
            }

            // get the joint object to use in the realtime loop
            joint_ = hw->getHandle(my_joint);  // throws on failure
            return true;
        }

        void update(const ros::Time &time, const ros::Duration &period) {
            double error = setpoint_ - joint_.getPosition();
            joint_.setCommand(error * gain_);
        }

        void starting(const ros::Time &time) { }

        void stopping(const ros::Time &time) { }

    private:
        hardware_interface::JointHandle joint_;
        static const double gain_ = 1.25;
        static const double setpoint_ = 3.00;
    };

    PLUGINLIB_DECLARE_CLASS(package_name, PositionController, controller_ns::PositionController, controller_interface::ControllerBase
    );
}//namespace
