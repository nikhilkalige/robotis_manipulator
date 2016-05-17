//
// Created by lonewolf on 4/15/16.
//

#include "manipulator_driver.h"

ManipulatorDriver::ManipulatorDriver(Robotis::RobotisController* controller, Robotis::GroupHandler* group_handler, std::mutex* com_mutex) {
    controller_ = controller;
    grp_handler_ = group_handler;
    com_lock_ = com_mutex;
}

void ManipulatorDriver::initiate_read() {
    com_lock_->lock();
    grp_handler_->runBulkRead();
    com_lock_->unlock();
}

std::vector<double> ManipulatorDriver::get_position() {
    std::vector<double> joint_states;

    // std::cout << "\033[2J\033[1;1H";

    for (int i = 0; i < controller_->idList.size(); ++i) {
        int id = controller_->idList[i];
        long int position = 0;
        if(!grp_handler_->getReadData(id, controller_->getDevice(id)->ADDR_PRESENT_POSITION, &position)) {
            ROS_WARN("Unable to read position for %s", controller_->getDevice(id)->getJointName());
            joint_states.clear();
            break;
        }
        double radian;
        radian = controller_->getDevice(id)->value2Rad((int)position);
        joint_states.push_back(radian);

        /*
        double degree = radian * 180 / 3.14159265;
        printf("Motor: %d Position: %d, Radian: %f, Degree: %f\n", i,  (int)position, radian, degree);
        printf("Motor: %d Position: %#x, %#x, Radian: %f, Degree: %f\n", i, (int)position >> 16, (int)position, realrad, degree);
        */
    }
    return joint_states;
}

std::vector<double> ManipulatorDriver::get_velocity() {
    std::vector<double> joint_states;

    // std::cout << "\033[2J\033[1;1H";

    for (int i = 0; i < controller_->idList.size(); ++i) {
        int id = controller_->idList[i];
        long int velocity = 0;
        if(!grp_handler_->getReadData(id, controller_->getDevice(id)->ADDR_PRESENT_VELOCITY, &velocity)) {
            ROS_WARN("Unable to read position for %s", controller_->getDevice(id)->getJointName());
            joint_states.clear();
            break;
        }

        double velocity_ms;
        velocity_ms = controller_->getDevice(id)->value2Rpm((int)velocity);
        joint_states.push_back(velocity_ms);

        /*
        double degree = radian * 180 / 3.14159265;
        printf("Motor: %d Position: %d, Radian: %f, Degree: %f\n", i,  (int)position, radian, degree);
        printf("Motor: %d Position: %#x, %#x, Radian: %f, Degree: %f\n", i, (int)position >> 16, (int)position, realrad, degree);
        */
    }
    return joint_states;
}

void ManipulatorDriver::write(int addr, int data_length, std::vector<unsigned char> param) {
    com_lock_->lock();
    grp_handler_->syncWrite(addr, data_length, &param[0], (int)param.size());
    com_lock_->unlock();
}

void ManipulatorDriver::write_position(std::vector<double> positions) {
    int addr, length, n = 0;
    std::vector<unsigned char> data;

    // printf("%f %f %f %f %f %f\n",positions[0], positions[1], positions[2],\
    //  positions[3], positions[4], positions[5]);

    for (int i = 0; i < positions.size(); ++i) {
        int id = controller_->idList[i];
        Robotis::GenericDevice* device = controller_->getDevice(id);

        addr = device->ADDR_GOAL_POSITION;
        length = device->getAddrLength(addr);
        int pos = device->rad2Value(positions[i]);
        data.resize(data.size() + length + 1);
        data[n++]  = id;
        if(length == 2)
        {
            data[n++]  = DXL_LOBYTE(pos);
            data[n++]  = DXL_HIBYTE(pos);
        }
        else if(length == 4)
        {
            data[n++]  = DXL_LOBYTE(DXL_LOWORD(pos));
            data[n++]  = DXL_HIBYTE(DXL_LOWORD(pos));
            data[n++]  = DXL_LOBYTE(DXL_HIWORD(pos));
            data[n++]  = DXL_HIBYTE(DXL_HIWORD(pos));
        }
    }
    write(addr, length, data);
}

void ManipulatorDriver::write_velocity(std::vector<double> velocities) {
    int addr, length, n = 0;
    std::vector<unsigned char> data;

    // printf("%f %f %f %f %f %f\n",velocities[0], velocities[1], velocities[2],
    //       velocities[3], velocities[4], velocities[5]);

    for (int i = 0; i < velocities.size(); ++i) {
        int id = controller_->idList[i];
        Robotis::GenericDevice* device = controller_->getDevice(id);

        addr = device->ADDR_GOAL_VELOCITY;
        length = device->getAddrLength(addr);
        int pos = device->rpm2Value(velocities[i]);
        data.resize(data.size() + length + 1);
        data[n++]  = id;
        if(length == 2)
        {
            data[n++]  = DXL_LOBYTE(pos);
            data[n++]  = DXL_HIBYTE(pos);
        }
        else if(length == 4)
        {
            data[n++]  = DXL_LOBYTE(DXL_LOWORD(pos));
            data[n++]  = DXL_HIBYTE(DXL_LOWORD(pos));
            data[n++]  = DXL_LOBYTE(DXL_HIWORD(pos));
            data[n++]  = DXL_HIBYTE(DXL_HIWORD(pos));
        }
    }
    write(addr, length, data);
}

bool ManipulatorDriver::switch_mode(bool postion_mode) {
    int mode_value;
    if (postion_mode) {
        ROS_DEBUG("Switching to position control mode");
        mode_value = 3;
    }
    else {
        ROS_DEBUG("Switching to velocity control mode");
        mode_value = 1;
    }

    // The torque needs to be disabled in order to change the operating mode, so
    // save the torque -> change mode -> rewrite torque
    com_lock_->lock();
    for (int i = 0; i < controller_->idList.size(); ++i) {
        int id = controller_->idList[i];
        int torque;

        controller_->getTorqueEnable(id, &torque);
        controller_->setOperatingMode(id, mode_value);
        controller_->setTorqueEnable(id, torque);
    }
    com_lock_->unlock();
    ROS_DEBUG("Completed switching the mode");
}



