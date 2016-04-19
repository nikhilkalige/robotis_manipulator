//
// Created by lonewolf on 4/15/16.
//

#include "manipulator_driver.h"
#include "device/GenericDevice.h"

ManipulatorDriver::ManipulatorDriver(Robotis::RobotisController* controller, Robotis::GroupHandler* group_handler) {
    controller_ = controller;
    grp_handler_ = group_handler;
   // com_lock_ = &lock;
}

std::vector<double> ManipulatorDriver::read() {
    std::vector<double> joint_states;

    com_lock_.lock();
    grp_handler_->runBulkRead();
    com_lock_.unlock();

    for (int i = 0; i < controller_->idList.size(); ++i) {
        int id = controller_->idList[i];
        int position = 0;
        if(!grp_handler_->getReadData(id, controller_->getDevice(id)->ADDR_PRESENT_POSITION, (long int*)&position)) {
            ROS_WARN("Unable to read position for %s", controller_->getDevice(id)->getJointName());
            joint_states.clear();
            break;
        }
        joint_states.push_back(controller_->getDevice(id)->value2Rad(position));
    }
    return joint_states;
}

void ManipulatorDriver::write(int addr, int data_length, std::vector<unsigned char> param) {
    com_lock_.lock();
    grp_handler_->syncWrite(addr, data_length, &param[0], (int)param.size());
    com_lock_.unlock();
}

void ManipulatorDriver::write_position(std::vector<double> positions) {
    int addr, length, n = 0;
    std::vector<unsigned char> data;
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



