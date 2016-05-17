/*
 *=====================================================
 * File   :  RobotisController.cpp
 * Author :  zerom <zerom@robotis.com>
 * Copyright (C) Robotis, 2015
 *=====================================================
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <ros/ros.h>

#include "RobotisController.h"

using namespace Robotis;

RobotisController::RobotisController(ros::NodeHandle& nh) : nh_(nh)
{
    packetHandlerList.push_back(PacketHandler::getPacketHandler(1.0));
    packetHandlerList.push_back(PacketHandler::getPacketHandler(2.0));
}

RobotisController::~RobotisController()
{

}

bool RobotisController::initialize()
{
    bool result = false;
    std::string node_namespace = "";
    // std::string namespace = "/leftarm/";

    std::string name;
    if(!ros::param::get("serial_port/name", name)) {
        ROS_ERROR("No valid serial port name found");
        return false;
    }
    //name = "/dev/ttyUSB";

    int baudrate = 1000000;
    ros::param::get("serial_port/baudrate", baudrate);
    ROS_INFO("Opening port \"%s\" with baudrate \"%d\"", name.c_str(), baudrate);

    int device_count = 0;
    ros::param::get("arm_config/count", device_count);
    if(device_count == 0) {
        ROS_ERROR("Invalid device count");
        return false;
    }
    ROS_INFO("Device Count: %d", device_count);

    if(!addSerialPort(name.c_str(), baudrate)) {
        ROS_ERROR("Unable to open serial port");
        return false;
    }

    ROS_INFO("Initializing the servos");

    XmlRpc::XmlRpcValue servos;
    if(!ros::param::get("arm_config/devices", servos)) {
        ROS_ERROR("No valid config found for the servos");
        return false;
    }
    if(servos.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Invalid yaml file loaded");
        return false;
    }
    for (int i = 0; i < servos.size(); ++i) {
        XmlRpc::XmlRpcValue servo;
        int id = 0;
        double prot_ver = 2.0;
        /*std::string model = "";
        std::string joint_name = "";*/

        servo = servos[i];
        id = servo["id"];
        prot_ver = servo["protocol"];
        std::string model = servo["model"];
        std::string joint_name = servo["joint_name"];
        if((id == 0) || (model == "") || (joint_name == "")) {
            ROS_ERROR("Invalid Servo Config");
            return false;
        }

        addDevice(portList.back(), id, joint_name.c_str(), model.c_str(), (float)prot_ver);
        ROS_INFO("Servo: %d, ID: %d, Model: %s, Ver: %f, Joint: %s added", i, id, model.c_str(), prot_ver, joint_name.c_str());
    }
    return true;
}

bool RobotisController::addSerialPort(const char* port_name, int baudrate)
{
    portList.push_back(new PortHandler(port_name));
    if(portList.back()->openPort() == false)
    {
        portList.pop_back();
        return false;
    }
    if(portList.back()->changeBaudRate(baudrate) == false)
    {
        portList.pop_back();
        return false;
    }
    return true;
}

void RobotisController::addDevice(PortHandler* port, int id, const char* joint_name, const char* model, float protocol_ver)
{
    idList.push_back(id);
    dxlList[id] = GenericDevice::getInstance(port, id, joint_name, model, protocol_ver);
}

GenericDevice *RobotisController::getDevice(int id)
{
    return dxlList[id];
}

int RobotisController::read(int id, int address, long *data, int *error)
{
    if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
        return COMM_TX_ERROR;

    return dxlList[id]->read(address, data, error);
}

int RobotisController::read(int id, int address, long *data, LENGTH_TYPE length, int *error)
{
    if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
        return COMM_TX_ERROR;

    return dxlList[id]->read(address, data, length, error);
}

int RobotisController::write(int id, int address, long data, int *error)
{
    if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
        return COMM_TX_ERROR;

    return dxlList[id]->write(address, data, error);
}

int RobotisController::write(int id, int address, long data, LENGTH_TYPE length, int *error)
{
    if(id == BROADCAST_ID)
    {
        for(int i = 0; i < portList.size(); i++)
        {
            for(int j = 0; j < packetHandlerList.size(); j++)
            {
                unsigned char *write_data = new unsigned char[length];

                switch(length)
                {
                case 1:
                    write_data[0] = data;
                    break;
                case 2:
                    write_data[0] = DXL_LOBYTE(data);
                    write_data[1] = DXL_HIBYTE(data);
                    break;
                case 4:
                    write_data[0] = DXL_LOBYTE(DXL_LOWORD(data));
                    write_data[1] = DXL_HIBYTE(DXL_LOWORD(data));
                    write_data[2] = DXL_LOBYTE(DXL_HIWORD(data));
                    write_data[3] = DXL_HIBYTE(DXL_HIWORD(data));
                    break;
                default:
                    break;
                }

                packetHandlerList[j]->write(portList[i], id, address, (int)length, write_data);
            }
        }
    }
    else if(std::find(idList.begin(), idList.end(), id) == idList.end() || dxlList[id] == NULL) // id is not exist.
    {
        return COMM_TX_ERROR;
    }
    else
    {
        return dxlList[id]->write(address, data, length, error);
    }
}

int RobotisController::getTorqueEnable(int id, int *enable)
{
    return dxlList[id]->read(dxlList[id]->ADDR_TORQUE_ENABLE, (long*)enable);
}

int RobotisController::setTorqueEnable(int id, int enable)
{
    return dxlList[id]->write(dxlList[id]->ADDR_TORQUE_ENABLE, enable);
}

int RobotisController::getPresentPositionRadian(int id, double *radian)
{
    long position = 0;
    int result = dxlList[id]->read(dxlList[id]->ADDR_PRESENT_POSITION, &position);

    if(result == 0)
        *radian = dxlList[id]->value2Rad(position);
    else
        radian = 0;

    return result;
}

int RobotisController::getPresentPositionValue(int id, long *position)
{
    return dxlList[id]->read(dxlList[id]->ADDR_PRESENT_POSITION, position);;
}

int RobotisController::getPresentVelocity(int id, long *velocity)
{
    return dxlList[id]->read(dxlList[id]->ADDR_PRESENT_VELOCITY, velocity);;
}

int RobotisController::getPresentLoad(int id, long *load)
{
    return dxlList[id]->read(dxlList[id]->ADDR_PRESENT_LOAD, load);;
}

int RobotisController::getGoalPositionRadian(int id, double *radian)
{
    long position = 0;
    int result = dxlList[id]->read(dxlList[id]->ADDR_GOAL_POSITION, &position);

    if(result == 0)
        *radian = dxlList[id]->value2Rad(position);
    else
        radian = 0;

    return result;
}

int RobotisController::setGoalPositionRadian(int id, double radian)
{
    long position = dxlList[id]->rad2Value(radian);

    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_POSITION, position);
}

int RobotisController::getGoalPositionValue(int id, long *position)
{
    return dxlList[id]->read(dxlList[id]->ADDR_GOAL_POSITION, position);;
}

int RobotisController::setGoalPositionValue(int id, long position)
{
    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_POSITION, position);
}

int RobotisController::getGoalVelocity(int id, long *velocity)
{
    return dxlList[id]->read(dxlList[id]->ADDR_GOAL_VELOCITY, velocity);;
}

int RobotisController::setGoalVelocity(int id, long velocity)
{
    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_VELOCITY, velocity);
}

int RobotisController::getGoalTorque(int id, long *torque)
{
    return dxlList[id]->read(dxlList[id]->ADDR_GOAL_TORQUE, torque);;
}

int RobotisController::setGoalTorque(int id, long torque)
{
    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_TORQUE, torque);
}

int RobotisController::getPositionPGain(int id, int *pgain)
{
    return dxlList[id]->read(dxlList[id]->ADDR_POSITION_P_GAIN, (long*)pgain);
}

int RobotisController::setPositionPGain(int id, int pgain)
{
    return dxlList[id]->write(dxlList[id]->ADDR_POSITION_P_GAIN, pgain);
}

int RobotisController::getPositionIGain(int id, int *igain)
{
    return dxlList[id]->read(dxlList[id]->ADDR_POSITION_I_GAIN, (long*)igain);
}

int RobotisController::setPositionIGain(int id, int igain)
{
    return dxlList[id]->write(dxlList[id]->ADDR_POSITION_I_GAIN, igain);
}

int RobotisController::getPositionDGain(int id, int *dgain)
{
    return dxlList[id]->read(dxlList[id]->ADDR_POSITION_D_GAIN, (long*)dgain);
}

int RobotisController::setPositionDGain(int id, int dgain)
{
    return dxlList[id]->write(dxlList[id]->ADDR_POSITION_D_GAIN, dgain);
}

int RobotisController::getOperatingMode(int id, int *mode)
{
    return dxlList[id]->read(dxlList[id]->ADDR_OPERATING_MODE, (long*)mode);
}

int RobotisController::setOperatingMode(int id, int mode)
{
    return dxlList[id]->write(dxlList[id]->ADDR_OPERATING_MODE, mode);
}

int RobotisController::setGoalAcceleration(int id, long acceleration)
{
    return dxlList[id]->write(dxlList[id]->ADDR_GOAL_ACCELERATION, acceleration);
}

int RobotisController::isMoving(int id, bool *ismoving)
{
    long _moving;
    int result = dxlList[id]->read(dxlList[id]->ADDR_MOVING, &_moving);
    if(result == COMM_SUCCESS)
    {
        if(_moving == 0)
            *ismoving = false;
        else
            *ismoving = true;
    }
    return result;
}
