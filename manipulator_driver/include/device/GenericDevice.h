/*
 *=====================================================
 * File   :  GenericDevice.h
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

#ifndef GENERICDEVICE_H_
#define GENERICDEVICE_H_

#include <map>
#include <string>

#include "../handler/PacketHandler.h"

#define PI 3.14159265

namespace Robotis
{

class GenericDevice
{
protected:
    const long      MIN_VALUE;
    const long      MAX_VALUE;
    const long      CENTER_VALUE;
    const double    MIN_RADIAN;
    const double    MAX_RADIAN;
    const unsigned int GEAR_RATIO;

    std::map<int, int> addr_length;

    char            jointName[40];
    PortHandler     *comPort;
    PacketHandler   *packetHandler;

    GenericDevice(PortHandler *port, long min_value, long max_value, long center_value, double min_radian, double max_radian, unsigned int gear_ratio);

public:
    int             ID;
    float           PROTOCOL_VERSION;   // Dynamixel Protocol Version

    int             ADDR_MODEL_NUMBER;
    int             ADDR_FIRMWARE_VERSION;
    int             ADDR_ID;
    int             ADDR_BAUD_RATE;
    int             ADDR_RETURN_DELAY_TIME;
    int             ADDR_RETURN_LEVEL;
    int             ADDR_MIN_POSITION_LIMIT;
    int             ADDR_MAX_POSITION_LIMIT;
    int             ADDR_TORQUE_ENABLE;
    int             ADDR_POSITION_D_GAIN;
    int             ADDR_POSITION_I_GAIN;
    int             ADDR_POSITION_P_GAIN;
    int             ADDR_GOAL_POSITION;
    int             ADDR_GOAL_VELOCITY;
    int             ADDR_GOAL_TORQUE;
    int             ADDR_PRESENT_POSITION;
    int             ADDR_PRESENT_VELOCITY;
    int             ADDR_PRESENT_LOAD;
    int             ADDR_MOVING;
    int             ADDR_OPERATING_MODE;
    int             ADDR_GOAL_ACCELERATION;

    virtual ~GenericDevice() { }

    virtual long rad2Value(double radian)   = 0;
    virtual double value2Rad(long value)    = 0;

    virtual long velocity2Value(double vel) = 0;
    virtual double value2Velocity(long value) = 0;

    static GenericDevice *getInstance(PortHandler *port, int id, const char *joint_name, const char *model, float protocol_ver = 2.0);

    char*   getJointName();
    PortHandler *getSerialPort();

    int     getAddrLength(int addr);

    int     read(int address, long *data, int *error = 0);
    int     read(int address, long *data, LENGTH_TYPE length, int *error = 0);
    int     write(int address, long data, int *error = 0);
    int     write(int address, long data, LENGTH_TYPE length, int *error = 0);
};


}

#endif /* GENERICDEVICE_H_ */
