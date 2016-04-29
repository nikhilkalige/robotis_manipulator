/*
 * ur_driver
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MANIPULATOR_DRIVER_H_
#define MANIPULATOR_DRIVER_H_

#include <mutex>
#include <condition_variable>
#include <vector>
#include <math.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "RobotisController.h"
#include "handler/GroupHandler.h"
#include <chrono>


class ManipulatorDriver {
public:
    Robotis::RobotisController* controller_;
    Robotis::GroupHandler* grp_handler_;
    std::mutex* com_lock_;

    ManipulatorDriver(Robotis::RobotisController* controller, Robotis::GroupHandler* group_handler, std::mutex* com_mutex);
    bool start();
    void initiate_read();
    std::vector<double> get_position();
    std::vector<double> get_velocity();
    void write(int addr, int data_length, std::vector<unsigned char> param);
    void write_position(std::vector<double> positions);
    void write_velocity(std::vector<double> velocities);
};

#endif /* MANIPULATOR_DRIVER_H_ */
