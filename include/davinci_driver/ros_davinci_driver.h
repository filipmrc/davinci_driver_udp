/*
Driver class for the Davinci Robot with ros_control specifics.

Copyright (C) 2014 Karl D. Hansen (kdh@es.aau.dk)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ROS_DAVINCI_DRIVER_H
#define ROS_DAVINCI_DRIVER_H

#include "ros/ros.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "davinci_driver/davinci_driver.h"


class RosDavinciDriver : public hardware_interface::RobotHW
{
public:
    RosDavinciDriver(std::vector<std::pair<std::string, unsigned short> > robot_ips);
    ~RosDavinciDriver();

    void read();
    void write();

private:
    DavinciDriver _low_level_driver;

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::EffortJointInterface _effort_joint_interface;
};

#endif
