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

#include <boost/thread.hpp>
#include "ros/console.h"
#include "davinci_driver/ros_davinci_driver.h"

RosDavinciDriver::RosDavinciDriver(std::vector<std::pair<std::string, unsigned short> > robot_ips) :
    _low_level_driver(robot_ips)
{
    ROS_INFO("Connecting to Davinci Robot...");
    boost::thread connect_thread(boost::bind(&DavinciDriver::connect, &_low_level_driver));
    // Wait for connection or termination
    while(ros::ok() && connect_thread.joinable())
    {
        connect_thread.timed_join(boost::posix_time::seconds(0.1));
    }
    // if we were terminated while waiting for connection
    if (! ros::ok())
    {
        connect_thread.interrupt();
        connect_thread.join();
        return;
    }
    ROS_INFO("Connection to Davinci established.");

    std::vector<std::string> joint_names = _low_level_driver.get_names();
    for(size_t i = 0; i < joint_names.size(); ++i)
    {
        hardware_interface::JointStateHandle state_handle(
            joint_names[i],
            &_low_level_driver.joint_positions[i],
            &_low_level_driver.joint_velocities[i],
            &_low_level_driver.joint_efforts[i]
        );
        _joint_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle effort_handle(
            state_handle,
            &_low_level_driver.joint_efforts[i]
        );
        _effort_joint_interface.registerHandle(effort_handle);
    }
    registerInterface(&_joint_state_interface);
    registerInterface(&_effort_joint_interface);

    ROS_INFO("Davinci driver initialized.");
}

RosDavinciDriver::~RosDavinciDriver()
{}

void RosDavinciDriver::read()
{
    _low_level_driver.read();
}

void RosDavinciDriver::write()
{
    _low_level_driver.write();
}