/*
ROS driver for Davinci Robot.
Main ROS node for interacting with the Davinci surgical robot at the
control lab at Aalborg university.

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

#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>

#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <controller_manager/controller_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "davinci_driver/davinci_driver.h"

class RosDavinciDriver : public hardware_interface::RobotHW
{
public:
    RosDavinciDriver(std::vector<std::pair<std::string, unsigned short> > robot_ips);
    ~RosDavinciDriver(){};

    void read(){
        _low_level_driver.read();
    };

    void write(){
        _low_level_driver.write();
    };

private:
    DavinciDriver _low_level_driver;

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::EffortJointInterface _effort_joint_interface;

    diagnostic_updater::Updater updater;
};

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


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "davinci_driver_node");

    ros::NodeHandle ros_nh;

    std::vector<std::string> input_ip_list;
    std::vector<std::pair<std::string, unsigned short> > robot_ips;
    if(! ros_nh.getParam("robot_ips", input_ip_list))
    {
        ROS_ERROR("No ip address for the robot specified.");
        return -1;
    }
    for (size_t i = 0; i < input_ip_list.size(); ++i)
    {
        boost::regex expression("((?:\\d{1,3}\\.){3}\\d{1,3}):(\\d{5})");
        boost::cmatch what;
        std::string ip, port;
        if(boost::regex_match(input_ip_list[i].c_str(), what, expression))
        {
            // what[0] contains the whole string
            // what[1] contains the ip address
            // what[2] contains the port
            ip = what[1].str();
            port = what[2].str();
            ROS_INFO("User input ip adress for Davinci robot at %s, port %s", ip.c_str(), port.c_str());
            robot_ips.push_back(std::pair<std::string, unsigned short>(ip, atoi(port.c_str())));
        }
        else
        {
            ROS_ERROR("Ip address for the robot malformed: %s", input_ip_list[i].c_str());
            return -1;
        }
    }

    // Everything is in order; start a spinner
    ros::AsyncSpinner spinner(1); // Use 1 thread for spinning.
    spinner.start();

    // and start the driver:
    RosDavinciDriver ros_driver(robot_ips);
    controller_manager::ControllerManager cm(&ros_driver);

    // Now to the business
    ros::Time last = ros::Time::now();
    ros::Rate r(100);
    while(ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros_driver.read();
        cm.update(current_time, current_time - last);
        ros_driver.write();

        last = current_time;
        r.sleep();
    }

    return 0;
}
