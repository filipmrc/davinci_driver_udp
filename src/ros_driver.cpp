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
#include <string>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "davinci_driver/davinci_driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "davinci_driver_node");
    
    ros::NodeHandle private_nh("~");

    std::string robot_ip_input;
    if (! private_nh.getParam("robot_ip", robot_ip_input))
    {
        ROS_ERROR("No ip address for the robot specified.");
        return -1;
    }

    boost::regex expression("((?:\\d{1,3}\\.){3}\\d{1,3}):(\\d{5})");
    boost::cmatch what;
    std::string ip, port;
    if(boost::regex_match(robot_ip_input.c_str(), what, expression))
    {
        // what[0] contains the whole string 
        // what[1] contains the ip address 
        // what[2] contains the port
        ip = what[1].str();
        port = what[2].str();
        ROS_INFO("User input ip adress for Davinci robot at %s, port %s", ip.c_str(), port.c_str());
    }
    else
    {
        ROS_ERROR("Ip address for the robot malformed: %s", robot_ip_input.c_str());
        return -1;
    }

    DavinciDriver d(ip, atoi(port.c_str()));
    boost::thread driver_thread(&DavinciDriver::run, &d);
    ros::spin();

    return 0;
}