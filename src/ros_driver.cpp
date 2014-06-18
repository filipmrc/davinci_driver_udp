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
#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "davinci_driver/davinci_driver.h"

class RosDavinciDriver
{
public:
    RosDavinciDriver(std::string robot_ip, unsigned short robot_port) :
        _low_level_driver(robot_ip, robot_port)
    {
        _joint_state_publisher = _ros_nh.advertise<sensor_msgs::JointState>("joint_states", 10);

        _joint_state_timer = _ros_nh.createTimer(
            ros::Duration(0.01),
            boost::bind(&RosDavinciDriver::_publish_joint_states, this, _1)
        );

        bool retry_connection = true;
        std::string retry_string = "Connection to Davinci refused, retrying";
        while (retry_connection && ros::ok())
        {
            try
            {
                retry_connection = false;
                _low_level_driver.connect();
            }
            catch (const boost::system::system_error& e)
            {
                if (e.code() == boost::asio::error::connection_refused)
                {
                    std::cout << retry_string;
                    std::cout.flush();
                    retry_string = ".";
                    retry_connection = true;
                    ros::spinOnce();
                    ros::Duration(2.0).sleep();
                }
                else
                    throw e;
            }
        }
        std::cout << "\nConnection to Davinci established." << std::endl;

        _low_level_driver.run();
    };

    ~RosDavinciDriver()
    {

    }

private:
    DavinciDriver _low_level_driver;

    ros::NodeHandle _ros_nh;
    ros::Publisher _joint_state_publisher;
    ros::Timer _joint_state_timer;

    void _publish_joint_states(const ros::TimerEvent& event)
    {
        sensor_msgs::JointState state;
        state.header.stamp = ros::Time::now();
        state.name = _low_level_driver._joint_names;
        state.position = _low_level_driver._joint_positions;
        // _parse_json(
        //     _low_level_driver.get_state_json(),
        //     "",
        //     state
        // );
        if (state.name.size() > 0)
        {
            _joint_state_publisher.publish(state);
        }
    }

    void _parse_json(const JSONNode &n, const std::string parent_string, sensor_msgs::JointState& state)
    {
        for (JSONNode::const_iterator i = n.begin(); i != n.end(); ++i)
        {
            // The node name is concatenated with that of its parents
            // e.g. root-subnode-node
            std::string node_name = i->name();;
            if (!parent_string.empty())
            {
                node_name = parent_string + "-" + node_name;
            }

            // Recursively call ourselves to dig deeper into the tree
            if (i->type() == JSON_NODE)
            {
                _parse_json(*i, node_name, state);
            }

            // if we get an array, it is the position, velocity, effort and setpoint
            else if (i->type() == JSON_ARRAY)
            {
                state.name.push_back(node_name);
                state.position.push_back((*i)[0].as_float());
                state.velocity.push_back((*i)[1].as_float());
                state.effort.push_back((*i)[2].as_float());
            }

            // if we get a single value, it is a position.
            else
            {
                state.name.push_back(node_name);
                state.position.push_back(i->as_float());
                state.velocity.push_back(0.0);
                state.effort.push_back(0.0);
            }
        }
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "davinci_driver_node");
    
    ros::NodeHandle private_nh("~");
    ros::NodeHandle ros_nh;

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

    // Everything is in order; start the driver:
    RosDavinciDriver r(ip, atoi(port.c_str()));

    ros::spin();

    return 0;
}
