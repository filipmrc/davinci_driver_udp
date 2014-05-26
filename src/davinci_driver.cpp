/*
Driver class for the Davinci Robot.
This is the business code for the ROS node for interacting with the
Davinci surgical robot at the control lab at Aalborg university.

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

#include <iostream>
#include <sstream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include "sensor_msgs/JointState.h"
#include "davinci_driver/davinci_driver.h"

using namespace boost::asio;

/// Constructor for the Davinci driver
///
/// @param robot_ip The network address of the robot.
/// @param robot_port The destination port on the robot.
DavinciDriver::DavinciDriver(std::string robot_ip, unsigned short robot_port)
:
    _socket(_io_service),
    _json_stream(
        _update_state,
        _bad_json,
        this
    )
{
    _banner();
    
    // Connect to the robot
    ip::address_v4 ip_asio_t = ip::address_v4::from_string(robot_ip);
    ip::tcp::endpoint robot_ep(ip_asio_t, robot_port);
    _socket.connect(robot_ep);

    // Set up state publisher
    _joint_state_publisher = _ros_nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Rate r(10);
    _joint_state_timer = _ros_nh.createTimer(
        r,
        &DavinciDriver::_publish_joint_states,
        this,
        false,
        false
    );
}

DavinciDriver::~DavinciDriver()
{}

void DavinciDriver::run()
{
    _joint_state_timer.start();
    _receive_loop();
    _joint_state_timer.stop();

    ROS_INFO("Connection to Davinci terminated.");
}

void DavinciDriver::_publish_joint_states(const ros::TimerEvent& timer_event)
{
    sensor_msgs::JointState message;
    message.name.push_back("karl");
    message.position.push_back(2.54);
    _joint_state_publisher.publish(message);
}

void DavinciDriver::_receive_loop()
{
    while (true)
    {
        boost::array<char, 128> buf;
        boost::system::error_code error;

        size_t len = _socket.read_some(boost::asio::buffer(buf), error);
        if (error == boost::asio::error::eof)
            break; // Connection closed cleanly by peer.
        else if (error)
            throw boost::system::system_error(error); // Some other error.

        std::string incomming_json(buf.data(), len);
        _json_stream << incomming_json;
    }
}

void DavinciDriver::_update_state(JSONNode& node, void* id)
{
    DavinciDriver * driver_pointer = static_cast<DavinciDriver*>(id);

    for (JSONNode::const_iterator i = node.begin(); i != node.end(); ++i)
    {
        std::cout << i->name() << std::endl;
    }
}

void DavinciDriver::_bad_json(void* id)
{
    std::cout << "Bad JSON" << std::endl;
}

void DavinciDriver::_banner()
{
    std::cout << "Davinci driver" << std::endl;
    std::cout << "--------------" << std::endl << std::endl;
}

void DavinciDriver::_demo_json()
{
    std::cout << "Demonstrating JSON:" << std::endl;
    JSONNode n(JSON_NODE);
    n.push_back(JSONNode("Robot IP", _socket.remote_endpoint().address().to_string()));
    n.push_back(JSONNode("Robot Port", _socket.remote_endpoint().port()));
    std::string jc = n.write_formatted();
    std::cout << jc << std::endl << std::endl;
}

void DavinciDriver::_demo_asio()
{
    std::cout << "Demonstrating ASIO:" << std::endl;
    io_service io_service;
    ip::address_v4 robot_ip_t = ip::address_v4::from_string("127.0.0.1");
    ip::tcp::endpoint robot_ep(robot_ip_t, 41229);
    ip::tcp::socket socket(io_service);
    socket.connect(robot_ep);
    for (;;)
    {
      boost::array<char, 128> buf;
      boost::system::error_code error;

      size_t len = socket.read_some(boost::asio::buffer(buf), error);
      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.

      std::cout.write(buf.data(), len);
    }
    std::cout << "Connection closed..." << std::endl;
}