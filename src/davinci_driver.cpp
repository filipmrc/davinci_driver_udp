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
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include "libjson.h"
#include "davinci_driver/davinci_driver.h"

using namespace boost::asio;

/// Constructor for the Davinci driver
///
/// @param robot_ip The network address of the robot.
/// @param robot_port The destination port on the robot.
DavinciDriver::DavinciDriver(std::string robot_ip, unsigned short robot_port)
:
    _robot_ip(robot_ip),
    _robot_port(robot_port)
{
    std::cout << "Davinci driver" << std::endl;
    std::cout << "--------------" << std::endl << std::endl;

    std::cout << "Demonstrating JSON:" << std::endl;
    JSONNode n(JSON_NODE);
    n.push_back(JSONNode("Robot IP", _robot_ip));
    n.push_back(JSONNode("Robot Port", _robot_port));
    std::string jc = n.write_formatted();
    std::cout << jc << std::endl << std::endl;

    std::cout << "Demonstrating ASIO:" << std::endl;
    io_service io_service;
    ip::address_v4 robot_ip_t = ip::address_v4::from_string(_robot_ip);
    ip::tcp::endpoint robot_ep(robot_ip_t, _robot_port);
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

DavinciDriver::~DavinciDriver()
{}