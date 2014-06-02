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
}

DavinciDriver::~DavinciDriver()
{}

std::string DavinciDriver::state_formatted() const
{
    boost::lock_guard<boost::mutex> state_guard(_state_mutex);
    return _state_node.write_formatted();
}

void DavinciDriver::run()
{
    while (true)
    {
        boost::array<char, 128> buf;
        boost::system::error_code error;

        size_t len = _socket.read_some(boost::asio::buffer(buf), error);
        std::cout << "Got " << len << " bytes." << std::endl;
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

    boost::lock_guard<boost::mutex> state_guard(driver_pointer->_state_mutex);
    driver_pointer->_state_node = node;
}

void DavinciDriver::_bad_json(void* id)
{
    std::cout << "Bad JSON" << std::endl;
}

void DavinciDriver::_banner()
{
    std::cout << "Davinci driver" << std::endl;
    std::cout << "--------------" << std::endl;
}
