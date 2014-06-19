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
#include <stdexcept>
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
    ip::address_v4 ip_asio_t = ip::address_v4::from_string(robot_ip);
    _robot_ep = ip::tcp::endpoint(ip_asio_t, robot_port);
}

DavinciDriver::~DavinciDriver()
{
    _loop_thread.interrupt();

    if (_socket.is_open())
    {
        _socket.shutdown(ip::tcp::socket::shutdown_both);
        _socket.close();
    }

    _loop_thread.join();
}

void DavinciDriver::connect()
{
    boost::system::error_code error;
    _socket.connect(_robot_ep, error);
    if (error)
        throw boost::system::system_error(error);
}

void DavinciDriver::run()
{
    std::cout << "Entering run()" << std::endl;
    if (! _socket.is_open())
        throw std::logic_error("Not connected to Davinci.");

    _loop_thread = boost::thread(&DavinciDriver::_loop, this);
    std::cout << "Exiting run()" << std::endl;
}

void DavinciDriver::_update_state(JSONNode& node, void* id)
{
    std::cout << "Entering _update_state" << std::endl;
    std::cout << node.write_formatted() << std::endl;

    DavinciDriver * driver_pointer = static_cast<DavinciDriver*>(id);

    boost::lock_guard<boost::mutex> state_guard(driver_pointer->_state_mutex);

    // We got at node, get the name of the root node
    JSONNode::const_iterator root_iter = node.begin();
    std::string root_name = root_iter->name();

    // ends with -name
    if (0 == root_name.compare(root_name.length() - 5, 5, "-name"))
    {
        std::string root_prefix = root_name.substr(0, root_name.length() - 5);
        for(JSONNode::const_iterator name_array = root_iter->begin(); name_array != root_iter->end(); ++name_array)
        {
            std::string joint_name = root_prefix + "-" + name_array->as_string();
            bool name_exists = false;
            for(size_t i; i < driver_pointer->_joint_names.size(); ++i)
            {
                if (joint_name.compare(driver_pointer->_joint_names[i]) == 0)
                {
                    name_exists = true;
                    break;
                }
            }
            if (!name_exists)
            {
                driver_pointer->_joint_names.push_back(joint_name);
            }
        }
    }

    // ends with -position
    if (0 == root_name.compare(root_name.length() - 4, 4, "-pos"))
    {
        driver_pointer->_joint_positions.clear();
        for(JSONNode::const_iterator pos_array = root_iter->begin(); pos_array != root_iter->end(); ++pos_array)
        {
            double joint_position = pos_array->as_float();
            driver_pointer->_joint_positions.push_back(joint_position);
        }
    }

    std::cout << "Exiting _update_state" << std::endl;
}

void DavinciDriver::_bad_json(void* id)
{
    std::cout << "Bad JSON" << std::endl;
}

void DavinciDriver::_loop()
{
    while (true)
    {
        // If the caller wants to stop executing, this thread stops here.
        boost::this_thread::interruption_point();

        // Read something from the socket.
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
