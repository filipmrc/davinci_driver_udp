/*
Driver class for the single board RIO's on the Davinci Robot.

Copyright (C) 2014 Aalborg University - Karl D. Hansen (kdh@es.aau.dk)

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

#include <stdexcept>
#include <boost/array.hpp>
#include <boost/regex.hpp>
#include "davinci_driver/sbrio_driver.h"

using namespace boost::asio;

/// Constructor for the sbRIO driver
///
/// @param robot_ip The network address of the robot.
/// @param robot_port The destination port on the robot.
SbrioDriver::SbrioDriver(std::string robot_ip, unsigned short robot_port)
:
    _socket(_io_service),
    _json_stream(
        _update_state,
        _bad_json,
        this
    ),
    _got_bad_json(false),
    _connected(false),
    _initialized(false),
    new_setpoints(false)
{
    ip::address_v4 ip_asio_t = ip::address_v4::from_string(robot_ip);
    _robot_ep = ip::tcp::endpoint(ip_asio_t, robot_port);
}

/// Destructor for the sbRIO driver
///
SbrioDriver::~SbrioDriver()
{
    // The thread is interrupted, but if the robot is not transmitting,
    // then the loop is blocked at read_some, so we close the socket,
    // which unblocks the call and the interrrupt point is reached.
    _loop_thread.interrupt();

    if (_socket.is_open())
    {
        _socket.shutdown(ip::tcp::socket::shutdown_both);
        _socket.close();
    }

    _loop_thread.join();

    std::cout << "sbrio driver shuttting down" << std::endl;
}

/// Connect to the robot
///
void SbrioDriver::connect()
{
    boost::system::error_code error;
    _socket.connect(_robot_ep, error);
    if (! error)
    {
        _loop_thread = boost::thread(&SbrioDriver::_loop, this);
        _connected = true;
    }
}

/// Check whether the sbRIO is connected.
///
bool SbrioDriver::connected()
{
    return _connected;
}

/// Check whether the sbRIO has sent its initialization message.
///
bool SbrioDriver::initialized()
{
    return _initialized;
}

/// Loop to continously monitor incomming messages from the robot.
///
void SbrioDriver::_loop()
{
    while (true)
    {
        // Read something from the socket.
        size_t bytes_available = _socket.available();
        if (bytes_available != 0)
        {
            boost::array<char, 1500> buf;
            boost::system::error_code error;
            size_t len = _socket.read_some(boost::asio::buffer(buf), error);
            if (error == boost::asio::error::eof)
                break; // Connection closed cleanly by peer.
            else if (error)
                throw boost::system::system_error(error); // Some other error.

            std::string incomming_json(buf.data(), len);
            _json_stream << incomming_json;
            if (_got_bad_json) // this is being set by the bad_json handler
            {
                _json_stream.reset();
                _got_bad_json = false;
            }
        }

        // Write something to the socket
        if (new_setpoints)
        {
            boost::lock_guard<boost::mutex> state_guard(state_mutex);

            JSONNode setpoint_array(JSON_ARRAY);
            setpoint_array.set_name("setpoints");
            for(std::vector<double>::const_iterator i = joint_setpoints.begin(); i != joint_setpoints.end(); ++i)
            {
                setpoint_array.push_back(JSONNode("", *i));
            }

            JSONNode root_node(JSON_NODE);
            root_node.push_back(setpoint_array);
            std::string send_string = root_node.write() + "\r\n";
            boost::asio::write(_socket, boost::asio::buffer(send_string, send_string.size()));

            new_setpoints = false;
        }

        // If the caller wants to stop executing, this thread stops here.
        boost::this_thread::sleep(boost::posix_time::millisec(1));
    }
}

/// Callback for the receiving loop when receiving valid json
///
void SbrioDriver::_update_state(JSONNode& node, void* id)
{
    SbrioDriver * driver_pointer = static_cast<SbrioDriver*>(id);

    // We got a node, get the name of the root node
    JSONNode::const_iterator root_iter = node.begin();
    // Maybe there is no root.
    if (root_iter == node.end())
        return;

    std::string root_name = root_iter->name();
    boost::regex expression("(.*)-names");
    boost::smatch regex_result;

    if(boost::regex_match(root_name, regex_result, expression)) // It is a initialization message.
    {
        // If the prefix has already reported its names, we wil disregard it.
        if (! driver_pointer->initialized())
        {
            boost::lock_guard<boost::mutex> state_guard(driver_pointer->state_mutex);
            for(JSONNode::const_iterator name_array = root_iter->begin(); name_array != root_iter->end(); ++name_array)
            {
                std::string joint_name = name_array->as_string();
                driver_pointer->joint_names.push_back(joint_name);
                driver_pointer->joint_positions.push_back(0.0);
                driver_pointer->joint_velocities.push_back(0.0);
                driver_pointer->joint_efforts.push_back(0.0);
                driver_pointer->joint_setpoints.push_back(0.0);
            }
            driver_pointer->_initialized = true;
        }
    }
    else // it is a state update.
    {
        boost::lock_guard<boost::mutex> state_guard(driver_pointer->state_mutex);
        size_t i = 0;
        for(JSONNode::const_iterator pos_array = root_iter->begin(); pos_array != root_iter->end(); ++pos_array)
        {
            driver_pointer->joint_positions[i] = pos_array->as_float();
            i += 1;
        }
    }
}

/// Callback for when receiving bad json.
///
void SbrioDriver::_bad_json(void* id)
{
    std::cout << "Bad JSON" << std::endl;
    SbrioDriver * driver_pointer = static_cast<SbrioDriver*>(id);
    driver_pointer->_got_bad_json = true;
}
