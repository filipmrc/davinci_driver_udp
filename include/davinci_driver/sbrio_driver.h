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

#ifndef SBRIO_DRIVER_H
#define SBRIO_DRIVER_H

#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "libjson.h"

class SbrioDriver
{
public:
    SbrioDriver(std::string robot_ip, unsigned short robot_port);
    ~SbrioDriver();

    void connect();
    bool connected();
    bool initialized();

    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    std::vector<double> joint_setpoints;
    std::vector<std::string> motor_names;
    std::vector<bool> motors_active;
    std::vector<bool> motors_enabled;
    boost::mutex state_mutex;
    bool new_setpoints;
    bool new_motor_enables;

private:
    boost::asio::io_service _io_service;
    boost::asio::ip::tcp::endpoint _robot_ep;
    boost::asio::ip::tcp::socket _socket;

    boost::thread _loop_thread;
    JSONStream _json_stream;
    bool _connected;
    bool _initialized;

    void _loop();
    static void _update_state(JSONNode&, void*);
    static void _bad_json(void*);
    bool _got_bad_json;

    std::string _root_header;
};

#endif