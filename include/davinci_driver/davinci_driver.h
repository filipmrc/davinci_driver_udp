/*
Driver class for the Davinci Robot.

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

#ifndef DAVINCI_DRIVER_H
#define DAVINCI_DRIVER_H

#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "libjson.h"
#include "ros/ros.h"

class DavinciDriver
{
public:
    DavinciDriver(std::string robot_ip, unsigned short robot_port);
    ~DavinciDriver();

    void run();

    std::string state_formatted() const;

private:
    boost::asio::io_service _io_service;
    boost::asio::ip::tcp::socket _socket;

    JSONNode _state_node;
    mutable boost::mutex _state_mutex;
    JSONStream _json_stream;
    static void _update_state(JSONNode&, void*);
    static void _bad_json(void*);

    void _banner();
};

#endif
