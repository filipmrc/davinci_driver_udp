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

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "davinci_driver/sbrio_driver.h"

class DavinciDriver
{
public:
    DavinciDriver(std::vector<std::pair<std::string, unsigned short> > robot_ips);
    ~DavinciDriver();

    void connect();
    bool initialized() const;

    void read();
    void write();

    std::vector<std::string> get_joint_names() const;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    std::vector<double> joint_setpoints;

    std::vector<std::string> get_motor_names() const;
    std::vector<std::string> get_active_motors() const;
    std::vector<bool> get_active_motors_vector() const;
    std::vector<std::string> get_enabled_motors() const;
    void enable_motor(std::string motor_name, bool enable);

private:
    std::vector<boost::shared_ptr<SbrioDriver> > _sbRioDrivers;

    mutable bool _all_initialized;

    std::vector<std::string> _joint_names;
    std::vector<std::vector<std::string>::iterator> _offset_names;
    std::vector<std::vector<double>::iterator> _offset_positions;
    std::vector<std::vector<double>::iterator> _offset_velocities;
    std::vector<std::vector<double>::iterator> _offset_efforts;
    std::vector<std::vector<double>::iterator> _offset_setpoints;
};

#endif
