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

#include <algorithm>
#include <stdexcept>
#include <boost/thread.hpp>
#include "davinci_driver/davinci_driver.h"

using namespace boost::asio;

/// Constructor for the Davinci driver
///
/// This driver congregates the sub-drivers for the sbRIOs on the Davinci
/// robot. It replicates the state of the sub-drivers, so that the client
/// does not need to acquire mutexes. However the driver needs to be
/// explicitly updated.
///
/// @param robot_ip The network address of the robot.
/// @param robot_port The destination port on the robot.
DavinciDriver::DavinciDriver(std::vector<std::pair<std::string, unsigned short> > robot_ips) :
    _all_initialized(false)
{
    std::vector<std::pair<std::string, unsigned short> >::iterator i;
    for (i = robot_ips.begin(); i != robot_ips.end(); ++i)
    {
        boost::shared_ptr<SbrioDriver> p(new SbrioDriver(i->first, i->second));
        _sbRioDrivers.push_back(p);
    }
}

DavinciDriver::~DavinciDriver()
{
}

/// Connect to each of the controllers on the robot.
///
/// This call is blocking.
void DavinciDriver::connect()
{
    bool all_connected = false;
    while (! all_connected)
    {
        all_connected = true;
        for (size_t i = 0; i < _sbRioDrivers.size(); ++i)
        {
            if (! _sbRioDrivers[i]->connected())
                _sbRioDrivers[i]->connect();
            if (! _sbRioDrivers[i]->connected())
                all_connected = false;
        }
        if (! all_connected)
        {
            boost::this_thread::sleep(boost::posix_time::seconds(0.1));
        }
    }

    while (! _all_initialized)
    {
        _all_initialized = true;
        for (size_t d = 0; d < _sbRioDrivers.size(); ++d)
        {
            if (! _sbRioDrivers[d]->initialized())
                _all_initialized = false;
        }

        // If this is the time where all sbRIOs have been initialized
        // then get the names and initialize the state vectors.
        if (_all_initialized)
        {
            for (size_t i = 0; i < _sbRioDrivers.size(); ++i)
            {
                boost::lock_guard<boost::mutex> state_guard(_sbRioDrivers[i]->state_mutex);
                //Names
                std::vector<std::string>::const_iterator start_str = _sbRioDrivers[i]->joint_names.begin();
                std::vector<std::string>::const_iterator stop_str = _sbRioDrivers[i]->joint_names.end();
                _joint_names.insert(_joint_names.end(), start_str, stop_str);
                _offset_names.push_back(_joint_names.end());
                //Positions
                std::vector<double>::const_iterator start = _sbRioDrivers[i]->joint_positions.begin();
                std::vector<double>::const_iterator stop = _sbRioDrivers[i]->joint_positions.end();
                joint_positions.insert(joint_positions.end(), start, stop);
                //Velocities
                start = _sbRioDrivers[i]->joint_velocities.begin();
                stop = _sbRioDrivers[i]->joint_velocities.end();
                joint_velocities.insert(joint_velocities.end(), start, stop);
                //Efforts
                start = _sbRioDrivers[i]->joint_efforts.begin();
                stop = _sbRioDrivers[i]->joint_efforts.end();
                joint_efforts.insert(joint_efforts.end(), start, stop);
                //Setpoints
                start = _sbRioDrivers[i]->joint_setpoints.begin();
                stop = _sbRioDrivers[i]->joint_setpoints.end();
                joint_setpoints.insert(joint_setpoints.end(), start, stop);
            }
        }

        if (! _all_initialized)
        {
            boost::this_thread::sleep(boost::posix_time::seconds(0.1));
        }
    }
}

/// Check to see if the robot has sent all initialization messages.
///
/// @returns true if all messages has been received.
bool DavinciDriver::initialized() const
{
    return _all_initialized;
}


std::vector<std::string> DavinciDriver::get_names() const
{
    if (! initialized())
        throw std::runtime_error("Attemt to get names before driver is initialized.");

    return _joint_names;
}

void DavinciDriver::read()
{
    if (_all_initialized)
    {
        std::vector<double>::iterator pos_iter = joint_positions.begin();
        std::vector<double>::iterator vel_iter = joint_velocities.begin();
        std::vector<double>::iterator eff_iter = joint_efforts.begin();
        for (size_t i = 0; i < _sbRioDrivers.size(); ++i)
        {
            boost::lock_guard<boost::mutex> state_guard(_sbRioDrivers[i]->state_mutex);
            //Positions
            std::vector<double>::const_iterator start = _sbRioDrivers[i]->joint_positions.begin();
            std::vector<double>::const_iterator stop = _sbRioDrivers[i]->joint_positions.end();
            pos_iter = std::copy(start, stop, pos_iter);
            //Velocities
            start = _sbRioDrivers[i]->joint_velocities.begin();
            stop = _sbRioDrivers[i]->joint_velocities.end();
            vel_iter = std::copy(start, stop, vel_iter);
            //Efforts
            start = _sbRioDrivers[i]->joint_efforts.begin();
            stop = _sbRioDrivers[i]->joint_efforts.end();
            eff_iter = std::copy(start, stop, eff_iter);
        }
    }
    else
    {
        throw std::runtime_error("Attemt to read state before driver is initialized.");
    }
}

void DavinciDriver::write()
{
    if (_all_initialized)
    {
        std::vector<double>::iterator set_iter = joint_efforts.begin();
        for (size_t i = 0; i < _sbRioDrivers.size(); ++i)
        {
            boost::lock_guard<boost::mutex> state_guard(_sbRioDrivers[i]->state_mutex);
            size_t num_setpoints = _sbRioDrivers[i]->joint_setpoints.size();
            _sbRioDrivers[i]->joint_setpoints.assign(set_iter, set_iter + num_setpoints);
            _sbRioDrivers[i]->new_setpoints = true;
            set_iter += num_setpoints;
        }
    }
    else
    {
        throw std::runtime_error("Attemt to write state before driver is initialized.");
    }
}
