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
/// @param robot_ips A list of pairs of network addresses and ports of the embedded controllers on the robot.
DavinciDriver::DavinciDriver(std::vector<std::pair<std::string, unsigned short> > robot_ips) :
    _all_initialized(false)
{
    std::vector<std::pair<std::string, unsigned short> >::iterator i;
    for (i = robot_ips.begin(); i != robot_ips.end(); ++i)
    {
        boost::shared_ptr<SbrioDriver_UDP> p(new SbrioDriver_UDP(i->first, i->second));
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
        // then initialize the state vectors.
        if (_all_initialized)
        {
            for (size_t i = 0; i < _sbRioDrivers.size(); ++i)
            {
                boost::lock_guard<boost::mutex> state_guard(_sbRioDrivers[i]->state_mutex);
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

/// Get all names of the joints on the robot.
///
/// The order of the names is the same as that of the joint_positions,
/// joint_velocities, joint_efforts, and joint_setpoints vectors.
///
/// @returns The names of the joints.
std::vector<std::string> DavinciDriver::get_joint_names() const
{
    if (! initialized())
        throw std::runtime_error("Attemt to get joint names before driver is initialized.");

    std::vector<std::string> joint_names;
    for (size_t sbrio_i = 0; sbrio_i < _sbRioDrivers.size(); ++sbrio_i)
    {
        for (size_t joint_i = 0; joint_i < _sbRioDrivers[sbrio_i]->joint_names.size(); ++joint_i)
        {
            joint_names.push_back(_sbRioDrivers[sbrio_i]->joint_names[joint_i]);
        }
    }

    return joint_names;
}

/// Get all names of the motors on the robot.
///
/// These are the names that enable_motor takes as argument.
///
/// @returns The names of the motors.
std::vector<std::string> DavinciDriver::get_motor_names() const
{
    if (! initialized())
        throw std::runtime_error("Attemt to get motor names before driver is initialized.");

    std::vector<std::string> motor_names;
    for (size_t sbrio_i = 0; sbrio_i < _sbRioDrivers.size(); ++sbrio_i)
    {
        for (size_t motor_i = 0; motor_i < _sbRioDrivers[sbrio_i]->motor_names.size(); ++motor_i)
        {
            motor_names.push_back(_sbRioDrivers[sbrio_i]->motor_names[motor_i]);
        }
    }

    return motor_names;
}

/// Get names of the active motors on the robot.
///
/// The order of the names is the same as that of the motors_active vector.
/// It is also the names that enable_motor takes as argument.
///
/// @returns The names of the active motors.
std::vector<std::string> DavinciDriver::get_active_motors() const
{
    if (! initialized())
        throw std::runtime_error("Attemt to get active motor names before driver is initialized.");

    std::vector<std::string> act_mts;
    for (size_t sbrio_i = 0; sbrio_i < _sbRioDrivers.size(); ++sbrio_i)
    {
        for (size_t motor_i = 0; motor_i < _sbRioDrivers[sbrio_i]->motor_names.size(); ++motor_i)
        {
            if (_sbRioDrivers[sbrio_i]->motors_active[motor_i])
                act_mts.push_back(_sbRioDrivers[sbrio_i]->motor_names[motor_i]);
        }
    }

    return act_mts;
}

/// Get a vector of the active motors on the robot.
///
/// The structure of the names is the same as the output of get_motor_names.
///
/// @returns A vector indicating if motors are active.
std::vector<bool> DavinciDriver::get_active_motors_vector() const
{
    if (! initialized())
        throw std::runtime_error("Attemt to get active motor states before driver is initialized.");

    std::vector<bool> act_mts;
    for (size_t sbrio_i = 0; sbrio_i < _sbRioDrivers.size(); ++sbrio_i)
    {
        act_mts.insert(act_mts.end(),
                       _sbRioDrivers[sbrio_i]->motors_active.begin(),
                       _sbRioDrivers[sbrio_i]->motors_active.end());
    }

    return act_mts;
}

/// Get names of the enabled motors on the robot.
///
/// The order of the names is the same as that of the motors_active vector.
/// It is also the names that enable_motor takes as argument.
///
/// @returns The names of the active motors.
std::vector<std::string> DavinciDriver::get_enabled_motors() const
{
    if (! initialized())
        throw std::runtime_error("Attemt to get enabled motor names before driver is initialized.");

    std::vector<std::string> en_mts;
    for (size_t sbrio_i = 0; sbrio_i < _sbRioDrivers.size(); ++sbrio_i)
    {
        for (size_t motor_i = 0; motor_i < _sbRioDrivers[sbrio_i]->motor_names.size(); ++motor_i)
        {
            if (_sbRioDrivers[sbrio_i]->motors_enabled[motor_i])
                en_mts.push_back(_sbRioDrivers[sbrio_i]->motor_names[motor_i]);
        }
    }

    return en_mts;
}

/// Enable or disable motor driver outputs
///
/// @param motor_name Name of the motor. Use get_motor_names to get a list of motors.
/// @param enable True to enable output, false to disable.
void DavinciDriver::enable_motor(std::string motor_name, bool enable)
{
    if (! initialized())
        throw std::runtime_error("Attemt to enable motor before driver is initialized.");

    // Loop through the motor names of the sbRIOs
    bool didnt_find_it = true;
    for (size_t sbrio_i = 0; sbrio_i < _sbRioDrivers.size(); ++sbrio_i)
    {
        for (size_t motor_i = 0; motor_i < _sbRioDrivers[sbrio_i]->motor_names.size(); ++motor_i)
        {
            if (motor_name == _sbRioDrivers[sbrio_i]->motor_names[motor_i])
            {
                boost::lock_guard<boost::mutex> state_guard(_sbRioDrivers[sbrio_i]->state_mutex);
                _sbRioDrivers[sbrio_i]->motors_enabled[motor_i] = enable;
                _sbRioDrivers[sbrio_i]->new_motor_enables = true;
                didnt_find_it = false;
                break;
            }
        }
    }

    if (didnt_find_it)
        throw std::runtime_error("No motor \"" + motor_name + "\" connected");
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



	    /*std::cout << "positions: ";
	    for(int j=0;j<joint_positions.size();j++) std::cout<<joint_positions[j] << "\t";
	    std::cout << std::endl;

	    std::cout << "velocity: ";
	    for(int j=0;j<joint_velocities.size();j++) std::cout<<joint_velocities[j] << "\t";
	    std::cout << std::endl;

	    std::cout << "effort: ";
	    for(int j=0;j<joint_efforts.size();j++) std::cout<<joint_efforts[j] << "\t";
	    std::cout << std::endl;*/
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
        std::vector<double>::iterator set_iter = joint_positions.begin();
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
