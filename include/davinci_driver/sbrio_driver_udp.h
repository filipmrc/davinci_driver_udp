#ifndef SBRIO_DRIVER_H
#define SBRIO_DRIVER_H

#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <stdexcept>
#include <boost/array.hpp>
#include <boost/regex.hpp>


#include <iostream>
#include <fstream>


#define FLOAT_LENGTH 4 //length of a float (in bytes)
#define NUMBER_OF_JOINTS 4 //number of value sent

class SbrioDriver_UDP
{
public:
    SbrioDriver_UDP(std::string robot_ip, unsigned short robot_port);
    ~SbrioDriver_UDP();

    void connect();
    bool connected();
    bool initialized();


    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    std::vector<bool> joint_setpoints_mask;
    std::vector<double> joint_setpoints;
    std::vector<std::string> motor_names;
    std::vector<bool> motors_active;
    std::vector<bool> motors_enabled;
    boost::mutex state_mutex;
    bool new_setpoints;
    bool new_motor_enables;


private:
    boost::asio::io_service _io_service;
    boost::asio::ip::udp::endpoint _robot_ep;
    boost::asio::ip::udp::socket _socket;

    boost::thread _loop_thread;
    bool _connected;
    bool _initialized;
    int _no_packet_received;
    boost::array<char, FLOAT_LENGTH*NUMBER_OF_JOINTS*3+1> _rcv_buf;

    void _loop();
    void _start_receive();
    void _handle_receive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);

    double _binary_to_double(char* buf, int index);
    char _bool_to_char(std::vector<bool> b_vect);
    void _char_to_bool(char c,std::vector<bool> b_vect);


    //time measurement for testing:

    std::ofstream _received_packets;
    std::ofstream _sent_packets;
};

#endif
