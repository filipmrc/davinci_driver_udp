

#include "davinci_driver/sbrio_driver_udp.h"

using namespace boost::asio;

//#define FLOAT_LENGTH 4 //length of a float (in bytes)
//#define NUMBER_OF_JOINTS 5 //number of value sent

/// Constructor for the sbRIO driver
///
/// @param robot_ip The network address of the robot.
/// @param robot_port The destination port on the robot.
SbrioDriver_UDP::SbrioDriver_UDP(std::string robot_ip, unsigned short robot_port)
:
    _socket(_io_service, ip::udp::endpoint(ip::udp::v4(), robot_port)),
    _connected(false),
    _initialized(false),
    _no_packet_received(0),
    new_setpoints(false),
    new_motor_enables(false)
{
    ip::address_v4 ip_asio_t = ip::address_v4::from_string(robot_ip);
    _robot_ep = ip::udp::endpoint(ip_asio_t, robot_port);

    _received_packets.open("/home/uurcz/received.csv");
    _sent_packets.open("/home/uurcz/sent.csv");
}


/// Destructor for the sbRIO driver
///
SbrioDriver_UDP::~SbrioDriver_UDP()
{
	//stop the asynchronous receive thread
	_io_service.stop();

	// The thread is interrupted, but if the robot is not transmitting,
	// then the loop is blocked at read_some, so we close the socket,
	// which unblocks the call and the interrrupt point is reached.
	_loop_thread.interrupt();

	if (_socket.is_open())
	{
		_socket.shutdown(ip::udp::socket::shutdown_both);
		_socket.close();
	}

	_loop_thread.join();

	std::cout << "sbrio driver shutting down" << std::endl;

	_received_packets.close();
	_sent_packets.close();
}

/// Connect to the robot
///
void SbrioDriver_UDP::connect()
{
	boost::system::error_code error;
	_socket.connect(_robot_ep, error);

	if (! error)
	{
		_loop_thread = boost::thread(&SbrioDriver_UDP::_loop, this);
		_connected = true;
		std::cout << "SbrioDriver connected to the robot\n" << std::endl;
	    	_start_receive();
		boost::thread t(boost::bind(&boost::asio::io_service::run, &_io_service)); 
	}


    //TODO: this will be handled in the davinci_driver but is needed for testing purpose
   boost::lock_guard<boost::mutex> state_guard(state_mutex);
   int i;
   const char* args[] = {"p4_hand_roll", "p4_hand_pitch", "p4_instrument_slide", "p4_instrument_roll", "p4_instrument_pitch"};
   //const char* args[] = {"p4_hand_roll", "p4_hand_pitch", "p4_instrument_roll", "p4_instrument_pitch"};
   std::vector<std::string> names_list(args,args+5);
   
   joint_names = names_list;
   for(i=0;i<NUMBER_OF_JOINTS+1;i++){
	joint_positions.push_back(0.0);
	joint_velocities.push_back(0.0);
	joint_efforts.push_back(0.0);
	joint_names.push_back(names_list[i]);
        motor_names.push_back(names_list[i]);
	joint_setpoints.push_back(0.1*i);
        joint_setpoints_mask.push_back(false);
   }

	//joint_names = names_list;

	motors_enabled.push_back(true);
	motors_enabled.push_back(true);
	motors_enabled.push_back(true);
	motors_enabled.push_back(true);
	motors_enabled.push_back(true);

	motors_active.push_back(true);
	motors_active.push_back(true);
	motors_active.push_back(true);
	motors_active.push_back(true);
	motors_active.push_back(true);

	new_motor_enables = true;
	new_setpoints = true;

        this->_initialized = true;
}

/// Check whether the sbRIO is connected.
///
bool SbrioDriver_UDP::connected()
{
    return _connected;
}

/// Check whether the sbRIO has sent its initialization message.
///
bool SbrioDriver_UDP::initialized()
{
    return _initialized;
}

/// Loop to continously monitor incomming messages from the robot.
///
void SbrioDriver_UDP::_loop()
{
	while (true)
	{
		int i,j;
		boost::array<char, NUMBER_OF_JOINTS*FLOAT_LENGTH+1> send_buf;
        	// Write something to the socket
		bool new_contents = false;
		if (new_setpoints)
		{
			boost::lock_guard<boost::mutex> state_guard(state_mutex);

			//memcpy(&send_buf, &joint_setpoints, sizeof(joint_setpoints));

			for(i=0;i<NUMBER_OF_JOINTS;i++)//joint_setpoints.size();i++){
			{	
				float f = (float)joint_setpoints[i];
				char *floatToConvert = (char*)&f;
				char convertedFloat[FLOAT_LENGTH];
				for(j=0;j<FLOAT_LENGTH;j++)
					convertedFloat[j] = floatToConvert[FLOAT_LENGTH-1-j];
				memcpy(&send_buf[i],&convertedFloat, sizeof(f));
			}
			//new_setpoints = false; //TODO this line should be uncommented for "real" operation
			new_contents = true;
		}
		if (new_motor_enables)
		{
			boost::lock_guard<boost::mutex> state_guard(state_mutex);		
			//send_buf[motor_names.size()*sizeof(float)] = _bool_to_char(motors_enabled);
			send_buf[NUMBER_OF_JOINTS*sizeof(float)] = _bool_to_char(motors_enabled);	
			
			new_motor_enables = false;
			new_contents = true;
		}

			/*for(i=0;i<send_buf.size();i++)
				std::cout << std::hex <<(int)joint_setpoints[i];
			std::cout << std::endl;*/


		if (new_contents){
			_socket.send(boost::asio::buffer(send_buf));

			struct timespec spec;
			clock_gettime(CLOCK_REALTIME, &spec);
			_sent_packets << spec.tv_sec <<"."<< spec.tv_nsec <<";\n";

		}


		//Connection timeout detection
		_no_packet_received++;
		if(_no_packet_received > 10)
		{
			/*printf("NO PACKET RECEIVED IN THE LAST %d MS",_no_packet_received);
			if(_no_packet_received > 20)
				printf(" /!!! CONNEXION TIMEOUT /!!! ");
			printf("\n");*/
		}

        boost::this_thread::sleep(boost::posix_time::millisec(100));
	}
}




///Schedule an asynchronous receive
///
void SbrioDriver_UDP::_start_receive(){
	//printf("start receive\n");
	_socket.async_receive(boost::asio::buffer(_rcv_buf), 
				boost::bind(&SbrioDriver_UDP::_handle_receive, this,
          				boost::asio::placeholders::error,
          				boost::asio::placeholders::bytes_transferred));
	
	//_socket.receive_from(boost::asio::buffer(_rcv_buf), _robot_ep);	
}

///handle the data received and schedule a new asynchronous receive
///the parameters are handled by the bind fonction in _start_receive()
///
void SbrioDriver_UDP::_handle_receive(const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/)
{
	//printf("Message received\n");
	if (!error || error == boost::asio::error::message_size)
	{
		//take time measurements
		struct timespec spec;
		clock_gettime(CLOCK_REALTIME, &spec);
		_received_packets << spec.tv_sec <<"."<< spec.tv_nsec <<";\n";


		int vector_length = FLOAT_LENGTH * NUMBER_OF_JOINTS;//motor_names.size();
		int i;

		boost::lock_guard<boost::mutex> state_guard(state_mutex);

		//connexion timeout tracking
		_no_packet_received = 0;


		//packet handling
		//update the position, velocity and effort for each motor
		for(i=0;i<NUMBER_OF_JOINTS;i++)//motor_names.size();i++)
		{	

			joint_positions[i] = _binary_to_double(&_rcv_buf[0], i*FLOAT_LENGTH);
			joint_velocities[i] = _binary_to_double(&_rcv_buf[0], i*FLOAT_LENGTH + vector_length);
			joint_efforts[i] = _binary_to_double(&_rcv_buf[0], i*FLOAT_LENGTH + vector_length*2);

		}
		//printf("\n");
		//update the motors_active vector
		_char_to_bool(_rcv_buf[vector_length*3], motors_active);
	}
	else
		std::cout<<error<<std::endl;
	 
	//schedule a new asynchronous receive
	_start_receive();
}

/// converts the binary code stored as char into a double
/// this function takes a double stored in 4 bytes (format used by the sbrio)
///
/// @param buf The char array that contains the binary code
/// @param index The index of the first byte of the number to convert
double SbrioDriver_UDP::_binary_to_double(char* buf, int index)
{
	char tmp_buf[4];
	tmp_buf[3] = buf[index+0];
	tmp_buf[2] = buf[index+1];
	tmp_buf[1] = buf[index+2];
	tmp_buf[0] = buf[index+3];
		/*int i; //affiche le tmp_buf en binaire
		//printf("length: %zu\n",strlen(tmp_buf));
		for(i=0;i<4;i++){ std::bitset<8> x(tmp_buf[i]); std::cout << x << "\t" ;}
printf("\n");

	for(int i=0; i<4; ++i)
			printf("%x\t", tmp_buf[i] & 0xff);
	printf("\n");
		*/
	float f;
	memcpy(&f, &tmp_buf, sizeof(f));


	//printf("float: %f\n",f);
	return static_cast<double>(f);
}

///converts a vector of up to 8 boolean into a char
///
/// @param b_vect The vector of boolean to convert
char SbrioDriver_UDP::_bool_to_char(std::vector<bool> b_vect)
{
	int i;
	char c = 0;

	for(i=0;i<b_vect.size();i++) c = (c << 1) + b_vect[i];

	printf("%x\n", c & 0xff);
	return c;
}


///converts a char into a vector of booleans
///
/// @param c The char that contain the boolean as bits
/// @param b_vect The vector of booleans to update
void SbrioDriver_UDP::_char_to_bool(char c,std::vector<bool> b_vect)
{
	int i;
	for(i=0;i<b_vect.size();i++)
	{
		b_vect[i] = c%2;
		c = (c >> 1);
	}
}
