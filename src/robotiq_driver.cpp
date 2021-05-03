/*!
  \file		robotiq_driver.cpp
  \author	Toshio UESHIBA
*/
#include <ros/ros.h>
#include <aist_robotiq/robotiq_driver.h>
#include <controller_manager/controller_manager.h>
#include <sys/socket.h>
#include <netinet/in.h>		// for struct sockaddr_in
#include <arpa/inet.h>		// for inet_addr()
#include <netdb.h>		// for struct hostent, gethostbyname()
#include <errno.h>
#include <fstream>

namespace aist_robotiq
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> constexpr const T&
clamp(const T& v, const T& low, const T& high)
{
    return (v < low ? low : v > high ? high : v);
}

/************************************************************************
*  class robotiq_driver_base						*
************************************************************************/
robotiq_driver_base::robotiq_driver_base(const ros::NodeHandle& nh)
    :_nh(nh),
     _period(1.0/_nh.param<double>("publish_rate", 100.0)),
     _min_pos_counts(_nh.param<int>("min_position_counts", 224)),
     _min_pos(_nh.param<double>("min_position", 0.000)),
     _max_pos(_nh.param<double>("max_position", 0.085)),
     _min_vel(_nh.param<double>("min_velocity", 0.013)),
     _max_vel(_nh.param<double>("max_velocity", 0.100)),
     _min_eff(_nh.param<double>("min_effort",   40.0)),
     _max_eff(_nh.param<double>("max_effort",   100.0)),
     _pos(0),
     _vel(0),
     _eff(0),
     _cmd_pos(0)
{
    using JointStateHandle = hardware_interface::JointStateHandle;
    using JointHandle	   = hardware_interface::JointHandle;

    const auto	joint_name = _nh.param<std::string>("joint_name",
						    "finger_joint");

  // Connect and register the joint state interface.
    _joint_state_interface.registerHandle(
	JointStateHandle(joint_name, &_pos, &_vel, &_eff));
    registerInterface(&_joint_state_interface);

  // Connect and register the joint position interface.
    _joint_position_interface.registerHandle(
	JointHandle(_joint_state_interface.getHandle(joint_name), &_cmd_pos));
    registerInterface(&_joint_position_interface);
}

robotiq_driver_base::~robotiq_driver_base()
{
}

void
robotiq_driver_base::run()
{
    controller_manager::ControllerManager	cm(this, _nh);
    ros::AsyncSpinner				spinner(1);
    spinner.start();

    while (ros::ok())
    {
	read();		// gripper ==> (_pos, _vel, _eff)
	cm.update(ros::Time::now(), _period);
	write();	// gripper <== (_cmd_pos, _cmd_vel, _cmd_eff)

	_period.sleep();
    }

    spinner.stop();
}

bool
robotiq_driver_base::ready() const
{
    const auto	status = getStatus();
    return (status.gSTA == 3 && status.gACT == 1);
}

bool
robotiq_driver_base::moving() const
{
    const auto	status = getStatus();
    return (status.gGTO == 1 && status.gOBJ == 0);
}

bool
robotiq_driver_base::stalled() const
{
    const auto	status = getStatus();
    return (status.gOBJ == 1 || status.gOBJ == 2);
}

void
robotiq_driver_base::activate()
{
    sendCommand({1, 1, 0, 0, 0, 255, 150});
}

void
robotiq_driver_base::stop()
{
    sendCommand({1, 0, 0, 0, 0, 0, 0});
}

void
robotiq_driver_base::read()
{
    const auto	status = getStatus();

    _pos = clamp(-(status.gPO - _min_pos_counts)*
		 (_max_pos - _min_pos)/_min_pos_counts,
		 _min_pos, _max_pos);
}

void
robotiq_driver_base::write()
{
    Command	command;
    command.rACT = 1;
    command.rGTO = 1;
    command.rATR = 0;
    command.rARD = 0;
    command.rPR  = clamp(_min_pos_counts -
			 int(_min_pos_counts*
			     (_cmd_pos - _min_pos)/(_max_pos - _min_pos)),
			 0, _min_pos_counts);
    command.rSP  = clamp(int(255*(_cmd_vel - _min_vel)/(_max_vel - _min_vel)),
			 0, 255);
    command.rFR  = clamp(int(255*(_cmd_eff - _min_eff)/(_max_eff - _min_eff)),
			 0, 255);
    sendCommand(command);
}

/************************************************************************
*  class robotiq_urcap_driver						*
************************************************************************/
robotiq_urcap_driver::robotiq_urcap_driver(const ros::NodeHandle& nh)
    :robotiq_driver_base(nh),
     _socket(socket(AF_INET, SOCK_STREAM, 0))
{
  // Check whether the socket is correctly opened.
    if (_socket < 0)
	throw std::runtime_error(std::string("failed to open socket: ")
				 + strerror(errno));

  // Get hoastname and port from parameters.
    const auto	hostname = _nh.param<std::string>("hostname", "10.66.171.52");
    const auto	port	 = _nh.param<int>("port", 63352);

  // Connect socket to hostname:port.
    const auto	addr = inet_addr(hostname.c_str());
    if (addr == 0xffffffff)
    {
	const auto	h = gethostbyname(hostname.c_str());
	if (!h)
	    throw std::runtime_error("unknown host name: " + hostname);

	for (auto addr_ptr = (u_long**)h->h_addr_list; *addr_ptr; ++addr_ptr)
	    if (connect_socket(*(*addr_ptr), port))
		return;

	throw std::runtime_error("no address found");
    }
    else
    {
	if (connect_socket(addr, port))
	    return;

	throw std::runtime_error(std::string("failed to connect socket: ")
				 + strerror(errno));
    }
}

robotiq_urcap_driver::~robotiq_urcap_driver()
{
    if (_socket >= 0)
	close(_socket);
}

bool
robotiq_urcap_driver::connect_socket(u_long s_addr, int port)
{
    sockaddr_in	server;
    server.sin_family	   = AF_INET;
    server.sin_port	   = htons(port);
    server.sin_addr.s_addr = s_addr;
    ROS_INFO_STREAM("trying to connect socket to "
		    << inet_ntoa(server.sin_addr) << ':'
		    << port << "...");
    if (::connect(_socket, (sockaddr*)&server, sizeof(server)) == 0)
    {
	ROS_INFO_STREAM("succeeded.");
	return true;
    }
    else
    {
	ROS_ERROR_STREAM("failed: " << strerror(errno));
	return false;
    }
}

robotiq_urcap_driver::Status
robotiq_urcap_driver::getStatus() const
{
    return {getVar("ACT"), getVar("GTO"), getVar("STA"), getVar("OBJ"),
	    getVar("FLT"), getVar("PRE"), getVar("POS"), getVar("CUR")};
}

void
robotiq_urcap_driver::sendCommand(const Command& cmd)
{
    const auto	urcap_cmd = std::string("SET")
			  + " ACT " + std::to_string(cmd.rACT)
			  + " GTO " + std::to_string(cmd.rGTO)
			  + " ATR " + std::to_string(cmd.rATR)
			  + " ADR " + std::to_string(cmd.rARD)
			  + " POS " + std::to_string(cmd.rPR)
			  + " SPE " + std::to_string(cmd.rSP)
			  + " FOR " + std::to_string(cmd.rFR)
			  + '\n';
    if (::write(_socket, urcap_cmd.data(), urcap_cmd.size()) < 0)
	throw std::runtime_error(std::string("failed to write to socket. ")
				 + strerror(errno));
}

uint8_t
robotiq_urcap_driver::getVar(const std::string& name) const
{
  // Request URCap the value of a variable with the specified name.
    const auto	urcap_cmd = "GET " + name + '\n';
    if (::write(_socket, urcap_cmd.data(), urcap_cmd.size()) < 0)
	throw std::runtime_error(std::string("failed to write to socket. ")
				 + strerror(errno));

  // Store the response from URCap in a character array.
    std::array<char, 1024>	buf;
    const auto		nbytes = ::read(_socket, buf.data(), buf.size());
    if (nbytes < 0)
	throw std::runtime_error(std::string("failed to read from socket. ")
				 + strerror(errno));
    buf[nbytes] = '\0';

  // Extract echoed variable name and check if it matches the specified name.
    const auto	var_name = strtok(buf.data(), " \t\n");
    if (!var_name)
	throw std::runtime_error("variable name not echoed: " + name);
    else if (var_name != name)
	throw std::runtime_error("mismatchied varilable name: "
				 + name + " != " + var_name);

  // Extract value of the variable with the spcedified name.
    return strtol(strtok(nullptr, " \t\n"), nullptr, 0);
}

}	// namespace aist_robotiq

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "aist_robotiq_driver");
    ros::NodeHandle	nh;

    try
    {
	aist_robotiq::robotiq_urcap_driver	driver(nh);
	driver.run();
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("(aist_robotiq_driver) " << err.what());
    }

    return 0;
}
