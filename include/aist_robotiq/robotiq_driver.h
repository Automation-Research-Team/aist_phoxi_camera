/*!
  \file		robotiq_driver.h
  \author	Toshio UESHIBA
*/
#if !defined(AIST_ROBOTIQ_ROBOTIQ_DRIVER_H)
#  include <cstdint>
#  include <hardware_interface/robot_hw.h>
#  include <hardware_interface/joint_state_interface.h>
#  include <hardware_interface/joint_command_interface.h>

namespace aist_robotiq
{
/************************************************************************
*  class robotiq_driver_base						*
************************************************************************/
class robotiq_driver_base : public hardware_interface::RobotHW
{
  public:
    struct Status
    {
	uint8_t	gACT;	// activateion status [0: reset, 1: activation]
	uint8_t	gGTO;	// action status [0: stopped, 1: going to rGTO]
	uint8_t	gSTA;	// [0: reset, i: moving, 2: unused 3: completed]
	uint8_t	gOBJ;	// [0: moving, 1/2: stalled, 3: stopped, not stalled]
	uint8_t	gFLT;	// falult status [0: no fault]
	uint8_t	gPR;	// position request echo
	uint8_t	gPO;	// actual position
	uint8_t	gCU;	// actual current [10x of mA]
    };

    struct Command
    {
	uint8_t	rACT;	// action request [0: inactive, 1: active]
	uint8_t	rGTO;	// go-to [0: stop, 1: go to requested position]
	uint8_t	rATR;	// automatic release [0: normal, 1: emergency]
	uint8_t	rARD;	// auto-release direction [0: closing, 1: opening]
	uint8_t	rPR;	// position request
	uint8_t	rSP;	// speed request
	uint8_t	rFR;	// force request
    };

  public:
			robotiq_driver_base(const ros::NodeHandle& nh)	;
    virtual		~robotiq_driver_base()				;

    void		run()						;

  private:
    bool		ready()					const	;
    bool		moving()				const	;
    bool		stalled()				const	;
    void		activate()					;
    void		stop()						;

    void		read()						;
    void		write()						;

    virtual Status	getStatus()				const	= 0;
    virtual void	sendCommand(const Command& cmd)			= 0;

  protected:
    const ros::NodeHandle			_nh;

  private:
    hardware_interface::JointStateInterface	_joint_state_interface;
    hardware_interface::PositionJointInterface	_joint_position_interface;

    const ros::Duration	_period;
    const int		_min_pos_counts;
    const double	_min_pos;
    const double	_max_pos;
    const double	_min_vel;
    const double	_max_vel;
    const double	_min_eff;
    const double	_max_eff;

    double		_pos;	// position
    double		_vel;	// velocity
    double		_eff;	// effort

    double		_cmd_pos;
};

/************************************************************************
*  class robotiq_urcap_driver						*
************************************************************************/
class robotiq_urcap_driver : public robotiq_driver_base
{
  public:
			robotiq_urcap_driver(const ros::NodeHandle& nh)	;
    virtual		~robotiq_urcap_driver()				;

  private:
    bool		connect_socket(u_long hostname, int port)	;

  private:
    virtual Status	getStatus()				const	;
    virtual void	sendCommand(const Command& cmd)			;
    uint8_t		getVar(const std::string& name)		const	;

  private:
    const int		_socket;
};

}	// namespace aist_roibotiq
#endif	// AIST_ROBOTIQ_ROBOTIQ_DRIVER_H
