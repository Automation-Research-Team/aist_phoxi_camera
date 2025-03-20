//#define WITH_ROS
#define WITH_ROS_H

#if defined(WITH_ROS)
#  define WITH_ROS_H
#endif
#if defined(WITH_ROS_H)
#  include <ros/ros.h>
#endif

#include <PhoXi.h>
#include <iostream>

int
main(int argc, char* argv[])
{
#if defined(WITH_ROS)
    ros::init(argc, argv, "test_compatibility");

    ros::NodeHandle		nh("~");
    std::cerr << "*** NodeHandle created" << std::endl;
#endif
    pho::api::PhoXiFactory	factory;
    std::cerr << "*** PhoXiFactory created" << std::endl;
    
    if (!factory.isPhoXiControlRunning())
    {
	std::cerr << "*** PhoXiControll is not running" << std::endl;
	return 1;
    }
    std::cerr << "*** PhoXiControll is running" << std::endl;

    return 0;
}
