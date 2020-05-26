/*!
 *  \file	main.cpp
 */
#include "Camera.h"

/************************************************************************
*   global functions                                                    *
************************************************************************/
int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aist_phoxi_camera");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	ros::NodeHandle			nh("~");
        aist_phoxi_camera::Camera	camera(nh);
        camera.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
        return 1;
    }

    return 0;
}
