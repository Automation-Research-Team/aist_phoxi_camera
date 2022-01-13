// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
 *  \file	Camera.h
 */
#include <string>
#include <array>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <aist_phoxi_camera/SetString.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include <PhoXi.h>

namespace aist_phoxi_camera
{
/************************************************************************
*   class Camera                                                        *
************************************************************************/
class Camera
{
  private:
    using cloud_t = sensor_msgs::PointCloud2;
    using cloud_p = sensor_msgs::PointCloud2Ptr;
    using image_t = sensor_msgs::Image;
    using image_p = sensor_msgs::ImagePtr;
    using cinfo_t = sensor_msgs::CameraInfo;

    enum
    {
	XYZ_ONLY = 0, WITH_RGB = 1, WITH_NORMAL = 2, WITH_RGB_NORMAL = 4
    };

  public:
		Camera(const ros::NodeHandle& nh)			;
		~Camera()						;

    void	run()							;
    void	tick()							;
    double	rate()						const	;

  private:
    void	setup_ddr_phoxi()					;
    void	setup_ddr_motioncam()					;
    void	setup_ddr_common()					;
    void	set_resolution(int idx)					;
    template <class F, class T>
    void	set_feature(pho::api::PhoXiFeature<F> pho::api::PhoXi::*
			      feature,
			    T value, bool pause)			;
    template <class F, class T>
    void	set_field(pho::api::PhoXiFeature<F> pho::api::PhoXi::*
			      feature,
			  T F::* member, T value, bool pause)		;
    template <class T>
    void	set_member(T& member, T value, const std::string& name)	;
    void	lock_gui(bool enable)					;
    bool	trigger_frame(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    bool	save_frame(SetString::Request&  req,
			   SetString::Response& res)			;
    bool	save_settings(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    bool	restore_settings(std_srvs::Trigger::Request&  req,
				 std_srvs::Trigger::Response& res)	;
    template <class T>
    static bool	save_image(const std::string& filename,
			   const pho::api::Mat2D<T>& phoxi_image,
			   float scale)					;
    void	publish_frame()					const	;
    void	publish_cloud(const ros::Time& stamp,
			      float distanceScale)		const	;
    template <class T>
    void	publish_image(const pho::api::Mat2D<T>& phoxi_image,
			      const image_transport::Publisher& publisher,
			      const ros::Time& stamp,
			      const std::string& encoding,
			      typename T::ElementChannelType scale)
								const	;
    void	publish_camera_info(const ros::Time& stamp)	const	;

  private:
    ros::NodeHandle			_nh;

    const pho::api::PhoXiFactory	_factory;
    pho::api::PPhoXi			_device;
    pho::api::PFrame			_frame;
    std::string				_frame_id;	// frame id used by tf
    double				_rate;		// frequency
    std::array<double, 8>		_D;		// distortion param.
    std::array<double, 9>		_K;		// intrinsic param.
    int					_pointFormat;
    double				_intensityScale;

    ddynamic_reconfigure::DDynamicReconfigure	_ddr;

    const ros::ServiceServer		_trigger_frame_server;
    const ros::ServiceServer		_save_frame_server;
    const ros::ServiceServer		_save_settings_server;
    const ros::ServiceServer		_restore_settings_server;

    image_transport::ImageTransport	_it;
    const ros::Publisher		_cloud_publisher;
    const image_transport::Publisher	_normal_map_publisher;
    const image_transport::Publisher	_depth_map_publisher;
    const image_transport::Publisher	_confidence_map_publisher;
    const image_transport::Publisher	_texture_publisher;
    const ros::Publisher		_camera_info_publisher;
};

}	// namespace aist_phoxi_camera
