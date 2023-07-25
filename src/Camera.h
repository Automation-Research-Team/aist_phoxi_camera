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
#include <tf/transform_broadcaster.h>

#include <PhoXi.h>

#if defined(PROFILE)
#  include "TU/Profiler.h"
#endif

namespace aist_phoxi_camera
{
/************************************************************************
*   class Camera                                                        *
************************************************************************/
class Camera
#if defined(PROFILE)
    : public TU::Profiler<>
#endif
{
  private:
    using cloud_t		 = sensor_msgs::PointCloud2;
    using cloud_p		 = sensor_msgs::PointCloud2Ptr;
    using image_t		 = sensor_msgs::Image;
    using image_p		 = sensor_msgs::ImagePtr;
    using image_cp		 = sensor_msgs::ImageConstPtr;
    using cinfo_t		 = sensor_msgs::CameraInfo;
    using cinfo_p		 = sensor_msgs::CameraInfoPtr;
    using ddynamic_reconfigure_t = ddynamic_reconfigure::DDynamicReconfigure;
#if defined(PROFILE)
    using profiler_t		 = TU::Profiler<>;
#endif

    enum
    {
	XYZ_ONLY = 0, WITH_RGB = 1, WITH_NORMAL = 2, WITH_RGB_NORMAL = 4
    };

  public:
		Camera(ros::NodeHandle& nh,
		       const std::string& nodelet_name)			;
		~Camera()						;

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
			    T value, bool suspend)			;
    template <class F, class T>
    void	set_field(pho::api::PhoXiFeature<F> pho::api::PhoXi::*
			      feature,
			  T F::* member, T value, bool suspend,
			  const std::string& field_name)		;
    template <class T>
    void	set_member(T& member, T value, const std::string& name)	;
    void	set_color_resolution(int idx)				;
    void	set_white_balance_preset(const std::string& preset)	;
    bool	trigger_frame(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    bool	save_frame(SetString::Request&  req,
			   SetString::Response& res)			;
    bool	save_settings(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    bool	restore_settings(std_srvs::Trigger::Request&  req,
				 std_srvs::Trigger::Response& res)	;
    template <class T>
    image_cp	create_image(const ros::Time& stamp,
			     const std::string& frame_id,
			     const std::string& encoding, float scale,
			     const pho::api::Mat2D<T>& phoxi_image) const;
    void	cache_camera_matrix()				const	;
    void	set_camera_info(const cinfo_p& cinfo,
				const ros::Time& stamp,
				const std::string& frame_id,
				size_t width, size_t height,
				const pho::api::CameraMatrix64f& K,
				const std::vector<double>& D,
				const pho::api::Point3_64f& t,
				const pho::api::Point3_64f& rx,
				const pho::api::Point3_64f& ry,
				const pho::api::Point3_64f& rz)	const	;
    void	publish_frame()					const	;
    void	publish_cloud(const ros::Time& stamp,
			      float distanceScale)		const	;
    template <class T>
    void	publish_image(const ros::Time& stamp,
			      const std::string& encoding, float scale,
			      const pho::api::Mat2D<T>& phoxi_image,
			      const image_transport::Publisher& publisher)
								const	;
    void	publish_camera_info(const ros::Time& stamp)	const	;
    void	publish_color_camera(const ros::Time& stamp)	const	;
    const std::string&
		getName()		const	{ return _nodelet_name; }
    void	profiler_start(int n) const
		{
#if defined(PROFILE)
		    profiler_t::start(n);
#endif
		}
    void	profiler_print(std::ostream& out) const
		{
#if defined(PROFILE)
		    profiler_t::nextFrame();
		    profiler_t::print(out);
#endif
		}

  private:
    const std::string				_nodelet_name;

    pho::api::PhoXiFactory			_factory;
    pho::api::PPhoXi				_device;
    pho::api::PFrame				_frame;
    const std::string				_frame_id;
    const std::string				_color_camera_frame_id;
    const double				_rate;
    bool					_denseCloud;
    double					_intensityScale;
    bool					_is_color_camera;
    mutable pho::api::CameraMatrix64f		_camera_matrix;

    const cinfo_p				_cinfo;

    ddynamic_reconfigure_t			_ddr;

    const ros::ServiceServer			_trigger_frame_server;
    const ros::ServiceServer			_save_frame_server;
    const ros::ServiceServer			_save_settings_server;
    const ros::ServiceServer			_restore_settings_server;

    image_transport::ImageTransport		_it;
    const ros::Publisher			_cloud_publisher;
    const image_transport::Publisher		_normal_map_publisher;
    const image_transport::Publisher		_depth_map_publisher;
    const image_transport::Publisher		_confidence_map_publisher;
    const image_transport::Publisher		_event_map_publisher;
    const image_transport::Publisher		_texture_publisher;
    const ros::Publisher			_camera_info_publisher;
    const image_transport::CameraPublisher	_color_camera_publisher;
    mutable tf::TransformBroadcaster		_broadcaster;
};

}	// namespace aist_phoxi_camera
