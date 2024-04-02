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

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/Trigger.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msgs/PointCloud2.h>
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
class Camera : public rclcpp::Node
#if defined(PROFILE)
    : public TU::Profiler<>
#endif
{
  private:
    using cloud_t	= sensor_msgs::msg::PointCloud2;
    using cloud_p	= cloud_t::Ptr;
    using image_t	= sensor_msgs::msg::Image;
    using image_p	= image_t::Ptr;
    using cinfo_t	= sensor_msgs::msg::CameraInfo;
    using cinfo_p	= cinfo_t::Ptr;
    using trigger_srv_p	= rclcpp::Service<std_srvs::srv::Trigger>::Ptr;
#if defined(PROFILE)
    using profiler_t	= TU::Profiler<>;
#endif

    enum
    {
	XYZ_ONLY = 0, WITH_RGB = 1, WITH_NORMAL = 2, WITH_RGB_NORMAL = 4
    };

  public:
		Camera(const std::string& node_name,
		       const rclcpp::NodeOptions& options)		;
		~Camera()						;

    void	tick()							;
    double	rate()						const	;

  private:
    void	setup_ddr_phoxi()					;
    void	setup_ddr_motioncam()					;
    void	setup_ddr_common()					;
    void	set_resolution(size_t idx)				;
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
    void	set_color_resolution(size_t idx)			;
    void	set_white_balance_preset(const std::string& preset)	;
    bool	trigger_frame(std_srvs::srv::Trigger::Request&  req,
			      std_srvs::srv::Trigger::Response& res)	;
    bool	save_settings(std_srvs::srv::Trigger::Request&  req,
			      std_srvs::srv::Trigger::Response& res)	
    bool	restore_settings(std_srvs::srv::Trigger::Request&  req,
				 std_srvs::srv::Trigger::Response& res)	;
    template <class T>
    void	set_image(const image_p& image, const ros::Time& stamp,
			  const std::string& frame_id,
			  const std::string& encoding, float scale,
			  const pho::api::Mat2D<T>& phoxi_image)	;
    void	set_camera_matrix()					;
    void	set_camera_info(const cinfo_p& cinfo,
				const ros::Time& stamp,
				const std::string& frame_id,
				size_t width, size_t height,
				const pho::api::CameraMatrix64f& K,
				const std::vector<double>& D,
				const pho::api::Point3_64f& t,
				const pho::api::Point3_64f& rx,
				const pho::api::Point3_64f& ry,
				const pho::api::Point3_64f& rz)		;
    void	publish_frame()						;
    void	publish_cloud(const ros::Time& stamp,
			      float distanceScale)			;
    template <class T>
    void	publish_image(const image_p& image, const ros::Time& stamp,
			      const std::string& encoding, float scale,
			      const pho::api::Mat2D<T>& phoxi_image,
			      const image_transport::Publisher& publisher);
    void	publish_camera_info(const ros::Time& stamp)		;
    void	publish_color_camera(const ros::Time& stamp)		;
    const std::string&
		getName()		const	{ return _nodelet_name; }
    void	profiler_start(int n)
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
    pho::api::PhoXiFactory			_factory;
    pho::api::PPhoXi				_device;
    pho::api::PFrame				_frame;
    const std::string				_frame_id;
    const std::string				_color_camera_frame_id;
    const double				_rate;
    bool					_denseCloud;
    double					_intensityScale;
    bool					_is_color_camera;
    pho::api::CameraMatrix64f			_camera_matrix;

    const cloud_p				_cloud;
    const image_p				_normal_map;
    const image_p				_depth_map;
    const image_p				_confidence_map;
    const image_p				_event_map;
    const image_p				_texture;
    const cinfo_p				_cinfo;
    const image_p				_color_camera_image;
    const cinfo_p				_color_camera_cinfo;

    const trigger_srv_p				_trigger_frame_server;
    const trigger_srv_p				_save_settings_server;
    const trigger_srv_p				_restore_settings_server;

    image_transport::ImageTransport		_it;
    const ros::Publisher			_cloud_publisher;
    const image_transport::Publisher		_normal_map_publisher;
    const image_transport::Publisher		_depth_map_publisher;
    const image_transport::Publisher		_confidence_map_publisher;
    const image_transport::Publisher		_event_map_publisher;
    const image_transport::Publisher		_texture_publisher;
    const ros::Publisher			_camera_info_publisher;
    const image_transport::CameraPublisher	_color_camera_publisher;
    tf::TransformBroadcaster			_broadcaster;
};

}	// namespace aist_phoxi_camera
