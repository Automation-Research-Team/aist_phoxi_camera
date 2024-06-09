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
#include <std_srvs/srv/trigger.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
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
    using cloud_t		 = sensor_msgs::msg::PointCloud2;
    using cloud_p		 = cloud_t::UniquePtr;
    using image_t		 = sensor_msgs::msg::Image;
    using image_p		 = image_t::UniquePtr;
    using camera_info_t		 = sensor_msgs::msg::CameraInfo;
    using camera_info_p		 = camera_info_t::UniquePtr;
    using trigger_t		 = std_srvs::srv::Trigger;
    using trigger_srv_p		 = rclcpp::Service<trigger_t>::SharedPtr;
    using trigger_req_p		 = trigger_t::Request::SharedPtr;
    using trigger_res_p		 = trigger_t::Response::SharedPtr;
    using ddynamic_reconfigure_t = ddynamic_reconfigure2::DDynamicReconfigure;
#if defined(PROFILE)
    using profiler_t		 = TU::Profiler<>;
#endif
    template <class MSG>
    using publisher_p		 = typename rclcpp::Publisher<MSG>::SharedPtr;
    using broadcaster_t		 = tf2_ros::StaticTransformBroadcaster;
    using timer_p		 = rclcpp::TimerBase::SharedPtr;

    enum
    {
	XYZ_ONLY = 0, WITH_RGB = 1, WITH_NORMAL = 2, WITH_RGB_NORMAL = 4
    };

  public:
		Camera(const rclcpp::NodeOptions& options)		;
		~Camera()						;

  private:
    std::string	node_name() const
		{
		    return get_name();
		}
    void	tick()							;
    void	setup_ddr_phoxi()					;
    void	setup_ddr_motioncam()					;
    void	setup_ddr_common()					;
    template <class F>
    bool	is_available(const pho::api::PhoXiFeature<F>& feature)
								const	;
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
    template <class F>
    void	set_texture_source(
		    pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
		    pho::api::PhoXiTextureSource texture_source)	;
    void	set_resolution(size_t idx)				;
    void	set_color_resolution(size_t idx)			;
    void	set_white_balance_preset(const std::string& preset)	;
    bool	trigger_frame(const trigger_req_p, trigger_res_p res)	;
    bool	save_settings(const trigger_req_p, trigger_res_p res)	;
    bool	restore_settings(const trigger_req_p, trigger_res_p res);
    void	cache_camera_matrix()					;
    template <class T>
    image_p	create_image(const rclcpp::Time& stamp,
			     const std::string& frame_id,
			     const std::string& encoding, float scale,
			     const pho::api::Mat2D<T>& phoxi_image) const;
    camera_info_p
		create_camera_info(const rclcpp::Time& stamp,
				   const std::string& frame_id,
				   size_t width, size_t height,
				   const pho::api::CameraMatrix64f& K,
				   const std::vector<double>& D,
				   const pho::api::Point3_64f& t,
				   const pho::api::Point3_64f& rx,
				   const pho::api::Point3_64f& ry,
				   const pho::api::Point3_64f& rz) const;
    void	publish_frame()						;
    void	publish_cloud(const rclcpp::Time& stamp,
			      float distanceScale)		const	;
    template <class T>
    void	publish_image(const rclcpp::Time& stamp,
			      const std::string& encoding, float scale,
			      const pho::api::Mat2D<T>& phoxi_image,
			      const image_transport::Publisher& publisher)
								const	;
    void	publish_camera_info(const rclcpp::Time& stamp)		;
    void	publish_color_camera(const rclcpp::Time& stamp)		;
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
    ddynamic_reconfigure_t			_ddr;

    pho::api::PhoXiFactory			_factory;
    pho::api::PPhoXi				_device;
    pho::api::PFrame				_frame;
    pho::api::PhoXiSize				_depth_map_size;
    pho::api::PhoXiSize				_color_camera_image_size;
    pho::api::CameraMatrix64f			_camera_matrix;

    const std::string				_frame_id;
    const std::string				_color_camera_frame_id;
    double					_intensity_scale;
    bool					_dense_cloud;
    bool					_color_texture_source;

    const trigger_srv_p				_trigger_frame_srv;
    const trigger_srv_p				_save_settings_srv;
    const trigger_srv_p				_restore_settings_srv;

    image_transport::ImageTransport		_it;
    const publisher_p<cloud_t>			_cloud_pub;
    const image_transport::Publisher		_normal_map_pub;
    const image_transport::Publisher		_depth_map_pub;
    const image_transport::Publisher		_confidence_map_pub;
    const image_transport::Publisher		_event_map_pub;
    const image_transport::Publisher		_texture_pub;
    const publisher_p<camera_info_t>		_camera_info_pub;
    const image_transport::CameraPublisher	_color_camera_pub;
    broadcaster_t				_static_broadcaster;

    const timer_p				_timer;
};
}	// namespace aist_phoxi_camera
