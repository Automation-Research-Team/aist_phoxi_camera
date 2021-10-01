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
 *  \file	nodelet.cpp
 */
#include "Camera.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace aist_phoxi_camera
{
/************************************************************************
*  class CameraNodelet							*
************************************************************************/
class CameraNodelet : public nodelet::Nodelet
{
  public:
    CameraNodelet()							{}

    virtual void	onInit()					;
    void		timer_callback(const ros::TimerEvent&)		;

  private:
    boost::shared_ptr<Camera>	_node;
    ros::Timer			_timer;
};

void
CameraNodelet::onInit()
{
    NODELET_INFO("aist_phoxi_camera::CameraNodelet::onInit()");

    const auto&	nh = getPrivateNodeHandle();
    _node.reset(new Camera(nh));
    std::cout << "rate = " << 1.0/_node->rate() << std::endl;
    _timer = nh.createTimer(ros::Duration(1.0/_node->rate()),
			    &CameraNodelet::timer_callback, this);

}

void
CameraNodelet::timer_callback(const ros::TimerEvent&)
{
    _node->tick();
}

}

PLUGINLIB_EXPORT_CLASS(aist_phoxi_camera::CameraNodelet, nodelet::Nodelet);
