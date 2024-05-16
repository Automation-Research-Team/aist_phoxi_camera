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
 *  \file	Camera.cpp
 */
#if PHO_SOFTWARE_VERSION_MAJOR >= 1
#  if PHO_SOFTWARE_VERSION_MINOR >= 4
#    define HAVE_MOTIONCAM
#    if PHO_SOFTWARE_VERSION_MINOR >= 5
#      define HAVE_INTERREFLECTIONS_FILTERING
#      define HAVE_HARDWARE_TRIGGER
#      define HAVE_MOTIONCAM_EXPOSURE
#      if PHO_SOFTWARE_VERSION_MINOR >= 7
#        define HAVE_HARDWARE_TRIGGER_SIGNAL
#        define HAVE_INTERREFLECTION_FILTER_STRENGTH
#	 if PHO_SOFTWARE_VERSION_MINOR >= 8
#          define HAVE_LED_POWER
//#          define HAVE_LED_SHUTTER_MULTIPLIER
#	   if PHO_SOFTWARE_VERSION_MINOR >= 9
#            define HAVE_COLOR_CAMERA
#          endif
#        endif
#      endif
#    endif
#  endif
#endif

#include "Camera.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <x86intrin.h>

namespace aist_phoxi_camera
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static std::ostream&
operator <<(std::ostream& out, const pho::api::PhoXiFeature<T>& feature)
{
    return out << feature.GetValue();
}

static std::ostream&
operator <<(std::ostream& out, const pho::api::Point3_64f& p)
{
    return out << '(' << p.x << ' ' << p.y << ' ' << p.z << ')';
}

static size_t
npoints_valid(const pho::api::PointCloud32f& cloud)
{
    size_t	n = 0;
    for (int v = 0; v < cloud.Size.Height; ++v)
    {
	auto	p = cloud[v];

	for (const auto q = p +  cloud.Size.Width; p != q; ++p)
	    if (float(p->z) != 0.0f)
		++n;
    }

    return n;
}

static void
scale_copy(const float* in, const float* ie, uint8_t* out, float scale)
{
#if defined(AVX2)
    const auto	k = _mm256_set1_ps(scale);

    for (; in != ie; out += 32)
    {
	auto i_lo = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), k));
	in += 8;
	auto i_hi = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), k));
	in += 8;
	const auto s_lo = _mm256_packs_epi32(
				_mm256_permute2f128_si256(i_lo, i_hi, 0x20),
				_mm256_permute2f128_si256(i_lo, i_hi, 0x31));

	i_lo = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), k));
	in += 8;
	i_hi = _mm256_cvtps_epi32(_mm256_mul_ps(_mm256_loadu_ps(in), k));
	in += 8;
	const auto s_hi = _mm256_packs_epi32(
				_mm256_permute2f128_si256(i_lo, i_hi, 0x20),
				_mm256_permute2f128_si256(i_lo, i_hi, 0x31));

	_mm256_storeu_si256(reinterpret_cast<__m256i*>(out),
			    _mm256_packus_epi16(
				_mm256_permute2f128_si256(s_lo, s_hi, 0x20),
				_mm256_permute2f128_si256(s_lo, s_hi, 0x31)));
    }
#else
    std::transform(in, ie, out,
		   [scale](const auto& x)->uint8_t
		   { return std::min(scale * x, 255.0f); });
#endif
}

static void
scale_copy(const float* in, const float* ie, float* out, float scale)
{
#if defined(AVX2)
    const auto	k = _mm256_set1_ps(scale);

    for (; in != ie; in += 8, out += 8)
	_mm256_storeu_ps(out, _mm256_mul_ps(_mm256_loadu_ps(in), k));
#else
    std::transform(in, ie, out,
		   [scale](const auto& x)->float
		   { return scale * x; });
#endif
}

static void
scale_copy(const uint16_t* in, const uint16_t* ie, uint8_t* out, float scale)
{
    std::transform(in, ie, out,
		   [scale](const auto& x)->uint8_t
		   { return std::min(scale * x, 255.0f); });
}

static void
scale_copy(const uint16_t* in, const uint16_t* ie, float* out, float scale)
{
    std::transform(in, ie, out,
		   [scale](const auto& x)->float
		   { return scale * x; });
}

/************************************************************************
*  class Camera								*
************************************************************************/
Camera::Camera(const rclcpp::NodeOptions& options)
    :rclcpp::Node("aist_phoxi_camera", options),
#if defined(PROFILE)
     profiler_t(8),
#endif
     _factory(),
     _device(nullptr),
     _frame(nullptr),
     _frame_id(declare_parameter<std::string>("frame", "sensor")),
     _color_camera_frame_id(declare_parameter<std::string>(
				"color_camera_frame", "color_camera_frame")),
     _dense_cloud(false),
     _intensity_scale(0.5),
     _camera_matrix(pho::api::PhoXiSize(3, 3)),
     _cloud(new cloud_t),
     _normal_map(new image_t),
     _depth_map(new image_t),
     _confidence_map(new image_t),
     _event_map(new image_t),
     _texture(new image_t),
     _camera_info(new camera_info_t),
     _color_camera_image(new image_t),
     _color_camera_camera_info(new camera_info_t),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _trigger_frame_srv(create_service<trigger_t>(
			    "trigger_frame",
			    std::bind(&Camera::trigger_frame, this,
				      std::placeholders::_1,
				      std::placeholders::_2))),
     _save_settings_srv(create_service<trigger_t>(
			    "save_settings",
			    std::bind(&Camera::save_settings, this,
				      std::placeholders::_1,
				      std::placeholders::_2))),
     _restore_settings_srv(create_service<trigger_t>(
			       "restore_settings",
			       std::bind(&Camera::restore_settings, this,
					 std::placeholders::_1,
					 std::placeholders::_2))),
     _it(rclcpp::Node::SharedPtr(this)),
     _cloud_pub(create_publisher<cloud_t>("pointcloud",		1)),
     _normal_map_pub(    _it.advertise("normal_map",		1)),
     _depth_map_pub(     _it.advertise("depth_map",		1)),
     _confidence_map_pub(_it.advertise("confidence_map",	1)),
     _event_map_pub(     _it.advertise("event_map",		1)),
     _texture_pub(       _it.advertise("texture",		1)),
     _camera_info_pub(create_publisher<camera_info_t>("camera_info",	1)),
     _color_camera_pub(  _it.advertiseCamera("color/image",	1)),
     _static_broadcaster(*this),
     _timer(create_wall_timer(std::chrono::duration<double>(
				  declare_parameter<double>("period", 0.1)),
			      std::bind(&Camera::tick, this)))
{
    using namespace	pho::api;

  // Search for a device with specified ID.
    auto	id = declare_parameter<std::string>(
			"id", "InstalledExamples-basic-example");
    for (size_t pos; (pos = id.find('\"')) != std::string::npos; )
	id.erase(pos, 1);

    if (!_factory.isPhoXiControlRunning())
    {
	RCLCPP_ERROR_STREAM(get_logger(), "PhoXiControll is not running");
	throw;
    }

    for (const auto& devinfo : _factory.GetDeviceList())
	if (devinfo.HWIdentification == id)
	{
	    _device = _factory.Create(devinfo.GetTypeHWIdentification());
	    break;
	}
    if (!_device)
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "failed to find camera[" << id << ']');
	throw;
    }

  // Connect to the device.
    if (!_device->Connect())
    {
	RCLCPP_ERROR_STREAM(get_logger(),
			    "failed to open camera[" << id << ']');
	throw;
    }

  // Stop acquisition.
    _device->StopAcquisition();

    RCLCPP_INFO_STREAM(get_logger(),"initializing configuration");

  // Assure _camera_matrix to be set on the arrival of first frame.
    _camera_info->width = _camera_info->height = 0;

  // Setup ddynamic_reconfigure services.
    switch (PhoXiDeviceType::Value(_device->GetType()))
    {
      case PhoXiDeviceType::PhoXiScanner:
	setup_ddr_phoxi();
	break;
#if defined(HAVE_MOTIONCAM)
      case PhoXiDeviceType::MotionCam3D:
	setup_ddr_motioncam();
	break;
#endif
      default:
	RCLCPP_ERROR_STREAM(get_logger(), "unknown device type["
			    << std::string(_device->GetType()) << ']');
	throw;
    }
    setup_ddr_common();

  // Start acquisition.
    _device->ClearBuffer();
    _device->StartAcquisition();

    RCLCPP_INFO_STREAM(get_logger(), "camera is active");
}

Camera::~Camera()
{
    if (_device && _device->isConnected())
    {
	_device->StopAcquisition();
	_device->Disconnect();
    }
}

void
Camera::tick()
{
    using namespace	pho::api;

    if (_device->TriggerMode == PhoXiTriggerMode::Freerun &&
	_device->isAcquiring())
    {
	if ((_frame = _device->GetFrame(PhoXiTimeout::ZeroTimeout)) &&
	    _frame->Successful)
	    publish_frame();
    }
}

void
Camera::setup_ddr_phoxi()
{
    using namespace	pho::api;
    using namespace	std::placeholders;

  // 1. CapturingMode
    if (is_available(_device->CapturingMode))
    {
	const auto	modes = _device->SupportedCapturingModes.GetValue();
	std::map<std::string, int>	enum_resolution;
	size_t				idx = 0;
	for (size_t i = 0; i < modes.size(); ++i)
	{
	    const auto&	resolution = modes[i].Resolution;
	    enum_resolution.emplace(std::to_string(resolution.Width) + 'x' +
				    std::to_string(resolution.Height), i);
	    if (modes[i] == _device->CapturingMode)
		idx = i;
	}
	_ddr.registerEnumVariable<int>("resolution", idx,
				       std::bind(&Camera::set_resolution,
						 this, _1),
				       "Image resolution", enum_resolution);
    }

  // 2. CapturingSettings
    if (is_available(_device->CapturingSettings))
    {
      // 2.1 ShutterMultiplier
	_ddr.registerVariable<int>(
	    "capturing_settings.shutter_multiplier",
	    _device->CapturingSettings->ShutterMultiplier,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::ShutterMultiplier, _1,
		      false, "ShutterMultiplier"),
	    "The number of repeats of indivisual pattern", {1, 20});

      // 2.2 ScanMultiplier
	_ddr.registerVariable<int>(
	    "capturing_settings.scan_multiplier",
	    _device->CapturingSettings->ScanMultiplier,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::ScanMultiplier, _1,
		      false, "ScanMultiplier"),
	    "The number of scans taken and merged to sigle output", {1, 20});

      // 2.3 CameraOnlyMode
	_ddr.registerVariable<bool>(
	    "capturing_settings.camera_only_mode",
	    _device->CapturingSettings->CameraOnlyMode,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::CameraOnlyMode, _1,
		      false, "CameraOnlyMode"),
	    "Use the scanner internal camera to capture only 2D images");

      // 2.4 AmbientLightSuppression
	_ddr.registerVariable<bool>(
	    "capturing_settings.ambient_light_suppression",
	    _device->CapturingSettings->AmbientLightSuppression,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::AmbientLightSuppression, _1,
		      false, "AmbientLightSuppression"),
	    "Enables the mode that suppress ambient illumination");

      // 2.5 MaximumFPS
	_ddr.registerVariable<double>(
	    "capturing_settings.maximum_fps",
	    _device->CapturingSettings->MaximumFPS,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, double>,
		      this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::MaximumFPS, _1,
		      false, "MaximumFPS"),
	    "Maximum fps in freerun mode", {0.0, 30.0});

      // 2.6 SinglePatternExposure
	const auto	exposures = _device->SupportedSinglePatternExposures.GetValue();
	std::map<std::string, double>	enum_single_pattern_exposure;
	for (const auto& exposure: exposures)
	    enum_single_pattern_exposure.emplace(std::to_string(exposure),
						 exposure);
	_ddr.registerEnumVariable<double>(
	    "capturing_settings.single_pattern_exposure",
	    _device->CapturingSettings->SinglePatternExposure,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, double>,
		      this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::SinglePatternExposure, _1,
		      false, "SinglePatternExposure"),
	    "Exposure time for a single patter in miliseconds",
	    enum_single_pattern_exposure);

      // 2.7 CodingStrategy
	if (_device->CapturingSettings->CodingStrategy !=
	    PhoXiCodingStrategy::NoValue)
	    _ddr.registerEnumVariable<int>(
		"capturing_settings.coding_strategy",
		_device->CapturingSettings->CodingStrategy,
		std::bind(&Camera::set_field<PhoXiCapturingSettings,
			  PhoXiCodingStrategy>,
			  this,
			  &PhoXi::CapturingSettings,
			  &PhoXiCapturingSettings::CodingStrategy, _1,
			  false, "CodingStrategy"),
		"Coding strategy",
		{{"Normal",	      PhoXiCodingStrategy::Normal},
		 {"Interreflections", PhoXiCodingStrategy::Interreflections}});

      // 2.8 CodingQuality
	if (_device->CapturingSettings->CodingQuality !=
	    PhoXiCodingQuality::NoValue)
	    _ddr.registerEnumVariable<int>(
		"capturing_settings.coding_quality",
		_device->CapturingSettings->CodingQuality,
		std::bind(&Camera::set_field<PhoXiCapturingSettings,
			  PhoXiCodingQuality>,
			  this,
			  &PhoXi::CapturingSettings,
			  &PhoXiCapturingSettings::CodingQuality, _1,
			  false, "CodingQuality"),
		"Coding quality",
		{{"Fast",	PhoXiCodingQuality::Fast},
		 {"High",	PhoXiCodingQuality::High},
		 {"Ultra",	PhoXiCodingQuality::Ultra}});

      // 2.9 TextureSource
	if (_device->CapturingSettings->TextureSource !=
	    PhoXiTextureSource::NoValue)
	    _ddr.registerEnumVariable<int>(
		"capturing_settings.texture_source",
		_device->CapturingSettings->TextureSource,
		std::bind(&Camera::set_texture_source<PhoXiCapturingSettings>,
			  this, &PhoXi::CapturingSettings, _1),
		"Source used for texture image",
		{{"Computed",	PhoXiTextureSource::Computed},
		 {"LED",	PhoXiTextureSource::LED},
		 {"Laser",	PhoXiTextureSource::Laser},
		 {"Focus",	PhoXiTextureSource::Focus},
		 {"Color",	PhoXiTextureSource::Color}});

      // 2.10 LaserPower
	_ddr.registerVariable<int>(
	    "capturing_settings.laser_power",
	    _device->CapturingSettings->LaserPower,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::LaserPower, _1,
		      false, "LaserPower"),
	    "Laser power", {1, 4095});

      // 2.11 LEDPower
	_ddr.registerVariable<int>(
	    "capturing_settings.led_power",
	    _device->CapturingSettings->LEDPower,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::LEDPower, _1,
		      false, "LEDPower"),
	    "LED power", {0, 4095});

#if defined(HAVE_HARDWARE_TRIGGER)
      // 2.12 hardware trigger
	_ddr.registerVariable<bool>(
	    "capturing_settings.hardware_trigger",
	    _device->CapturingSettings->HardwareTrigger,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::HardwareTrigger, _1,
		      false, "HardwareTrigger"),
	    "Hardware trigger");

#  if defined(HAVE_HARDWARE_TRIGGER_SIGNAL)
      // 2.13 hardware trigger signal
	_ddr.registerEnumVariable<int>(
	    "capturing_settings.hardware_trigger_signal",
	    _device->CapturingSettings->HardwareTriggerSignal,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings,
		      PhoXiHardwareTriggerSignal>,
		      this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::HardwareTriggerSignal, _1,
		      false, "HardwareTriggerSignal"),
	    "Hardware trigger siganl",
	    {{"Falling",	PhoXiHardwareTriggerSignal::Falling},
	     {"Rising",		PhoXiHardwareTriggerSignal::Rising},
	     {"Both",		PhoXiHardwareTriggerSignal::Both}});
#  endif
#endif
#if defined(HAVE_LED_SHUTTER_MULTIPLIER)
      // 2.14 LEDShutterMultiplier
	_ddr.registerVariable<int>(
	    "capturing_settings.led_shutter_multiplier",
	    _device->CapturingSettings->LEDShutterMultiplier,
	    std::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
		      &PhoXi::CapturingSettings,
		      &PhoXiCapturingSettings::LEDShutterMultiplier, _1,
		      false, "LEDShutterMultiplier"),
	    "LED shutter multiplier", {1, 20});
#endif
    }
}

#if defined(HAVE_MOTIONCAM)
void
Camera::setup_ddr_motioncam()
{
    using namespace	pho::api;
    using namespace	std::placeholders;

  // 1. Motioncam
    if (is_available(_device->MotionCam))
    {
      // 1.1 operation mode
	if (_device->MotionCam->OperationMode != PhoXiOperationMode::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam.operation_mode",
		_device->MotionCam->OperationMode,
		std::bind(&Camera::set_field<PhoXiMotionCam,
					     PhoXiOperationMode>, this,
			  &PhoXi::MotionCam,
			  &PhoXiMotionCam::OperationMode, _1,
			  true, "OperationMode"),
		"Operation mode",
		{{"Camera",	PhoXiOperationMode::Camera},
		 {"Scanner",	PhoXiOperationMode::Scanner},
		 {"Mode2D",	PhoXiOperationMode::Mode2D}});

      // 1.2 laser power
	_ddr.registerVariable<int>(
	    "motioncam.laser_power",
	    _device->MotionCam->LaserPower,
	    std::bind(&Camera::set_field<PhoXiMotionCam, int>, this,
		      &PhoXi::MotionCam, &PhoXiMotionCam::LaserPower, _1,
		      false, "LaserPower"),
	    "Laser power", {1, 4095});

#if defined(HAVE_LED_POWER)
      // 1.3 led power
	_ddr.registerVariable<int>(
	    "motioncam.led_power",
	    _device->MotionCam->LEDPower,
	    std::bind(&Camera::set_field<PhoXiMotionCam, int>, this,
		      &PhoXi::MotionCam, &PhoXiMotionCam::LEDPower, _1,
		      false, "LEDPower"),
	    "LED power", {0, 4095});
#endif

      // 1.4 maximum fps
	_ddr.registerVariable<double>(
	    "motioncam.maximum_fps",
	    _device->MotionCam->MaximumFPS,
	    std::bind(&Camera::set_field<PhoXiMotionCam, double>, this,
		      &PhoXi::MotionCam, &PhoXiMotionCam::MaximumFPS, _1,
		      false, "MaximumFPS"),
	    "Maximum fps", {0.0, 20.0});

#  if defined(HAVE_HARDWARE_TRIGGER)
      // 1.5 hardware trigger
	_ddr.registerVariable<bool>(
	    "motioncam.hardware_trigger",
	    _device->MotionCam->HardwareTrigger,
	    std::bind(&Camera::set_field<PhoXiMotionCam, bool>, this,
		      &PhoXi::MotionCam,
		      &PhoXiMotionCam::HardwareTrigger, _1,
		      false, "HardwareTrigger"),
	    "Hardware trigger");

#    if defined(HAVE_HARDWARE_TRIGGER_SIGNAL)
      // 1.6 hardware trigger signal
	_ddr.registerEnumVariable<int>(
	    "motioncam.hardware_trigger_signal",
	    _device->MotionCam->HardwareTriggerSignal,
	    std::bind(&Camera::set_field<PhoXiMotionCam,
		      PhoXiHardwareTriggerSignal>,
		      this,
		      &PhoXi::MotionCam,
		      &PhoXiMotionCam::HardwareTriggerSignal, _1,
		      false, "HardwareTriggerSignal"),
	    "Hardware trigger siganl",
	    {{"Falling",	PhoXiHardwareTriggerSignal::Falling},
	     {"Rising",		PhoXiHardwareTriggerSignal::Rising},
	     {"Both",		PhoXiHardwareTriggerSignal::Both}});
#    endif
#  endif
    }

  // 2. MotionCam camera mode
    if (is_available(_device->MotionCamCameraMode))
    {
      // 2.1 exposure
	std::map<std::string, double>	enum_exposures;
	for (auto exposure :
		 _device->SupportedSinglePatternExposures.GetValue())
	    enum_exposures.emplace(std::to_string(exposure), exposure);
	_ddr.registerEnumVariable<double>(
	    "motioncam_camera_mode.camera_exposure",
	    _device->MotionCamCameraMode->Exposure,
	    std::bind(&Camera::set_field<PhoXiMotionCamCameraMode, double>,
		      this,
		      &PhoXi::MotionCamCameraMode,
		      &PhoXiMotionCamCameraMode::Exposure, _1,
		      false, "CameraExposure"),
	    "Exposure time in miliseconds", enum_exposures);

      // 2,2 sampling topology
	if (_device->MotionCamCameraMode->SamplingTopology !=
	    PhoXiSamplingTopology::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam_camera_mode.sampling_topology",
		_device->MotionCamCameraMode->SamplingTopology,
		std::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					     PhoXiSamplingTopology>,
			  this,
			  &PhoXi::MotionCamCameraMode,
			  &PhoXiMotionCamCameraMode::SamplingTopology, _1,
			  false, "SamplingTopology"),
		"Sampling topology",
		{{"Standard", PhoXiSamplingTopology::Standard}});

      // 2.3 output topology
	if (_device->MotionCamCameraMode->OutputTopology !=
	    PhoXiOutputTopology::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam_camera_mode.output_topology",
		_device->MotionCamCameraMode->OutputTopology,
		std::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					     PhoXiOutputTopology>,
			  this,
			  &PhoXi::MotionCamCameraMode,
			  &PhoXiMotionCamCameraMode::OutputTopology, _1,
			  false, "OutputTopology"),
		"Output topology",
		{{"IrregularGrid",	PhoXiOutputTopology::IrregularGrid},
		 {"Raw",		PhoXiOutputTopology::Raw},
		 {"RegularGrid",	PhoXiOutputTopology::RegularGrid}});

      // 2.4 coding strategy
	if (_device->MotionCamCameraMode->CodingStrategy !=
	    PhoXiCodingStrategy::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam_camera_mode.camera_coding_strategy",
		_device->MotionCamCameraMode->CodingStrategy,
		std::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					     PhoXiCodingStrategy>,
			  this,
			  &PhoXi::MotionCamCameraMode,
			  &PhoXiMotionCamCameraMode::CodingStrategy, _1,
			  false, "CodingStrategy"),
		"Coding strategy",
		{{"Normal",	      PhoXiCodingStrategy::Normal},
		 {"Interreflections", PhoXiCodingStrategy::Interreflections}});

      // 2.5 TextureSource
	if (_device->MotionCamCameraMode->TextureSource !=
	    PhoXiTextureSource::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam_camera_mode.camera_texture_source",
		_device->MotionCamCameraMode->TextureSource,
		std::bind(&Camera::set_texture_source<PhoXiMotionCamCameraMode>,
			  this, &PhoXi::MotionCamCameraMode, _1),
		"Source used for texture image",
		{{"Computed",	PhoXiTextureSource::Computed},
		 {"LED",	PhoXiTextureSource::LED},
		 {"Laser",	PhoXiTextureSource::Laser},
		 {"Focus",	PhoXiTextureSource::Focus},
		 {"Color",	PhoXiTextureSource::Color}});
    }

  // 3. MotionCam scanner mode
    if (is_available(_device->MotionCamScannerMode))
    {
      // 3.1 shutter multiplier
	_ddr.registerVariable<int>(
	    "motioncam_scanner_mode.shutter_multiplier",
	    _device->MotionCamScannerMode->ShutterMultiplier,
	    std::bind(&Camera::set_field<PhoXiMotionCamScannerMode, int>, this,
		      &PhoXi::MotionCamScannerMode,
		      &PhoXiMotionCamScannerMode::ShutterMultiplier, _1,
		      false, "ShutterMultiplier"),
	    "Shutter multiplier", {1, 20});

      // 3.2 scan multiplier
	_ddr.registerVariable<int>(
	    "motioncam_scanner_mode.scan_multiplier",
	    _device->MotionCamScannerMode->ScanMultiplier,
	    std::bind(&Camera::set_field<PhoXiMotionCamScannerMode, int>, this,
		      &PhoXi::MotionCamScannerMode,
		      &PhoXiMotionCamScannerMode::ScanMultiplier, _1,
		      false, "ScanMultiplier"),
	    "Scan multiplier", {1, 20});

      // 3.3 coding strategy
	if (_device->MotionCamScannerMode->CodingStrategy !=
	    PhoXiCodingStrategy::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam_scanner_mode.scanner_coding_strategy",
		_device->MotionCamScannerMode->CodingStrategy,
		std::bind(&Camera::set_field<PhoXiMotionCamScannerMode,
					     PhoXiCodingStrategy>,
			  this,
			  &PhoXi::MotionCamScannerMode,
			  &PhoXiMotionCamScannerMode::CodingStrategy, _1,
			  false, "CodingStrategy"),
		"Coding  strategy",
		{{"Normal",	      PhoXiCodingStrategy::Normal},
		 {"Interreflections", PhoXiCodingStrategy::Interreflections}});

      // 3.4 coding quality
	if (_device->MotionCamScannerMode->CodingQuality !=
	    PhoXiCodingQuality::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam_scanner_mode.coding_quality",
		_device->MotionCamScannerMode->CodingQuality,
		std::bind(&Camera::set_field<PhoXiMotionCamScannerMode,
					     PhoXiCodingQuality>,
			  this,
			  &PhoXi::MotionCamScannerMode,
			  &PhoXiMotionCamScannerMode::CodingQuality, _1,
			  false, "CodingQuality"),
		"Coding quality",
		{{"Fast",	PhoXiCodingQuality::Fast},
		 {"High",	PhoXiCodingQuality::High},
		 {"Ultra",	PhoXiCodingQuality::Ultra}});

      // 3.5 texture source
	if (_device->MotionCamScannerMode->TextureSource !=
	    PhoXiTextureSource::NoValue)
	    _ddr.registerEnumVariable<int>(
		"motioncam_scanner_mode.texture_source",
		_device->MotionCamScannerMode->TextureSource,
		std::bind(&Camera::set_texture_source<PhoXiMotionCamScannerMode>,
			  this, &PhoXi::MotionCamScannerMode, _1),
		"Texture source",
		{{"Computed",	PhoXiTextureSource::Computed},
		 {"LED",	PhoXiTextureSource::LED},
		 {"Laser",	PhoXiTextureSource::Laser},
		 {"Focus",	PhoXiTextureSource::Focus},
		 {"Color",	PhoXiTextureSource::Color}});

      // 3.6 exposure
#  if defined(HAVE_MOTIONCAM_EXPOSURE)
	std::map<std::string, double>	enum_exposures;
	for (auto exposure :
		 _device->SupportedSinglePatternExposures.GetValue())
	    enum_exposures.emplace(std::to_string(exposure), exposure);
	_ddr.registerEnumVariable<double>(
	    "motioncam_scanner_mode.scanner_exposure",
	    _device->MotionCamScannerMode->Exposure,
	    std::bind(&Camera::set_field<PhoXiMotionCamScannerMode, double>,
		      this,
		      &PhoXi::MotionCamScannerMode,
		      &PhoXiMotionCamScannerMode::Exposure, _1,
		      false, "Exposure"),
	    "Exposure", enum_exposures);
#  endif
    }
}
#endif

void
Camera::setup_ddr_common()
{
    using namespace	pho::api;
    using namespace	std::placeholders;

  // 1. TriggerMode
    if (is_available(_device->TriggerMode))
    {
	if (_device->TriggerMode.GetValue() != PhoXiTriggerMode::NoValue)
	    _ddr.registerEnumVariable<int>(
		"trigger_mode", _device->TriggerMode.GetValue(),
		std::bind(&Camera::set_feature<PhoXiTriggerMode, int>, this,
			  &PhoXi::TriggerMode, _1, true),
		"Trigger mode",
		{{"Freerun",	PhoXiTriggerMode::Freerun},
		 {"Software",	PhoXiTriggerMode::Software},
		 {"Hardware",	PhoXiTriggerMode::Hardware}});
    }

  // 2. Timeout
    if (is_available(_device->Timeout))
    {
	_ddr.registerEnumVariable<int>(
	    "timeout",
	    _device->Timeout.GetValue(),
	    std::bind(&Camera::set_feature<PhoXiTimeout, int>, this,
		      &PhoXi::Timeout, _1, false),
	    "Timeout settings",
	    {{"ZeroTimeout",	PhoXiTimeout::ZeroTimeout},
	     {"Infinity",	PhoXiTimeout::Infinity},
	     {"LastStored",	PhoXiTimeout::LastStored},
	     {"Default",	PhoXiTimeout::Default}});
    }

  // 3, ProcessingSettings
    if (is_available(_device->ProcessingSettings))
    {
      // 3.1 ConfidenceValue
	_ddr.registerVariable<double>(
	    "processing_settings.confidence",
	    _device->ProcessingSettings->Confidence,
	    std::bind(&Camera::set_field<PhoXiProcessingSettings, double>,
		      this,
		      &PhoXi::ProcessingSettings,
		      &PhoXiProcessingSettings::Confidence, _1, false,
		      "Confidence"),
	    "Confidence value", {0.0, 100.0});

      // 3.2 SurfaceSmoothness
	if (_device->ProcessingSettings->SurfaceSmoothness !=
	    PhoXiSurfaceSmoothness::NoValue)
	    _ddr.registerEnumVariable<int>(
		"processing_settings.surface_smoothness",
		_device->ProcessingSettings->SurfaceSmoothness,
		std::bind(&Camera::set_field<PhoXiProcessingSettings,
			  PhoXiSurfaceSmoothness>,
			  this,
			  &PhoXi::ProcessingSettings,
			  &PhoXiProcessingSettings::SurfaceSmoothness, _1,
			  false, "SurfaceSmoothness"),
		"Surface smoothness",
		{{"Sharp",	PhoXiSurfaceSmoothness::Sharp},
		 {"Normal",	PhoXiSurfaceSmoothness::Normal},
		 {"Smooth",	PhoXiSurfaceSmoothness::Smooth}});

      // 3.3 CalibrationVolumeOnly
	_ddr.registerVariable<bool>(
	    "processing_settings.calibration_volume_only",
	    _device->ProcessingSettings->CalibrationVolumeOnly,
	    std::bind(&Camera::set_field<PhoXiProcessingSettings, bool>,
		      this,
		      &PhoXi::ProcessingSettings,
		      &PhoXiProcessingSettings::CalibrationVolumeOnly, _1,
		      false, "CalibrationVolumeOnly"),
	    "Calibration volume only");

      // 3.4 NormalsEstimationRadius
	_ddr.registerVariable<int>(
	    "processing_settings.normals_estimation_radius",
	    _device->ProcessingSettings->NormalsEstimationRadius,
	    std::bind(&Camera::set_field<PhoXiProcessingSettings, int>,
		      this,
		      &PhoXi::ProcessingSettings,
		      &PhoXiProcessingSettings::NormalsEstimationRadius, _1,
		      false, "NormalsEstimationRadius"),
	    "Normals estimation radius", {0, 4});

#if defined(HAVE_INTERREFLECTIONS_FILTERING)
      // 3.5 InterreflectionsFiltering
	_ddr.registerVariable<bool>(
	    "processing_settings.interreflections_filtering",
	    _device->ProcessingSettings->InterreflectionsFiltering,
	    std::bind(&Camera::set_field<PhoXiProcessingSettings, bool>,
		      this,
		      &PhoXi::ProcessingSettings,
		      &PhoXiProcessingSettings::InterreflectionsFiltering, _1,
		      false, "InterreflectionsFiltering"),
	    "Interreflections filtering");

#  if defined(HAVE_INTERREFLECTION_FILTER_STRENGTH)
      // 3.6 InterreflectionFilterStrength
	_ddr.registerVariable<double>(
	    "processing_settings.interreflection_filter_strength",
	    _device->ProcessingSettings->InterreflectionFilterStrength,
	    std::bind(&Camera::set_field<PhoXiProcessingSettings, double>,
		      this,
		      &PhoXi::ProcessingSettings,
		      &PhoXiProcessingSettings::InterreflectionFilterStrength,
		      _1, false, "InterreflectionFilterStrength"),
	    "Interreflection filter strength", {0, 4});
#  endif
#endif
    }

#if defined(HAVE_COLOR_CAMERA)
  // 4. ColorSettings
    if (is_available(_device->ColorSettings))
    {
	const auto	color_settings = _device->ColorSettings.GetValue();

      // 4.1 Iso
	const auto	iso = _device->SupportedColorIso.GetValue();
	if (iso.size() > 1)
	{
	    std::map<std::string, int>	enum_iso;
	    size_t			idx = 0;
	    for (size_t i = 0; i < iso.size(); ++i)
	    {
		enum_iso.emplace(std::to_string(iso[i]), iso[i]);
		if (iso[i] == color_settings.Iso)
		    idx = i;
	    }
	    _ddr.registerEnumVariable<int>(
		"color_settings.iso", iso[idx],
		std::bind(&Camera::set_field<PhoXiColorSettings, int>, this,
			  &PhoXi::ColorSettings,
			  &PhoXiColorSettings::Iso, _1, false, "Iso"),
		"Color ISO", enum_iso);
	}

      // 4.2 Exposure
	const auto	exposure = _device->SupportedColorExposure.GetValue();
	if (exposure.size() > 1)
	{
	    std::map<std::string, double>	enum_exposure;
	    size_t				idx = 0;
	    for (size_t i = 0; i < exposure.size(); ++i)
	    {
		enum_exposure.emplace(std::to_string(exposure[i]), exposure[i]);
		if (exposure[i] == color_settings.Exposure)
		    idx = i;
	    }
	    _ddr.registerEnumVariable<double>(
		"color_settings.exposure", exposure[idx],
		std::bind(&Camera::set_field<PhoXiColorSettings, double>,
			  this,
			  &PhoXi::ColorSettings,
			  &PhoXiColorSettings::Exposure, _1,
			  false, "Exposure"),
		"Color exposure", enum_exposure);
	}

      // 4.3 CapturingMode
	const auto modes = _device->SupportedColorCapturingModes.GetValue();
	if (modes.size() > 1)
	{
	    std::map<std::string, int>	enum_resolution;
	    size_t			idx = 0;
	    for (size_t i = 0; i < modes.size(); ++i)
	    {
		const auto&	resolution = modes[i].Resolution;
		enum_resolution.emplace(std::to_string(resolution.Width) + 'x' +
					std::to_string(resolution.Height), i);
		if (modes[i] == color_settings.CapturingMode)
		    idx = i;
	    }
	    _ddr.registerEnumVariable<int>(
		"color_settings.color_resolution", idx,
		std::bind(&Camera::set_color_resolution, this, _1),
		"Color image resolution", enum_resolution);
	}

      // 4.4 Gamma
	_ddr.registerVariable<double>(
	    "color_settings.gamma", color_settings.Gamma,
	    std::bind(&Camera::set_field<PhoXiColorSettings, double>, this,
		      &PhoXi::ColorSettings,
		      &PhoXiColorSettings::Gamma, _1, false, "Gamma"),
	    "Color gamma correction", {0.0, 5.0});

      // 4.5 WhiteBalance
	const auto white_balance_presets
		       = _device->SupportedColorWhiteBalancePresets.GetValue();
	if (white_balance_presets.size() > 1)
	{
	    std::map<std::string, std::string>	enum_presets;
	    std::string				current_preset = "";
	    for (const auto& preset : white_balance_presets)
	    {
		enum_presets.emplace(preset, preset);
		if (preset == color_settings.WhiteBalance.Preset)
		    current_preset = preset;
	    }
	    if (current_preset == "")
	    {
		current_preset = color_settings.WhiteBalance.Preset;
		enum_presets.emplace(current_preset, current_preset);
	    }

	    _ddr.registerEnumVariable<std::string>(
		"color_settings.white_balance", current_preset,
		std::bind(&Camera::set_white_balance_preset, this, _1),
		"White balance", enum_presets);
	}
    }
#endif

  // 5. OutputSettings
    if (is_available(_device->OutputSettings))
    {
	_ddr.registerVariable<bool>(
	    "output_settings.send_point_cloud",
	    _device->OutputSettings->SendPointCloud,
	    std::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
		      &PhoXi::OutputSettings,
		      &FrameOutputSettings::SendPointCloud, _1,
		      true, "SendPointCloud"),
	    "Publish point cloud if set");
	_ddr.registerVariable<bool>(
	    "output_settings.send_normal_map",
	    _device->OutputSettings->SendNormalMap,
	    std::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
		      &PhoXi::OutputSettings,
		      &FrameOutputSettings::SendNormalMap, _1,
		      true, "SendNormalMap"),
	    "Publish normal map if set");
	_ddr.registerVariable<bool>(
	    "output_settings.send_depth_map",
	    _device->OutputSettings->SendDepthMap,
	    std::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
		      &PhoXi::OutputSettings,
		      &FrameOutputSettings::SendDepthMap, _1,
		      true, "SendDepthMap"),
	    "Publish depth map if set");
	_ddr.registerVariable<bool>(
	    "output_settings.send_confidence_map",
	    _device->OutputSettings->SendConfidenceMap,
	    std::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
		      &PhoXi::OutputSettings,
		      &FrameOutputSettings::SendConfidenceMap, _1,
		      true, "SendConfidenceMap"),
	    "Publish confidence map if set");
	_ddr.registerVariable<bool>(
	    "output_settings.send_event_map",
	    _device->OutputSettings->SendEventMap,
	    std::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
		      &PhoXi::OutputSettings,
		      &FrameOutputSettings::SendEventMap, _1,
		      true, "SendEventMap"),
	    "Publish event map if set");
	_ddr.registerVariable<bool>(
	    "output_settings.send_texture",
	    _device->OutputSettings->SendTexture,
	    std::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
		      &PhoXi::OutputSettings,
		      &FrameOutputSettings::SendTexture, _1,
		      true, "SendTexture"),
	    "Publish texture if set");
#if defined(HAVE_COLOR_CAMERA)
	if (_device->SupportedColorCapturingModes->size() > 0)
	    _ddr.registerVariable<bool>(
		"output_settings.send_color_camera_image",
		_device->OutputSettings->SendColorCameraImage,
		std::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			  &PhoXi::OutputSettings,
			  &FrameOutputSettings::SendColorCameraImage, _1,
			  true, "SendColorCameraImage"),
		"Publish color camera image if set");
#endif
    }

  // 6. Density of the cloud
    _ddr.registerVariable<bool>(
	"dense_cloud", _dense_cloud,
	std::bind(&Camera::set_member<bool>, this,
		  std::ref(_dense_cloud), _1, "dense_cloud"),
	"Dense cloud if set");

  // 7. Intensity scale
    _ddr.registerVariable<double>(
	"intensity_scale", _intensity_scale,
	std::bind(&Camera::set_member<double>, this,
		  std::ref(_intensity_scale), _1, "intensity_scale"),
	"Multiplier for intensity values of published texture", {0.05, 5.0});
}

template <class F> bool
Camera::is_available(const pho::api::PhoXiFeature<F>& feature) const
{
    if (feature.isEnabled() && feature.CanGet() && feature.CanSet())
	return true;
    RCLCPP_WARN_STREAM(get_logger(),
		       "feature " << feature.GetName() << " is not available");
    return false;
}

template <class F, class T> void
Camera::set_feature(pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
		    T value, bool suspend)
{
    const auto acq = _device->isAcquiring();
    if (suspend && acq)
	_device->StopAcquisition();

    auto&	f = _device.operator ->()->*feature;
    f.SetValue(value);

    if (f.isLastOperationSuccessful())
	RCLCPP_INFO_STREAM(get_logger(), "set "
			   << f.GetName() << " to " << f.GetValue());
    else
	RCLCPP_ERROR_STREAM(get_logger(), "failed to set "
			    << f.GetName() << " to " << value << ": "
			    << f.GetLastErrorMessage());

    if (suspend)
    {
	_device->ClearBuffer();
	if (acq)
	    _device->StartAcquisition();
    }
}

template <class F, class T> void
Camera::set_field(pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
		  T F::* field, T value, bool suspend,
		  const std::string& field_name)
{
    const auto acq = _device->isAcquiring();
    if (suspend && acq)
	_device->StopAcquisition();

    auto&	f   = _device.operator ->()->*feature;
    auto	val = f.GetValue();
    val.*field = value;
    f.SetValue(val);

    if (f.isLastOperationSuccessful())
	RCLCPP_INFO_STREAM(get_logger(),
			   "set " << f.GetName() << "::" << field_name <<
			   " to " << f.GetValue().*field);
    else
	RCLCPP_ERROR_STREAM(get_logger(), "failed to set "
			    << f.GetName() << "::"
			    << field_name << " to " << value << ": "
			    << f.GetLastErrorMessage());

    if (suspend)
    {
	_device->ClearBuffer();
	if (acq)
	    _device->StartAcquisition();
    }
}

template <class T> void
Camera::set_member(T& member, T value, const std::string& name)
{
    member = value;
    RCLCPP_INFO_STREAM(get_logger(), "set " << name << " to " << member);
}

template <class F> void
Camera::set_texture_source(pho::api::PhoXiFeature<F> pho::api::PhoXi::* feature,
			   pho::api::PhoXiTextureSource texture_source)
{
    using namespace	pho::api;

    set_field(feature, &F::TextureSource, texture_source, true,
	      "TextureSource");

    _color_texture_source = (texture_source == PhoXiTextureSource::Color);
}

void
Camera::set_resolution(size_t idx)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    const auto	modes = _device->SupportedCapturingModes.GetValue();
    if (idx < modes.size())
    {
	_device->CapturingMode = modes[idx];

	if (_device->CapturingMode.isLastOperationSuccessful())
	    RCLCPP_INFO_STREAM(
		get_logger(), "set resolution to "
		<< _device->CapturingMode.GetValue().Resolution.Width
		<< 'x'
		<< _device->CapturingMode.GetValue().Resolution.Height);
	else
	    RCLCPP_ERROR_STREAM(
		get_logger(), "failed to set resolution to "
		<< modes[idx].Resolution.Width << 'x'
		<< modes[idx].Resolution.Height << ": "
		<< _device->CapturingMode.GetLastErrorMessage());
    }
    else
	RCLCPP_ERROR_STREAM(get_logger(), "index["
			    << idx << "] of CapturingMode is out of range");

    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();
}

#if defined(HAVE_COLOR_CAMERA)
void
Camera::set_color_resolution(size_t idx)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    const auto modes = _device->SupportedColorCapturingModes.GetValue();
    if (idx < modes.size())
    {
	_device->ColorSettings->CapturingMode = modes[idx];

	if (_device->CapturingMode.isLastOperationSuccessful())
	    RCLCPP_INFO_STREAM(
		get_logger(), "set color resolution to "
		<< _device->ColorSettings->CapturingMode.Resolution.Width
		<< 'x'
		<< _device->ColorSettings->CapturingMode.Resolution.Height);
	else
	    RCLCPP_ERROR_STREAM(
		get_logger(), "failed to set color resolution to "
		<< modes[idx].Resolution.Width << 'x'
		<< modes[idx].Resolution.Height << ": "
		<< _device->ColorSettings.GetLastErrorMessage());
    }
    else
	RCLCPP_ERROR_STREAM(get_logger(), "index["
			    << idx
			    << "] of color CapturingMode is out of range");

    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();
}

void
Camera::set_white_balance_preset(const std::string& preset)
{
    _device->ColorSettings->WhiteBalance.Preset = preset;
    RCLCPP_INFO_STREAM(get_logger(), "set white balance to "
		       << _device->ColorSettings->WhiteBalance.Preset);
}
#endif

bool
Camera::trigger_frame(const trigger_req_p, trigger_res_p res)
{
    using namespace	pho::api;

    RCLCPP_INFO_STREAM(get_logger(), "trigger_frame: service requested");

    if (_device->TriggerMode.GetValue() != PhoXiTriggerMode::Software)
    {
	res->success = true;
	res->message = "succeded, but device is not in software trigger mode";

	RCLCPP_WARN_STREAM(get_logger(), res->message);
	return true;
    }

    const auto	frameId = _device->TriggerFrame(true, true);

    RCLCPP_INFO_STREAM(get_logger(), "trigger_frame: triggered");

    switch (frameId)
    {
      case -1:
	res->success = false;
	res->message = "failed. [TriggerFrame not accepted]";
	break;
      case -2:
	res->success = false;
	res->message = "failed. [device is not running]";
	break;
      case -3:
	res->success = false;
	res->message = "failed. [communication error]";
	break;
      case -4:
	res->success = false;
	res->message = "failed. [WaitForGrabbingEnd is not supported]";
	break;
      default:
	if (!(_frame = _device->GetSpecificFrame(frameId,
						 PhoXiTimeout::Infinity)))
	{
	    res->success = false;
	    res->message = "failed. [not found frame #"
			 + std::to_string(frameId) + ']';
	    break;
	}

	RCLCPP_INFO_STREAM(get_logger(), "trigger_frame: frame got");

	if (_frame->Info.FrameIndex != uint64_t(frameId))
	{
	    res->success = false;
	    res->message = "failed. [triggered frame(#"
			 + std::to_string(frameId)
			 + ") is not captured frame(#"
			 + std::to_string(_frame->Info.FrameIndex)
			 + ")]";
	    break;
	}

	publish_frame();	// publish
	res->success = true;
	res->message = "succeeded. [frame #" + std::to_string(frameId) + ']';
	break;
    }

    if (res->success)
	RCLCPP_INFO_STREAM(get_logger(), "trigger_frame: " << res->message);
    else
	RCLCPP_ERROR_STREAM(get_logger(), "trigger_frame: " << res->message);

    return true;
}

bool
Camera::save_settings(const trigger_req_p, trigger_res_p res)
{
    res->success = _device->SaveSettings();

    if (res->success)
    {
	res->message = "succesfully saved settings";
	RCLCPP_INFO_STREAM(get_logger(), "save_settings: " << res->message);
    }
    else
    {
	res->message = "failed to save settings";
	RCLCPP_ERROR_STREAM(get_logger(), "save_settings: " << res->message);
    }

    return true;
}

bool
Camera::restore_settings(const trigger_req_p, trigger_res_p res)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    res->success = _device->ResetActivePreset();

    if (res->success)
    {
	res->message = "succesfully restored settings";
	RCLCPP_INFO_STREAM(get_logger(), "restore_settings: " << res->message);
    }
    else
    {
	res->message = "failed to restore settings";
	RCLCPP_ERROR_STREAM(get_logger(), "restore_settings: " << res->message);
    }

    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();

    return true;
}

template <class T> void
Camera::set_image(const image_p& image,
		  const rclcpp::Time& stamp, const std::string& frame_id,
		  const std::string& encoding, float scale,
		  const pho::api::Mat2D<T>& phoxi_image)
{
    using namespace	sensor_msgs;
    using		element_ptr = const typename T::ElementChannelType*;

    image->header.stamp    = stamp;
    image->header.frame_id = frame_id;
    image->encoding	   = encoding;
    image->is_bigendian    = 0;
    image->height	   = phoxi_image.Size.Height;
    image->width	   = phoxi_image.Size.Width;
    image->step		   = image->width
			   *  image_encodings::numChannels(image->encoding)
			   * (image_encodings::bitDepth(image->encoding)/8);
    image->data.resize(image->step * image->height);

    const auto	p = reinterpret_cast<element_ptr>(phoxi_image[0]);
    const auto	q = reinterpret_cast<element_ptr>(phoxi_image[
						      phoxi_image.Size.Height]);
    if (image->encoding == image_encodings::MONO8)
	scale_copy(p, q, image->data.data(), scale);
    else if (image->encoding == image_encodings::RGB8)
	scale_copy(p, q, image->data.data(), scale);
    else if (image->encoding.substr(0, 4) == "32FC")
	scale_copy(p, q, reinterpret_cast<float*>(image->data.data()), scale);
    else
    {
	RCLCPP_ERROR_STREAM(get_logger(), "unsupported image type!");
	throw;
    }
}

void
Camera::set_camera_matrix()
{
    using namespace	pho::api;

    constexpr static size_t	PhoXiNativeWidth	= 2064;
    constexpr static size_t	PhoXiNativeHeight	= 1544;
    constexpr static size_t	MotionCamNativeWidth	= 1680;
    constexpr static size_t	MotionCamNativeHeight	= 1200;

    const auto	camera_matrix = _device->CalibrationSettings->CameraMatrix;
    const bool	isMotionCam   = (PhoXiDeviceType::Value(_device->GetType()) ==
				 PhoXiDeviceType::MotionCam3D);
    const auto	scale_u	      = double(_frame->DepthMap.Size.Width)
			      / double(isMotionCam ? MotionCamNativeWidth
						   : PhoXiNativeWidth);
    const auto	scale_v       = double(_frame->DepthMap.Size.Height)
			      / double(isMotionCam ? MotionCamNativeHeight
						   : PhoXiNativeHeight);
    _camera_matrix[0][0] = scale_u * camera_matrix[0][0];
    _camera_matrix[0][1] = scale_u * camera_matrix[0][1];
    _camera_matrix[0][2] = scale_u * camera_matrix[0][2];
    _camera_matrix[1][0] = scale_v * camera_matrix[1][0];
    _camera_matrix[1][1] = scale_v * camera_matrix[1][1];
    _camera_matrix[1][2] = scale_v * camera_matrix[1][2];
    _camera_matrix[2][0] =	     camera_matrix[2][0];
    _camera_matrix[2][1] =	     camera_matrix[2][1];
    _camera_matrix[2][2] =	     camera_matrix[2][2];
}

void
Camera::set_camera_info(const camera_info_p& camera_info,
			const rclcpp::Time& stamp, const std::string& frame_id,
			size_t width, size_t height,
			const pho::api::CameraMatrix64f& K,
			const std::vector<double>& D,
			const pho::api::Point3_64f& t,
			const pho::api::Point3_64f& rx,
			const pho::api::Point3_64f& ry,
			const pho::api::Point3_64f& rz)
{
  // Set header.
    camera_info->header.stamp	   = stamp;
    camera_info->header.frame_id = frame_id;

  // Set size.
    camera_info->width  = width;
    camera_info->height = height;

  // Set distortion parameters.
    camera_info->distortion_model = "plumb_bob";
    camera_info->d.resize(5);
    std::fill(std::begin(camera_info->d), std::end(camera_info->d), 0.0);
    std::copy_n(std::begin(D), std::min(std::size(camera_info->d), std::size(D)),
    		std::begin(camera_info->d));

  // Set intrinsic parameters.
    camera_info->k[0] = K[0][0];
    camera_info->k[1] = K[0][1];
    camera_info->k[2] = K[0][2];
    camera_info->k[3] = K[1][0];
    camera_info->k[4] = K[1][1];
    camera_info->k[5] = K[1][2];
    camera_info->k[6] = K[2][0];
    camera_info->k[7] = K[2][1];
    camera_info->k[8] = K[2][2];

  // Set rotation matrix.
    camera_info->r[0] = rx.x;
    camera_info->r[1] = rx.y;
    camera_info->r[2] = rx.z;
    camera_info->r[3] = ry.x;
    camera_info->r[4] = ry.y;
    camera_info->r[5] = ry.z;
    camera_info->r[6] = rz.x;
    camera_info->r[7] = rz.y;
    camera_info->r[8] = rz.z;

  // Set 3x4 camera matrix.
    constexpr static double	scale = 0.001;	// milimeters ==> meters
    const double		T[] = {scale * t.x, scale * t.y, scale * t.z};
    for (int i = 0; i < 3; ++i)
    {
	camera_info->p[4*i + 3] = 0;

	for (int j = 0; j < 3; ++j)
	{
	    camera_info->p[4*i + j] = 0;

	    for (int k = 0; k < 3; ++k)
		camera_info->p[4*i + j] += camera_info->k[3*i + k] * camera_info->r[3*k + j];

	    camera_info->p[4*i + 3] -= camera_info->p[4*i + j] * T[j];
	}
    }

  // No binning
    camera_info->binning_x = camera_info->binning_y = 0;

  // ROI is same as entire image.
    camera_info->roi.width = camera_info->roi.height = 0;
}

void
Camera::publish_frame()
{
    using	namespace sensor_msgs;
    using	duration_t = std::chrono::duration<double, std::milli>;

  // Common setting.
    const auto	stamp = get_clock()->now()
		      - duration_t(_frame->Info.FrameDuration +
				   _frame->Info.FrameComputationDuration +
				   _frame->Info.FrameTransferDuration);

  // Publish point cloud.
    profiler_start(0);
    constexpr float	distanceScale = 0.001;	// milimeters -> meters
    publish_cloud(stamp, distanceScale);

  // Publish normal_map, depth_map, confidence_map, event_map and texture.
    profiler_start(1);
    publish_image(_normal_map, stamp, image_encodings::TYPE_32FC3,
		  1, _frame->NormalMap, _normal_map_pub);
    profiler_start(2);
    publish_image(_depth_map, stamp, image_encodings::TYPE_32FC1,
		  distanceScale, _frame->DepthMap, _depth_map_pub);
    profiler_start(3);
    publish_image(_confidence_map, stamp, image_encodings::TYPE_32FC1,
		  1, _frame->ConfidenceMap, _confidence_map_pub);
    profiler_start(4);
    publish_image(_event_map, stamp, image_encodings::TYPE_32FC1,
		  1, _frame->EventMap, _event_map_pub);
    profiler_start(5);
#if defined(HAVE_COLOR_CAMERA)
    if (_color_texture_source)
	publish_image(_texture, stamp, image_encodings::RGB8,
		      _intensity_scale, _frame->TextureRGB, _texture_pub);
    else
#endif
	publish_image(_texture, stamp, image_encodings::MONO8,
		      _intensity_scale, _frame->Texture, _texture_pub);

  // Publish camera_info.
    profiler_start(6);
    publish_camera_info(stamp);

  // Publish color_camera.
    profiler_start(7);
#if defined(HAVE_COLOR_CAMERA)
    publish_color_camera(stamp);
#endif

    profiler_print(std::cerr);
    RCLCPP_DEBUG_STREAM(get_logger(), "frame published [#"
			<< _frame->Info.FrameIndex << ']');
}

void
Camera::publish_cloud(const rclcpp::Time& stamp, float distanceScale)
{
    using namespace	sensor_msgs;
    using PointField =  sensor_msgs::msg::PointField;

    const auto&	phoxi_cloud = _frame->PointCloud;
    if (!_device->OutputSettings->SendPointCloud || phoxi_cloud.Empty())
	return;

  // Convert pho::api::PointCloud32f to sensor_msgs::PointCloud2
    _cloud->header.stamp    = stamp;
    _cloud->header.frame_id = _frame_id;
    _cloud->is_bigendian    = false;
    _cloud->is_dense	    = _dense_cloud;

    PointCloud2Modifier	modifier(*_cloud);
    if (_device->OutputSettings->SendTexture)
	if (_device->OutputSettings->SendNormalMap)
	    modifier.setPointCloud2Fields(7,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32,
					  "rgb",      1, PointField::UINT32,
					  "normal_x", 1, PointField::FLOAT32,
					  "normal_y", 1, PointField::FLOAT32,
					  "normal_z", 1, PointField::FLOAT32);
	else
	    modifier.setPointCloud2Fields(4,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32,
					  "rgb",      1, PointField::UINT32);
    else
	if (_device->OutputSettings->SendNormalMap)
	    modifier.setPointCloud2Fields(6,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32,
					  "normal_x", 1, PointField::FLOAT32,
					  "normal_y", 1, PointField::FLOAT32,
					  "normal_z", 1, PointField::FLOAT32);
	else
	    modifier.setPointCloud2Fields(3,
					  "x",	      1, PointField::FLOAT32,
					  "y",	      1, PointField::FLOAT32,
					  "z",	      1, PointField::FLOAT32);

    if (_cloud->is_dense)
    {
	const auto	npoints = npoints_valid(phoxi_cloud);
	modifier.resize(npoints);
	_cloud->height = 1;
	_cloud->width  = npoints;
    }
    else
    {
	modifier.resize(phoxi_cloud.Size.Height * phoxi_cloud.Size.Width);
	_cloud->height = phoxi_cloud.Size.Height;
	_cloud->width  = phoxi_cloud.Size.Width;
    }
    _cloud->row_step = _cloud->width * _cloud->point_step;

    PointCloud2Iterator<float>	xyz(*_cloud, "x");

    for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
    {
	auto	p = phoxi_cloud[v];

	for (const auto q = p + phoxi_cloud.Size.Width; p != q; ++p)
	{
	    if (float(p->z) == 0.0f)
	    {
		if (!_cloud->is_dense)
		{
		    xyz[0] = xyz[1] = xyz[2]
			   = std::numeric_limits<float>::quiet_NaN();
		    ++xyz;
		}
	    }
	    else
	    {
		xyz[0] = distanceScale * float(p->x);
		xyz[1] = distanceScale * float(p->y);
		xyz[2] = distanceScale * float(p->z);
		++xyz;
	    }
	}
    }


    if (_device->OutputSettings->SendTexture)
    {
	PointCloud2Iterator<uint8_t> bgr(*_cloud, "rgb");
#if defined(HAVE_COLOR_CAMERA)
	if (_color_texture_source)
	{
	    for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
	    {
		auto	p = phoxi_cloud[v];
		auto	r = _frame->TextureRGB[v];

		for (const auto q = p + phoxi_cloud.Size.Width;
		     p != q; ++p, ++r)
		{
		    if (!_cloud->is_dense || float(p->z) != 0.0f)
		    {
			bgr[0] = uint8_t(std::min(_intensity_scale * r->b,
						  255.0));
			bgr[1] = uint8_t(std::min(_intensity_scale * r->g,
						  255.0));
			bgr[2] = uint8_t(std::min(_intensity_scale * r->r,
						  255.0));
			++bgr;
		    }
		}
	    }
	}
	else
#endif
	{
	    for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
	    {
		auto	p = phoxi_cloud[v];
		auto	r = _frame->Texture[v];

		for (const auto q = p + phoxi_cloud.Size.Width;
		     p != q; ++p, ++r)
		{
		    if (!_cloud->is_dense || float(p->z) != 0.0f)
		    {
			bgr[0] = bgr[1] = bgr[2]
			       = uint8_t(std::min(_intensity_scale*(*r),
						  255.0));
			++bgr;
		    }
		}
	    }
	}
    }

    if (_device->OutputSettings->SendNormalMap)
    {
	if (_device->ProcessingSettings->NormalsEstimationRadius == 0)
	{
	    RCLCPP_ERROR_STREAM(get_logger(),
				"normals_estimation_radius must be positive");
	    return;
	}

	PointCloud2Iterator<float>	normal(*_cloud, "normal_x");

	for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
	{
	    auto	p = phoxi_cloud[v];
	    auto	r = _frame->NormalMap[v];

	    for (const auto q = p + phoxi_cloud.Size.Width; p != q; ++p, ++r)
	    {
		if (!_cloud->is_dense || float(p->z) != 0.0f)
		{
		    normal[0] = r->x;
		    normal[1] = r->y;
		    normal[2] = r->z;
		    ++normal;
		}
	    }
	}
    }

    _cloud_pub->publish(*_cloud);
}

template <class T> void
Camera::publish_image(const image_p& image, const rclcpp::Time& stamp,
		      const std::string& encoding, float scale,
		      const pho::api::Mat2D<T>& phoxi_image,
		      const image_transport::Publisher& publisher)
{
    if (publisher.getNumSubscribers() == 0 || phoxi_image.Empty())
	return;

    set_image(image, stamp, _frame_id, encoding, scale, phoxi_image);
    publisher.publish(image);
}

void
Camera::publish_camera_info(const rclcpp::Time& stamp)
{
  // We have to extract the camera matrix from
  // _device->CalibrationSettings->CameraMatrix instead of
  // _frame->Info.CameraMatrix because the latter becomes empty
  // for non-reqular pointclooud topology. As accessing the former
  // is time-consuming, the extracted matrix is cached.
    if (_frame->DepthMap.Size.Width  != int(_camera_info->width) ||
    	_frame->DepthMap.Size.Height != int(_camera_info->height))
    	set_camera_matrix();

    set_camera_info(_camera_info, stamp, _frame_id,
		    _frame->DepthMap.Size.Width,
		    _frame->DepthMap.Size.Height,
		    _camera_matrix,
		    _frame->Info.DistortionCoefficients,
		    _frame->Info.SensorPosition,
		    _frame->Info.SensorXAxis,
		    _frame->Info.SensorYAxis,
		    _frame->Info.SensorZAxis);
    _camera_info_pub->publish(*_camera_info);
}

#if defined(HAVE_COLOR_CAMERA)
void
Camera::publish_color_camera(const rclcpp::Time& stamp)
{
    if (_color_camera_pub.getNumSubscribers() == 0 ||
	!_device->OutputSettings->SendColorCameraImage)
	return;

    set_image(_color_camera_image, stamp, _color_camera_frame_id,
	      sensor_msgs::image_encodings::RGB8, _intensity_scale,
	      _frame->ColorCameraImage);

    if (_frame->ColorCameraImage.Size.Width
	    != int(_color_camera_camera_info->width) ||
	_frame->ColorCameraImage.Size.Height
	    != int(_color_camera_camera_info->height))
    {
	set_camera_info(_color_camera_camera_info, stamp,
			_color_camera_frame_id,
			_frame->ColorCameraImage.Size.Width,
			_frame->ColorCameraImage.Size.Height,
			_frame->Info.ColorCameraMatrix,
			_frame->Info.ColorCameraDistortionCoefficients,
			_frame->Info.ColorCameraPosition,
			_frame->Info.ColorCameraXAxis,
			_frame->Info.ColorCameraYAxis,
			_frame->Info.ColorCameraZAxis);

	constexpr static double			s = 0.001;
	geometry_msgs::msg::TransformStamped	transform;
	transform.header.stamp    = stamp;
	transform.header.frame_id = _frame_id;
	transform.child_frame_id  = _color_camera_frame_id;
	transform.transform
	    = tf2::toMsg(
		tf2::Transform({_frame->Info.ColorCameraXAxis.x,
				_frame->Info.ColorCameraYAxis.x,
				_frame->Info.ColorCameraZAxis.x,
				_frame->Info.ColorCameraXAxis.y,
				_frame->Info.ColorCameraYAxis.y,
				_frame->Info.ColorCameraZAxis.y,
				_frame->Info.ColorCameraXAxis.z,
				_frame->Info.ColorCameraYAxis.z,
				_frame->Info.ColorCameraZAxis.z},
			       {s * _frame->Info.ColorCameraPosition.x,
				s * _frame->Info.ColorCameraPosition.y,
				s * _frame->Info.ColorCameraPosition.z}));
	_static_broadcaster.sendTransform(transform);
    }

    _color_camera_pub.publish(_color_camera_image, _color_camera_camera_info);

}
#endif
}	// namespace aist_phoxi_camera

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_phoxi_camera::Camera)
