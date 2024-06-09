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
#          define HAVE_LED_SHUTTER_MULTIPLIER
#	   if PHO_SOFTWARE_VERSION_MINOR >= 9
#            define HAVE_COLOR_CAMERA
#          endif
#        endif
#      endif
#    endif
#  endif
#endif

#include "Camera.h"
#include <ros/package.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nodelet/nodelet.h>
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
Camera::Camera(ros::NodeHandle& nh, const std::string& nodelet_name)
    :
#if defined(PROFILE)
     profiler_t(8),
#endif
     _nodelet_name(nodelet_name),
     _factory(),
     _device(nullptr),
     _frame(nullptr),
     _depth_map_size(0, 0),
     _color_camera_image_size(0, 0),
     _camera_matrix(pho::api::PhoXiSize(3, 3)),
     _ddr(nh),
     _frame_id(nh.param<std::string>("frame", getBaseName() + "_sensor")),
     _color_camera_frame_id(nh.param<std::string>(
				"color_camera_frame",
				getBaseName() + "_color_sensor")),
     _rate(nh.param<double>("rate", 20.0)),
     _intensity_scale(0.5),
     _dense_cloud(false),
     _color_texture_source(false),
     _trigger_frame_server(nh.advertiseService("trigger_frame",
					       &Camera::trigger_frame,	this)),
     _save_settings_server(nh.advertiseService("save_settings",
					       &Camera::save_settings, this)),
     _restore_settings_server(nh.advertiseService("restore_settings",
						  &Camera::restore_settings,
						  this)),
     _it(nh),
     _cloud_pub(	 nh.advertise<cloud_t>("pointcloud",	1)),
     _normal_map_pub(    _it.advertise("normal_map",		1)),
     _depth_map_pub(     _it.advertise("depth_map",		1)),
     _confidence_map_pub(_it.advertise("confidence_map",	1)),
     _event_map_pub(     _it.advertise("event_map",		1)),
     _texture_pub(       _it.advertise("texture",		1)),
     _camera_info_pub(    nh.advertise<camera_info_t>("camera_info",	1)),
     _color_camera_pub(  _it.advertiseCamera("color/image",	1)),
     _static_broadcaster()
{
    using namespace	pho::api;

  // Check existence of PhoXiControl.
    if (!_factory.isPhoXiControlRunning())
    {
	NODELET_ERROR_STREAM('('
			     << getBaseName()
			     << ") PhoXiControll is not running");
	throw;
    }

  // Load camera ID from the parameter.
    auto	id = nh.param<std::string>("id",
					   "InstalledExamples-basic-example");
    for (size_t pos; (pos = id.find('\"')) != std::string::npos; )
	id.erase(pos, 1);

    if (id == "")
    {
	_device = _factory.CreateAndConnectFirstAttached();
	if (!_device)
	{
	    NODELET_ERROR_STREAM('('
				 << getBaseName()
				 << ") Failed to connect with camera first attached to PhoXiControl");
	    throw;
	}
    }
    else
    {
      // Search for a device with specified ID.
	for (const auto& devinfo : _factory.GetDeviceList())
	    if (devinfo.HWIdentification == id)
	    {
		_device = _factory.Create(devinfo.GetTypeHWIdentification());
		break;
	    }
	if (!_device)
	{
	    NODELET_ERROR_STREAM('('
				 << getBaseName()
				 << ") Failed to find camera[" << id << ']');
	    throw;
	}

      // Connect to the device.
	if (!_device->Connect())
	{
	    NODELET_ERROR_STREAM('('
				 << getBaseName()
				 << ") Failed to connect with camera[" << id
				 << ']');
	    throw;
	}
    }

  // Stop acquisition.
    _device->StopAcquisition();

    NODELET_INFO_STREAM('('
			<< getBaseName()
			<< ") Initializing configuration");

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
	NODELET_ERROR_STREAM('('
			     << getBaseName()
			     << ") Unknown device type["
			     << std::string(_device->GetType()) << ']');
	throw;
    }

    setup_ddr_common();

    _ddr.publishServicesTopicsAndUpdateConfigData();

  // Start acquisition.
    _device->ClearBuffer();
    _device->StartAcquisition();

    NODELET_INFO_STREAM('('
			<< getBaseName()
			<< ") aist_phoxi_camera is active");
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

double
Camera::rate() const
{
    return _rate;
}

void
Camera::setup_ddr_phoxi()
{
    using namespace	pho::api;

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
				       boost::bind(&Camera::set_resolution,
						   this, _1),
				       "Image resolution", enum_resolution);
    }

  // 2. CapturingSettings
    if (is_available(_device->CapturingSettings))
    {
      // 2.1 ShutterMultiplier
	_ddr.registerVariable<int>(
	    "shutter_multiplier",
	    _device->CapturingSettings->ShutterMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::ShutterMultiplier, _1,
			false, "ShutterMultiplier"),
	    "The number of repeats of indivisual pattern",
	    1, 20, "capturing_settings");

      // 2.2 ScanMultiplier
	_ddr.registerVariable<int>(
	    "scan_multiplier",
	    _device->CapturingSettings->ScanMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::ScanMultiplier, _1,
			false, "ScanMultiplier"),
	    "The number of scans taken and merged to sigle output",
	    1, 20, "capturing_settings");

      // 2.3 CameraOnlyMode
	_ddr.registerVariable<bool>(
	    "camera_only_mode",
	    _device->CapturingSettings->CameraOnlyMode,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::CameraOnlyMode, _1, false,
			"CameraOnlyMode"),
	    "Use the scanner internal camera to capture only 2D images",
	    false, true, "capturing_settings");

      // 2.4 AmbientLightSuppression
	_ddr.registerVariable<bool>(
	    "ambient_light_suppression",
	    _device->CapturingSettings->AmbientLightSuppression,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::AmbientLightSuppression, _1,
			false, "AmbientLightSuppression"),
	    "Enables the mode that suppress ambient illumination",
	    false, true, "capturing_settings");

      // 2.5 MaximumFPS
	_ddr.registerVariable<double>(
	    "maximum_fps",
	    _device->CapturingSettings->MaximumFPS,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, double>,
			this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::MaximumFPS, _1,
			false, "MaximumFPS"),
	    "Maximum fps in freerun mode",
	    0.0, 30.0, "capturing_settings");

      // 2.6 SinglePatternExposure
	const auto	exposures = _device->SupportedSinglePatternExposures.GetValue();
	std::map<std::string, double>	enum_single_pattern_exposure;
	for (const auto& exposure: exposures)
	    enum_single_pattern_exposure.emplace(std::to_string(exposure),
						 exposure);
	_ddr.registerEnumVariable<double>(
	    "single_pattern_exposure",
	    _device->CapturingSettings->SinglePatternExposure,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, double>,
			this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::SinglePatternExposure, _1,
			false, "SinglePatternExposure"),
	    "Exposure time for a single patter in miliseconds",
	    enum_single_pattern_exposure, "", "capturing_settings");

      // 2.7 CodingStrategy
	if (_device->CapturingSettings->CodingStrategy !=
	    PhoXiCodingStrategy::NoValue)
	    _ddr.registerEnumVariable<int>(
		"coding_strategy",
		_device->CapturingSettings->CodingStrategy,
		boost::bind(&Camera::set_field<PhoXiCapturingSettings,
					       PhoXiCodingStrategy>,
			    this,
			    &PhoXi::CapturingSettings,
			    &PhoXiCapturingSettings::CodingStrategy, _1, false,
			    "CodingStrategy"),
		"Coding strategy",
		{{"Normal",		PhoXiCodingStrategy::Normal},
		 {"Interreflections",	PhoXiCodingStrategy::Interreflections}},
		"", "capturing_settings");

      // 2.8 CodingQuality
	if (_device->CapturingSettings->CodingQuality !=
	    PhoXiCodingQuality::NoValue)
	    _ddr.registerEnumVariable<int>(
		"coding_quality",
		_device->CapturingSettings->CodingQuality,
		boost::bind(&Camera::set_field<PhoXiCapturingSettings,
					       PhoXiCodingQuality>,
			    this,
			    &PhoXi::CapturingSettings,
			    &PhoXiCapturingSettings::CodingQuality, _1, false,
			    "CodingQuality"),
		"Coding quality",
		{{"Fast",	PhoXiCodingQuality::Fast},
		 {"High",	PhoXiCodingQuality::High},
		 {"Ultra",	PhoXiCodingQuality::Ultra}},
		"", "capturing_settings");

      // 2.9 TextureSource
	if (_device->CapturingSettings->TextureSource !=
	    PhoXiTextureSource::NoValue)
	    _ddr.registerEnumVariable<int>(
		"texture_source",
		_device->CapturingSettings->TextureSource,
		boost::bind(&Camera::set_texture_source<PhoXiCapturingSettings>,
			    this, &PhoXi::CapturingSettings, _1),
		"Source used for texture image",
		{{"Computed",	PhoXiTextureSource::Computed},
		 {"LED",	PhoXiTextureSource::LED},
		 {"Laser",	PhoXiTextureSource::Laser},
		 {"Focus",	PhoXiTextureSource::Focus},
		 {"Color",	PhoXiTextureSource::Color}},
		"", "capturing_settings");

      // 2.10 LaserPower
	_ddr.registerVariable<int>(
	    "laser_power",
	    _device->CapturingSettings->LaserPower,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::LaserPower, _1, false,
			"LaserPower"),
	    "Laser power", 1, 4095, "capturing_settings");

      // 2.11 LEDPower
	_ddr.registerVariable<int>(
	    "led_power",
	    _device->CapturingSettings->LEDPower,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::LEDPower, _1, false,
			"LEDPower"),
	    "LED power", 0, 4095, "capturing_settings");

#if defined(HAVE_HARDWARE_TRIGGER)
      // 2.12 hardware trigger
	_ddr.registerVariable<bool>(
	    "hardware_trigger",
	    _device->CapturingSettings->HardwareTrigger,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, bool>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::HardwareTrigger, _1, false,
			"HardwareTrigger"),
	    "Hardware trigger", false, true, "capturing_settings");

#  if defined(HAVE_HARDWARE_TRIGGER_SIGNAL)
      // 2.13 hardware trigger signal
	_ddr.registerEnumVariable<int>(
    	    "hardware_trigger_signal",
    	    _device->CapturingSettings->HardwareTriggerSignal,
    	    boost::bind(&Camera::set_field<PhoXiCapturingSettings,
					   PhoXiHardwareTriggerSignal>,
			this,
    			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::HardwareTriggerSignal,
			_1, false,
			"HardwareTriggerSignal"),
    	    "Hardware trigger siganl",
	    {{"Falling",	PhoXiHardwareTriggerSignal::Falling},
	     {"Rising",		PhoXiHardwareTriggerSignal::Rising},
	     {"Both",		PhoXiHardwareTriggerSignal::Both}},
	    "", "capturing_settings");
#  endif
#endif
#if defined(HAVE_LED_SHUTTER_MULTIPLIER)
      // 2.14 LEDShutterMultiplier
	_ddr.registerVariable<int>(
	    "led_shutter_multiplier",
	    _device->CapturingSettings->LEDShutterMultiplier,
	    boost::bind(&Camera::set_field<PhoXiCapturingSettings, int>, this,
			&PhoXi::CapturingSettings,
			&PhoXiCapturingSettings::LEDShutterMultiplier, _1,
			false, "LEDShutterMultiplier"),
	    "LED shutter multiplier", 1, 20, "capturing_settings");
#endif
    }
}

#if defined(HAVE_MOTIONCAM)
void
Camera::setup_ddr_motioncam()
{
    using namespace	pho::api;

  // 1. General settings
    if (is_available(_device->MotionCam))
    {
      // 1.1 operation mode
	if (_device->MotionCam->OperationMode != PhoXiOperationMode::NoValue)
	    _ddr.registerEnumVariable<int>(
		"operation_mode",
		_device->MotionCam->OperationMode,
		boost::bind(&Camera::set_field<PhoXiMotionCam,
					       PhoXiOperationMode>, this,
			    &PhoXi::MotionCam,
			    &PhoXiMotionCam::OperationMode, _1,
			    true, "OperationMode"),
		"Operation mode",
		{{"Camera",	PhoXiOperationMode::Camera},
		 {"Scanner",	PhoXiOperationMode::Scanner},
		 {"Mode2D",	PhoXiOperationMode::Mode2D}},
		"", "motioncam");

      // 1.2 laser power
	_ddr.registerVariable<int>(
	    "laser_power",
	    _device->MotionCam->LaserPower,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, int>, this,
			&PhoXi::MotionCam, &PhoXiMotionCam::LaserPower, _1,
			false, "LaserPower"),
	    "Laser power",
	    1, 4095, "motioncam");

#if defined(HAVE_LED_POWER)
      // 1.3 led power
	_ddr.registerVariable<int>(
	    "led_power",
	    _device->MotionCam->LEDPower,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, int>, this,
			&PhoXi::MotionCam, &PhoXiMotionCam::LEDPower, _1,
			false, "LEDPower"),
	    "LED power",
	    0, 4095, "motioncam");
#endif

      // 1.4 maximum fps
	_ddr.registerVariable<double>(
	    "maximum_fps",
	    _device->MotionCam->MaximumFPS,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, double>, this,
			&PhoXi::MotionCam, &PhoXiMotionCam::MaximumFPS, _1,
			false, "MaximumFPS"),
	    "Maximum fps",
	    0.0, 20.0, "motioncam");

#  if defined(HAVE_HARDWARE_TRIGGER)
      // 1.5 hardware trigger
	_ddr.registerVariable<bool>(
	    "hardware_trigger",
	    _device->MotionCam->HardwareTrigger,
	    boost::bind(&Camera::set_field<PhoXiMotionCam, bool>, this,
			&PhoXi::MotionCam,
			&PhoXiMotionCam::HardwareTrigger, _1, false,
			"HardwareTrigger"),
	    "Hardware trigger",
	    false, true, "motioncam");

#    if defined(HAVE_HARDWARE_TRIGGER_SIGNAL)
      // 1.6 hardware trigger signal
	_ddr.registerEnumVariable<int>(
    	    "hardware_trigger_signal",
    	    _device->MotionCam->HardwareTriggerSignal,
    	    boost::bind(&Camera::set_field<PhoXiMotionCam,
					   PhoXiHardwareTriggerSignal>,
			this,
    			&PhoXi::MotionCam,
			&PhoXiMotionCam::HardwareTriggerSignal, _1, false,
			"HardwareTriggerSignal"),
    	    "Hardware trigger siganl",
	    {{"Falling",	PhoXiHardwareTriggerSignal::Falling},
	     {"Rising",		PhoXiHardwareTriggerSignal::Rising},
	     {"Both",		PhoXiHardwareTriggerSignal::Both}},
	    "", "motioncam");
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
	    "camera_exposure",
	    _device->MotionCamCameraMode->Exposure,
	    boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode, double>,
			this,
			&PhoXi::MotionCamCameraMode,
			&PhoXiMotionCamCameraMode::Exposure, _1,
			false, "CameraExposure"),
	    "Exposure time in miliseconds",
	    enum_exposures, "", "motioncam_camera_mode");

      // 2,2 sampling topology
	if (_device->MotionCamCameraMode->SamplingTopology !=
	    PhoXiSamplingTopology::NoValue)
	    _ddr.registerEnumVariable<int>(
		"sampling_topology",
		_device->MotionCamCameraMode->SamplingTopology,
		boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					       PhoXiSamplingTopology>,
			    this,
			    &PhoXi::MotionCamCameraMode,
			    &PhoXiMotionCamCameraMode::SamplingTopology, _1,
			    false, "SamplingTopology"),
		"Sampling topology",
		{{"Standard", PhoXiSamplingTopology::Standard}},
		"", "motioncam_camera_mode");

      // 2.3 output topology
	if (_device->MotionCamCameraMode->OutputTopology !=
	    PhoXiOutputTopology::NoValue)
	    _ddr.registerEnumVariable<int>(
		"output_topology",
		_device->MotionCamCameraMode->OutputTopology,
		boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					       PhoXiOutputTopology>,
			    this,
			    &PhoXi::MotionCamCameraMode,
			    &PhoXiMotionCamCameraMode::OutputTopology, _1,
			    false, "OutputTopology"),
		"Output topology",
		{{"IrregularGrid",	PhoXiOutputTopology::IrregularGrid},
		 {"Raw",		PhoXiOutputTopology::Raw},
		 {"RegularGrid",	PhoXiOutputTopology::RegularGrid}},
		"", "motioncam_camera_mode");

      // 2.4 coding strategy
	if (_device->MotionCamCameraMode->CodingStrategy !=
	    PhoXiCodingStrategy::NoValue)
	    _ddr.registerEnumVariable<int>(
		"camera_coding_strategy",
		_device->MotionCamCameraMode->CodingStrategy,
		boost::bind(&Camera::set_field<PhoXiMotionCamCameraMode,
					       PhoXiCodingStrategy>,
			    this,
			    &PhoXi::MotionCamCameraMode,
			    &PhoXiMotionCamCameraMode::CodingStrategy, _1,
			    false, "CodingStrategy"),
		"Coding strategy",
		{{"Normal",	      PhoXiCodingStrategy::Normal},
		 {"Interreflections", PhoXiCodingStrategy::Interreflections}},
		"", "motioncam_camera_mode");

      // 2.5 TextureSource
	if (_device->MotionCamCameraMode->TextureSource !=
	    PhoXiTextureSource::NoValue)
	    _ddr.registerEnumVariable<int>(
		"camera_texture_source",
		_device->MotionCamCameraMode->TextureSource,
		boost::bind(&Camera::set_texture_source<PhoXiMotionCamCameraMode>,
			    this, &PhoXi::MotionCamCameraMode, _1),
		"Source used for texture image",
		{{"Computed",	PhoXiTextureSource::Computed},
		 {"LED",	PhoXiTextureSource::LED},
		 {"Laser",	PhoXiTextureSource::Laser},
		 {"Focus",	PhoXiTextureSource::Focus},
		 {"Color",	PhoXiTextureSource::Color}},
		"", "motioncam_camera_mode");
    }

  // 3. MotionCam scanner mode
    if (is_available(_device->MotionCamScannerMode))
    {
      // 3.1 shutter multiplier
	_ddr.registerVariable<int>(
	    "shutter_multiplier",
	    _device->MotionCamScannerMode->ShutterMultiplier,
	    boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode, int>,
			this,
			&PhoXi::MotionCamScannerMode,
			&PhoXiMotionCamScannerMode::ShutterMultiplier, _1,
			false, "ShutterMultiplier"),
	    "Shutter multiplier", 1, 20, "motioncam_scanner_mode");

      // 3.2 scan multiplier
	_ddr.registerVariable<int>(
	    "scan_multiplier",
	    _device->MotionCamScannerMode->ScanMultiplier,
	    boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode, int>,
			this,
			&PhoXi::MotionCamScannerMode,
			&PhoXiMotionCamScannerMode::ScanMultiplier, _1,
			false, "ScanMultiplier"),
	    "Scan multiplier", 1, 20, "motioncam_scanner_mode");

      // 3.3 coding strategy
	if (_device->MotionCamScannerMode->CodingStrategy !=
	    PhoXiCodingStrategy::NoValue)
	    _ddr.registerEnumVariable<int>(
		"scanner_coding_strategy",
		_device->MotionCamScannerMode->CodingStrategy,
		boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode,
					       PhoXiCodingStrategy>,
			    this,
			    &PhoXi::MotionCamScannerMode,
			    &PhoXiMotionCamScannerMode::CodingStrategy, _1,
			    false, "CodingStrategy"),
		"Coding  strategy",
		{{"Normal",	      PhoXiCodingStrategy::Normal},
		 {"Interreflections", PhoXiCodingStrategy::Interreflections}},
		"", "motioncam_scanner_mode");

      // 3.4 coding quality
	if (_device->MotionCamScannerMode->CodingQuality !=
	    PhoXiCodingQuality::NoValue)
	    _ddr.registerEnumVariable<int>(
		"coding_quality",
		_device->MotionCamScannerMode->CodingQuality,
		boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode,
					       PhoXiCodingQuality>,
			    this,
			    &PhoXi::MotionCamScannerMode,
			    &PhoXiMotionCamScannerMode::CodingQuality, _1,
			    false, "CodingQuality"),
		"Coding quality",
		{{"Fast",	PhoXiCodingQuality::Fast},
		 {"High",	PhoXiCodingQuality::High},
		 {"Ultra",	PhoXiCodingQuality::Ultra}},
		"",
		"motioncam_scanner_mode");

      // 3.5 texture source
	if (_device->MotionCamScannerMode->TextureSource !=
	    PhoXiTextureSource::NoValue)
	    _ddr.registerEnumVariable<int>(
		"texture_source",
		_device->MotionCamScannerMode->TextureSource,
		boost::bind(&Camera::set_texture_source<PhoXiMotionCamScannerMode>,
			    this, &PhoXi::MotionCamScannerMode, _1),
		"Texture source",
		{{"Computed",	PhoXiTextureSource::Computed},
		 {"LED",	PhoXiTextureSource::LED},
		 {"Laser",	PhoXiTextureSource::Laser},
		 {"Focus",	PhoXiTextureSource::Focus},
		 {"Color",	PhoXiTextureSource::Color}},
		"", "motioncam_scanner_mode");

      // 3.6 exposure
#  if defined(HAVE_MOTIONCAM_EXPOSURE)
	std::map<std::string, double>	enum_exposures;
	for (auto exposure :
		 _device->SupportedSinglePatternExposures.GetValue())
	    enum_exposures.emplace(std::to_string(exposure), exposure);
	_ddr.registerEnumVariable<double>(
	    "scanner_exposure",
	    _device->MotionCamScannerMode->Exposure,
	    boost::bind(&Camera::set_field<PhoXiMotionCamScannerMode, double>,
			this,
			&PhoXi::MotionCamScannerMode,
			&PhoXiMotionCamScannerMode::Exposure, _1, false,
			"Exposure"),
	    "Exposure", enum_exposures, "", "motioncam_scanner_mode");
#  endif
    }
}
#endif

void
Camera::setup_ddr_common()
{
    using namespace	pho::api;

  // 1. TriggerMode
    if (is_available(_device->TriggerMode))
    {
	if (_device->TriggerMode.GetValue() != PhoXiTriggerMode::NoValue)
	    _ddr.registerEnumVariable<int>(
		"trigger_mode", _device->TriggerMode.GetValue(),
		boost::bind(&Camera::set_feature<PhoXiTriggerMode, int>, this,
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
	    boost::bind(&Camera::set_feature<PhoXiTimeout, int>, this,
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
	    "confidence",
	    _device->ProcessingSettings->Confidence,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, double>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::Confidence, _1, false,
			"Confidence"),
	    "Confidence value", 0.0, 100.0, "processing_settings");

      // 3.2 SurfaceSmoothness
	if (_device->ProcessingSettings->SurfaceSmoothness !=
	    PhoXiSurfaceSmoothness::NoValue)
	    _ddr.registerEnumVariable<int>(
		"surface_smoothness",
		_device->ProcessingSettings->SurfaceSmoothness,
		boost::bind(&Camera::set_field<PhoXiProcessingSettings,
					       PhoXiSurfaceSmoothness>,
			    this,
			    &PhoXi::ProcessingSettings,
			    &PhoXiProcessingSettings::SurfaceSmoothness, _1,
			    false, "SurfaceSmoothness"),
		"Surface smoothness",
		{{"Sharp",	PhoXiSurfaceSmoothness::Sharp},
		 {"Normal",	PhoXiSurfaceSmoothness::Normal},
		 {"Smooth",	PhoXiSurfaceSmoothness::Smooth}},
		"", "processing_settings");

      // 3.3 CalibrationVolumeOnly
	_ddr.registerVariable<bool>(
	    "calibration_volume_only",
	    _device->ProcessingSettings->CalibrationVolumeOnly,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, bool>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::CalibrationVolumeOnly, _1,
			false, "CalibrationVolumeOnly"),
	    "Calibration volume only", false, true, "processing_settings");

      // 3.4 NormalsEstimationRadius
	_ddr.registerVariable<int>(
	    "normals_estimation_radius",
	    _device->ProcessingSettings->NormalsEstimationRadius,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, int>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::NormalsEstimationRadius, _1,
			false, "NormalsEstimationRadius"),
	    "Normals estimation radius", 0, 4, "processing_settings");

#if defined(HAVE_INTERREFLECTIONS_FILTERING)
      // 3.5 InterreflectionsFiltering
	_ddr.registerVariable<bool>(
	    "interreflections_filtering",
	    _device->ProcessingSettings->InterreflectionsFiltering,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, bool>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::InterreflectionsFiltering,
			_1, false, "InterreflectionsFiltering"),
	    "Interreflections filtering", false, true, "processing_settings");

#  if defined(HAVE_INTERREFLECTION_FILTER_STRENGTH)
      // 3.6 InterreflectionFilterStrength
	_ddr.registerVariable<double>(
	    "interreflection_filter_strength",
	    _device->ProcessingSettings->InterreflectionFilterStrength,
	    boost::bind(&Camera::set_field<PhoXiProcessingSettings, double>,
			this,
			&PhoXi::ProcessingSettings,
			&PhoXiProcessingSettings::InterreflectionFilterStrength,
			_1, false, "InterreflectionFilterStrength"),
	    "Interreflection filter strength", 0, 4, "processing_settings");
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
		"iso", iso[idx],
		boost::bind(&Camera::set_field<PhoXiColorSettings, int>, this,
			    &PhoXi::ColorSettings,
			    &PhoXiColorSettings::Iso, _1, false, "Iso"),
		"Color ISO", enum_iso, "", "color_settings");
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
		"exposure", exposure[idx],
		boost::bind(&Camera::set_field<PhoXiColorSettings, double>,
			    this,
			    &PhoXi::ColorSettings,
			    &PhoXiColorSettings::Exposure, _1, false,
			    "Exposure"),
		"Color exposure", enum_exposure, "", "color_settings");
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
	    _ddr.registerEnumVariable<int>("color_resolution", idx,
					   boost::bind(
					       &Camera::set_color_resolution,
					       this, _1),
					   "Color image resolution",
					   enum_resolution,
					   "", "color_settings");
	}

      // 4.4 Gamma
	_ddr.registerVariable<double>(
	    "gamma", color_settings.Gamma,
	    boost::bind(&Camera::set_field<PhoXiColorSettings, double>, this,
			&PhoXi::ColorSettings,
			&PhoXiColorSettings::Gamma, _1, false, "Gamma"),
	    "Color gamma correction", 0.0, 5.0, "color_settings");

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
		"white_balance", current_preset,
		boost::bind(&Camera::set_white_balance_preset, this, _1),
		"White balance", enum_presets, "", "color_settings");
	}
    }
#endif

  // 5. OutputSettings
    if (is_available(_device->OutputSettings))
    {
	_ddr.registerVariable<bool>(
	    "send_point_cloud",
	    _device->OutputSettings->SendPointCloud,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendPointCloud, _1, true,
			"SendPointCloud"),
	    "Publish point cloud if set", false, true, "output_settings");
	_ddr.registerVariable<bool>(
	    "send_normal_map",
	    _device->OutputSettings->SendNormalMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendNormalMap, _1, true,
			"SendNormalMap"),
	    "Publish normal map if set", false, true, "output_settings");
	_ddr.registerVariable<bool>(
	    "send_depth_map",
	    _device->OutputSettings->SendDepthMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendDepthMap, _1, true,
			"SendDepthMap"),
	    "Publish depth map if set", false, true, "output_settings");
	_ddr.registerVariable<bool>(
	    "send_confidence_map",
	    _device->OutputSettings->SendConfidenceMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendConfidenceMap, _1, true,
			"SendConfidenceMap"),
	    "Publish confidence map if set", false, true, "output_settings");
	_ddr.registerVariable<bool>(
	    "send_event_map",
	    _device->OutputSettings->SendEventMap,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendEventMap, _1, true,
			"SendEventMap"),
	    "Publish event map if set", false, true, "output_settings");
	_ddr.registerVariable<bool>(
	    "send_texture",
	    _device->OutputSettings->SendTexture,
	    boost::bind(&Camera::set_field<FrameOutputSettings, bool>, this,
			&PhoXi::OutputSettings,
			&FrameOutputSettings::SendTexture, _1, true,
			"SendTexture"),
	    "Publish texture if set", false, true, "output_settings");
#if defined(HAVE_COLOR_CAMERA)
	if (is_available(_device->ColorSettings))
	    _ddr.registerVariable<bool>(
		"send_color_camera_image",
		_device->OutputSettings->SendColorCameraImage,
		boost::bind(&Camera::set_field<FrameOutputSettings, bool>,
			    this,
			    &PhoXi::OutputSettings,
			    &FrameOutputSettings::SendColorCameraImage, _1,
			    true, "SendColorCameraImage"),
		"Publish color camera image if set", false, true,
		"output_settings");
#endif
    }

  // 6. Density of the cloud
    _ddr.registerVariable<bool>(
	    "dense_cloud", _dense_cloud,
	    boost::bind(&Camera::set_member<bool>, this,
			boost::ref(_dense_cloud), _1, "dense_cloud"),
	    "Dense cloud if set", false, true);

  // 7. Intensity scale
    _ddr.registerVariable<double>(
	    "intensity_scale", _intensity_scale,
	    boost::bind(&Camera::set_member<double>, this,
			boost::ref(_intensity_scale), _1, "intensity_scale"),
	    "Multiplier for intensity values of published texture",
	    0.05, 5.0);
}

template <class F> bool
Camera::is_available(const pho::api::PhoXiFeature<F>& feature) const
{
    if (feature.isEnabled() && feature.CanGet() && feature.CanSet())
	return true;
    NODELET_WARN_STREAM('('
			<< getBaseName()
			<< ") feature " << feature.GetName()
			<< " is not available");
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
	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") set " << f.GetName()
			    << " to " << f.GetValue());
    else
	NODELET_ERROR_STREAM('('
			     << getBaseName()
			     << ") failed to set " << f.GetName()
			     << " to " << value << ": "
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
	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") set " << f.GetName() << "::" << field_name
			    << " to "   << f.GetValue().*field);
    else
	NODELET_ERROR_STREAM('('
			     << getBaseName()
			     << ") failed to set "
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
    NODELET_INFO_STREAM('('
			<< getBaseName()
			<< ") set " << name << " to " << member);
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

    const auto modes = _device->SupportedCapturingModes.GetValue();
    if (idx < modes.size())
	_device->CapturingMode = modes[idx];
    NODELET_INFO_STREAM('('
			<< getBaseName()
			<< ") set resolution to "
			<< _device->CapturingMode.GetValue().Resolution.Width
			<< 'x'
			<< _device->CapturingMode.GetValue().Resolution.Height);

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
	_device->CapturingMode = modes[idx];
	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") set color resolution to "
			    << _device->ColorSettings->CapturingMode
				   .Resolution.Width
			    << 'x'
			    << _device->ColorSettings->CapturingMode
				   .Resolution.Height);
    }

    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();
}

void
Camera::set_white_balance_preset(const std::string& preset)
{
    _device->ColorSettings->WhiteBalance.Preset = preset;
    NODELET_INFO_STREAM('('
			<< getBaseName()
			<< ") set white balande to "
			<< _device->ColorSettings->WhiteBalance.Preset);
}
#endif

bool
Camera::trigger_frame(std_srvs::Trigger::Request&  req,
		      std_srvs::Trigger::Response& res)
{
    using namespace	pho::api;

    NODELET_INFO_STREAM('('
			<< getBaseName()
			<< ") trigger_frame: service requested");

    if (_device->TriggerMode.GetValue() != PhoXiTriggerMode::Software)
    {
	res.success = true;
	res.message = "succeded, but device is not in software trigger mode";

	NODELET_WARN_STREAM('('
			    << getBaseName()
			    << ") " << res.message);
	return true;
    }

    const auto	frameId = _device->TriggerFrame(true, true);

    NODELET_INFO_STREAM('('
			<< getBaseName()
			<< ") trigger_frame: triggered");

    switch (frameId)
    {
      case -1:
	res.success = false;
	res.message = "failed [TriggerFrame not accepted]";
	break;
      case -2:
	res.success = false;
	res.message = "failed [device is not running]";
	break;
      case -3:
	res.success = false;
	res.message = "failed [communication error]";
	break;
      case -4:
	res.success = false;
	res.message = "failed [WaitForGrabbingEnd is not supported]";
	break;
      default:
	if (!(_frame = _device->GetSpecificFrame(frameId,
						 PhoXiTimeout::Infinity)))
	{
	    res.success = false;
	    res.message = "failed [not found frame #"
			+ std::to_string(frameId) + ']';
	    break;
	}

	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") trigger_frame: frame got");

	if (_frame->Info.FrameIndex != uint64_t(frameId))
	{
	    res.success = false;
	    res.message = "failed [triggered frame(#"
			+ std::to_string(frameId)
			+ ") is not captured frame(#"
			+ std::to_string(_frame->Info.FrameIndex)
			+ ")]";
	    break;
	}

	publish_frame();	// publish
	res.success = true;
	res.message = "succeeded [frame #" + std::to_string(frameId) + ']';
	break;
    }

    if (res.success)
	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") trigger_frame: "
			    << res.message);
    else
	NODELET_ERROR_STREAM('('
			     << getBaseName()
			     << ") trigger_frame: "
			     << res.message);

    return true;
}

bool
Camera::save_settings(std_srvs::Trigger::Request&  req,
		      std_srvs::Trigger::Response& res)
{
    res.success = _device->SaveSettings();

    if (res.success)
    {
	res.message = "succesfully saved settings";
	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") save_settings: "
			    << res.message);
    }
    else
    {
	res.message = "failed to save settings";
	NODELET_ERROR_STREAM('('
			     << getBaseName()
			     << ") save_settings: "
			     << res.message);
    }

    return true;
}

bool
Camera::restore_settings(std_srvs::Trigger::Request&  req,
			 std_srvs::Trigger::Response& res)
{
    const auto acq = _device->isAcquiring();
    if (acq)
	_device->StopAcquisition();

    res.success = _device->ResetActivePreset();

    if (res.success)
    {
	res.message = "succesfully restored settings";
	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") restore_settings: "
			    << res.message);
    }
    else
    {
	res.message = "failed to restore settings";
	NODELET_ERROR_STREAM('('
			     << getBaseName()
			     << ") restore_settings: "
			     << res.message);
    }

    _device->ClearBuffer();
    if (acq)
	_device->StartAcquisition();

    return true;
}

void
Camera::cache_camera_matrix()
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

template <class T> Camera::image_p
Camera::create_image(const ros::Time& stamp, const std::string& frame_id,
		     const std::string& encoding, float scale,
		     const pho::api::Mat2D<T>& phoxi_image) const
{
    using namespace	sensor_msgs;
    using		element_ptr = const typename T::ElementChannelType*;

    image_p	image(new image_t);
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
	NODELET_ERROR_STREAM("Unsupported image type!");
	throw;
    }

    return image;
}

Camera::camera_info_p
Camera::create_camera_info(const ros::Time& stamp, const std::string& frame_id,
			   size_t width, size_t height,
			   const pho::api::CameraMatrix64f& K,
			   const std::vector<double>& D,
			   const pho::api::Point3_64f& t,
			   const pho::api::Point3_64f& rx,
			   const pho::api::Point3_64f& ry,
			   const pho::api::Point3_64f& rz) const
{
    camera_info_p	camera_info(new camera_info_t);

  // Set header.
    camera_info->header.stamp	   = stamp;
    camera_info->header.frame_id = frame_id;

  // Set size.
    camera_info->width  = width;
    camera_info->height = height;

  // Set distortion parameters.
    camera_info->distortion_model = "plumb_bob";
    camera_info->D.resize(5);
    std::fill(std::begin(camera_info->D), std::end(camera_info->D), 0.0);
    std::copy_n(std::begin(D),
		std::min(std::size(camera_info->D), std::size(D)),
    		std::begin(camera_info->D));

  // Set intrinsic parameters.
    camera_info->K[0] = K[0][0];
    camera_info->K[1] = K[0][1];
    camera_info->K[2] = K[0][2];
    camera_info->K[3] = K[1][0];
    camera_info->K[4] = K[1][1];
    camera_info->K[5] = K[1][2];
    camera_info->K[6] = K[2][0];
    camera_info->K[7] = K[2][1];
    camera_info->K[8] = K[2][2];

  // Set rotation matrix.
    camera_info->R[0] = rx.x;
    camera_info->R[1] = rx.y;
    camera_info->R[2] = rx.z;
    camera_info->R[3] = ry.x;
    camera_info->R[4] = ry.y;
    camera_info->R[5] = ry.z;
    camera_info->R[6] = rz.x;
    camera_info->R[7] = rz.y;
    camera_info->R[8] = rz.z;

  // Set 3x4 camera matrix.
    constexpr static double	scale = 0.001;	// milimeters ==> meters
    const double		T[] = {scale * t.x, scale * t.y, scale * t.z};
    for (int i = 0; i < 3; ++i)
    {
	camera_info->P[4*i + 3] = 0;

	for (int j = 0; j < 3; ++j)
	{
	    camera_info->P[4*i + j] = 0;

	    for (int k = 0; k < 3; ++k)
		camera_info->P[4*i + j] += camera_info->K[3*i + k]
					 * camera_info->R[3*k + j];

	    camera_info->P[4*i + 3] -= camera_info->P[4*i + j] * T[j];
	}
    }

  // No binning
    camera_info->binning_x = camera_info->binning_y = 0;

  // ROI is same as entire image.
    camera_info->roi.width = camera_info->roi.height = 0;

    return camera_info;
}

void
Camera::publish_frame()
{
    using	namespace sensor_msgs;

  // Common setting.
    const auto	stamp = ros::Time::now();
		      - ros::Duration((_frame->Info.FrameDuration +
				       _frame->Info.FrameComputationDuration +
				       _frame->Info.FrameTransferDuration)
				      * 0.001);

  // Publish point cloud.
    profiler_start(0);
    constexpr float	distanceScale = 0.001;	// milimeters -> meters
    publish_cloud(stamp, distanceScale);

  // Publish normal_map, depth_map, confidence_map, event_map and texture.
    profiler_start(1);
    publish_image(stamp, image_encodings::TYPE_32FC3,
		  1, _frame->NormalMap, _normal_map_pub);
    profiler_start(2);
    publish_image(stamp, image_encodings::TYPE_32FC1,
		  distanceScale, _frame->DepthMap, _depth_map_pub);
    profiler_start(3);
    publish_image(stamp, image_encodings::TYPE_32FC1,
		  1, _frame->ConfidenceMap, _confidence_map_pub);
    profiler_start(4);
    publish_image(stamp, image_encodings::TYPE_32FC1,
		  1, _frame->EventMap, _event_map_pub);
    profiler_start(5);
#if defined(HAVE_COLOR_CAMERA)
    if (_color_texture_source)
	publish_image(stamp, image_encodings::RGB8,
		      _intensity_scale, _frame->TextureRGB, _texture_pub);
    else
#endif
	publish_image(stamp, image_encodings::MONO8,
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
    NODELET_DEBUG_STREAM('('
			 << getBaseName()
			 << ") frame published [#"
			 << _frame->Info.FrameIndex << ']');
}

void
Camera::publish_cloud(const ros::Time& stamp, float distanceScale) const
{
    using namespace	sensor_msgs;

    const auto&	phoxi_cloud = _frame->PointCloud;
    if (_cloud_pub.getNumSubscribers() == 0 ||
    	!_device->OutputSettings->SendPointCloud || phoxi_cloud.Empty())
	return;

  // Convert pho::api::PointCloud32f to sensor_msgs::PointCloud2
    cloud_p	cloud(new cloud_t);
    cloud->header.stamp    = stamp;
    cloud->header.frame_id = _frame_id;
    cloud->is_bigendian    = false;
    cloud->is_dense	   = _dense_cloud;

    PointCloud2Modifier	modifier(*cloud);
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

    if (cloud->is_dense)
    {
	const auto	npoints = npoints_valid(phoxi_cloud);
	modifier.resize(npoints);
	cloud->height = 1;
	cloud->width  = npoints;
    }
    else
    {
	modifier.resize(phoxi_cloud.Size.Height * phoxi_cloud.Size.Width);
	cloud->height = phoxi_cloud.Size.Height;
	cloud->width  = phoxi_cloud.Size.Width;
    }
    cloud->row_step = cloud->width * cloud->point_step;

    PointCloud2Iterator<float>	xyz(*cloud, "x");

    for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
    {
	auto	p = phoxi_cloud[v];

	for (const auto q = p + phoxi_cloud.Size.Width; p != q; ++p)
	{
	    if (float(p->z) == 0.0f)
	    {
		if (!cloud->is_dense)
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
	PointCloud2Iterator<uint8_t> bgr(*cloud, "rgb");
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
		    if (!cloud->is_dense || float(p->z) != 0.0f)
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
		    if (!cloud->is_dense || float(p->z) != 0.0f)
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
	    NODELET_ERROR_STREAM('('
				 << getBaseName()
				 << ") normals_estimation_radius must be positive");
	    return;
	}

	PointCloud2Iterator<float>	normal(*cloud, "normal_x");

	for (int v = 0; v < phoxi_cloud.Size.Height; ++v)
	{
	    auto	p = phoxi_cloud[v];
	    auto	r = _frame->NormalMap[v];

	    for (const auto q = p + phoxi_cloud.Size.Width; p != q; ++p, ++r)
	    {
		if (!cloud->is_dense || float(p->z) != 0.0f)
		{
		    normal[0] = r->x;
		    normal[1] = r->y;
		    normal[2] = r->z;
		    ++normal;
		}
	    }
	}
    }

    _cloud_pub.publish(cloud);
}

template <class T> void
Camera::publish_image(const ros::Time& stamp,
		      const std::string& encoding, float scale,
		      const pho::api::Mat2D<T>& phoxi_image,
		      const image_transport::Publisher& publisher) const
{
    if (publisher.getNumSubscribers() == 0 || phoxi_image.Empty())
	return;

    publisher.publish(create_image(stamp, _frame_id,
				   encoding, scale, phoxi_image));
}

void
Camera::publish_camera_info(const ros::Time& stamp)
{
    if (_camera_info_pub.getNumSubscribers() == 0)
	return;

  // We have to extract the camera matrix from
  // _device->CalibrationSettings->CameraMatrix instead of
  // _frame->Info.CameraMatrix because the latter becomes empty
  // for non-reqular pointclooud topology. As accessing the former
  // is time-consuming, the extracted matrix is cached.
    if (_frame->DepthMap.Size != _depth_map_size)
    {
	_depth_map_size = _frame->DepthMap.Size;

	cache_camera_matrix();

	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") camera_matrix updated");
    }

    _camera_info_pub.publish(
	create_camera_info(stamp, _frame_id,
			   _frame->DepthMap.Size.Width,
			   _frame->DepthMap.Size.Height,
			   _camera_matrix,
			   _frame->Info.DistortionCoefficients,
			   _frame->Info.SensorPosition,
			   _frame->Info.SensorXAxis,
			   _frame->Info.SensorYAxis,
			   _frame->Info.SensorZAxis));
}

#if defined(HAVE_COLOR_CAMERA)
void
Camera::publish_color_camera(const ros::Time& stamp)
{
    if (_frame->ColorCameraImage.Size != _color_camera_image_size)
    {
	_color_camera_image_size = _frame->ColorCameraImage.Size;

	constexpr static double		s = 0.001;  // milimeters ==> meters
	geometry_msgs::TransformStamped	transform;
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

	NODELET_INFO_STREAM('('
			    << getBaseName()
			    << ") transform from color camera updated");
    }

    if (_color_camera_pub.getNumSubscribers() == 0 ||
	!_device->OutputSettings->SendColorCameraImage)
	return;

    _color_camera_pub.publish(
	create_image(stamp, _color_camera_frame_id,
		     sensor_msgs::image_encodings::RGB8, _intensity_scale,
		     _frame->ColorCameraImage),
	create_camera_info(stamp, _color_camera_frame_id,
			   _frame->ColorCameraImage.Size.Width,
			   _frame->ColorCameraImage.Size.Height,
			   _frame->Info.ColorCameraMatrix,
			   _frame->Info.ColorCameraDistortionCoefficients,
			   _frame->Info.ColorCameraPosition,
			   _frame->Info.ColorCameraXAxis,
			   _frame->Info.ColorCameraYAxis,
			   _frame->Info.ColorCameraZAxis));
}
#endif
}	// namespace aist_phoxi_camera
