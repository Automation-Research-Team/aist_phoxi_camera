aist_phoxi_camera: ROS2 driver for PhoXi 3D Scanner and MotionCam-3D
==================================================

This package provides a ROS2 driver for controlling [PhoXi 3D
scanners](http://www.photoneo.com/product-showcase/phoxi_3d_scanners). Although it is inspired by [ROS1
drvier](https://github.com/photoneo/phoxi_camera) by the manufacturer,
i.e. [Photoneo co.](https://www.photoneo.com), the structure of the code
is completely reorganized.

`aist_phoxi_camera` has the following features.
- The driver supports not only [PhoXi 3D Scanners](https://www.photoneo.com/phoxi-3d-scanner/) but also [MotionCam-3D and MotionCam-3D Color](https://www.photoneo.com/motioncam-3d/).
- Most of the scanning parameters configurable with [PhoXiControl](https://www.photoneo.com/downloads/phoxi-control) can be changed also with [rqt_reconfigure](https://index.ros.org/p/rqt_reconfigure/) at runtime.
- The driver is implemented as a [component](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-a-Composable-Node.html) which allows transferring point cloud and/or various 2D maps in a zero-copy manner to other components loaded to the same [component container](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html).

The driver is tested under the following conditions.
- ROS `jazzy` on `Ubuntu-24.04`
- `PhoXiControl-1.16.1`
- PhoXi 3D Scanner Gen1 with `firmware-1.2.38`, MotionCam-3D and MotionCam-3D Color both with `firmware-1.16.0`

If you are using ROS1, please checkout `ros1-devel` branch of this package and follow the instructions there.

## Installation

Before compiling the ROS driver, you have to install the latest version of
[PhoXiControl](https://www.photoneo.com/downloads/phoxi-control)
which is a GUI-based controller for PhoXi 3D scanners.


Then append the following lines to your `~/.bashrc`
```bash
if [ -d /opt/Photoneo/PhoXiControl-x.y.z ]; then
  export PHOXI_CONTROL_PATH=/opt/Photoneo/PhoXiControl-x.y.z
  export PATH=${PATH}:${PHOXI_CONTROL_PATH}/bin
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHOXI_CONTROL_PATH}/API/lib
  export CPATH=${CPATH}:${PHOXI_CONTROL_PATH}/API/include
fi
```
where `x.y.z` should be replaced with the correct version number of `PhoXiControl` you have downloaded. Update the environment variables by typing
```bash
$ source ~/.bashrc
```

After the preparation above, bdownload the [driver source code](https://github.com/Automation-Research-Team/aist_phoxi_camera) as well as the [ddynamic_reconfigure2](https://github.com/Automation-Research-Team/ddynamic_reconfigure2) package by typing
```bash
$ cd (your-ros2-workspace)/src
$ git clone https://github.com/Automation-Research-Team/aist_phoxi_camera
$ git clone https://github.com/Automation-Research-Team/ddynamic_reconfigure2
```
`aist_phoxi_camera` requires the latter which is a ROS2 replacement of the [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure) package in ROS1.

Then checkout appropriate branches.
```bash
$ cd aist_phoxi_camera
$ git checkout ros2-devel
$ cd ../ddynamic_reconfigure2
$ git checkout develop
```
Finally, you can compile the ROS driver by typing
```bash
$ source (your-ros2-workspace)/install/setup.bash
$ colcon build
```

### Installing a modified version of rqt_reconfigure (optional)
The [rqt_reconfigure](https://github.com/ros-visualization/rqt_reconfigure), a GUI-based node/plugin for interactively setting ROS parameter values, has been widely used since ROS1 and continuously supported in ROS2 as well. However, the current ROS2 version lacks some features supported in ROS1 version such as inputting numeric values through sliders, grouping parameters into tabs and selecting parameter values from candidates shown in pull-down menu. You will find these lacking features in the modified version [here](https://github.com/Automation-Research-Team/rqt_reconfigure), which is forked from the [rolling branch](https://github.com/ros-visualization/rqt_reconfigure/tree/rolling) of the original and will improve usability of `aist_phoxi_camera`.

## Testing the driver

You have to invoke `PhoXiControl` in advance of running
`aist_phoxi_camera`. It directly communicates
with one or more scanners on the network. Our ROS driver, `aist_phoxi_camera`,
establishes a connection to one of the available scanners, sends
control commands to it and receives various data streams,
ex. point cloud, texture map, depth map, confidence map, etc., via
`PhoXiControl`. You can start `PhoXiControl` by typing
```bash
$ PhoXiControl
```

`PhoXiControl` provides a virtual scanner device named
`"InstalledExamples-basic-example"`. Therefore, you can
test the driver even if no real scanners are
connected to your host. You can launch the driver and establish
a connection to the virtual scanner by typing
```bash
$ ros2 launch aist_phoxi_camera launch.py vis:=true
```
where `vis:=true` means that the ROS visualizer, `rviz2`, and the parameter setting GUI, `rqt_reconfigure`, are invoked as well. Here, `rviz2` is configured to subscribe two topics, `pointcloud`
and `texture`, published by the driver. As the driver is
started with `Free Run` mode, you will see
continuously updated point cloud and image streams. You can interactively
change various capturing parameters through `rqt_reconfigure`.

When the driver is started with a virtual scanner, there will be many error messages complaining that some parameters cannot be set. This might be because not all the features of real scanners are emulated by the virtual scanner. You can safely ignore them.

If you wish to connect a real device, specify its unique ID;
```bash
$ ros2 launch aist_phoxi_camera launch.py vis:=true id:="2018-09-016-LC3"
```
Here, the ID, "2018-09-016-LC3", varies for each device which can be
known from `Network Discovery` window of `PhoXiControl`.

## Trouble shooting

In ROS1 version of `aist_phoxi_camera`, some people reported that the driver fails to start with the following error message;
```bash
type is aist_phoxi_camera/aist_phoxi_camera
[ INFO] [1715922872.621560597]: Initializing nodelet with 32 worker threads.
[ INFO] [1715922872.622773835]: aist_phoxi_camera::CameraNodelet::onInit()
free(): invalid pointer
```
I also noticed that this happened when the driver was started in a docker container with `PhoXiControl-1.13.4` installed.

It seems that the problem is due to some incompatibility between PhoXi API library and ROS header files.  However, the issue was resolved by updating `PhoXiControl` to `1.14.0`. So, please make sure that you are using the latest version if you encountered the similar problem.

## Using the driver

You can launch the driver by typing
```
$ ros2 launch aist_phoxi_camera launch.py [camera_name:=<camera_name>...]
```
with options
- **camera_name** -- Node name of the camera (default: `phoxi`)
- **id** -- Unique ID of the camera (default: `InstalledExamples-basic-example`)
- **config_file** -- Absolute path to the configuration file for setting parameters listed below (default: package://aist_phoxi_camera/config/[default.yaml](./config/default.yaml))
- **container** -- Node name of the component container (default: `camera_container`)
- **external_container** -- If `true`, the driver will be loaded into the existing container with a name specified by `container` which has been started in advance. If `false`, a container with the name specified by `container` will be newly created and the driver will be loaded into it. (default: `false`)
- **vis** -- If `true`, launch the ROS visualizer, `rviz2`, and the parameter setting GUI, `rqt_reconfigure`. (default: `false`)

Then zero-copy transfer will be realized if you load subscriber components into the same container specified above.

## ROS2 services

The following services are available.

- **~/trigger_frame** (type: [std_srvs/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html)) -- Capture an image frame and publish point cloud and/or images in it.
- **~/save_settings** (type: [std_srvs/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html)) -- Save camera settings to the internal ROM of the device.
- **~/restore_settings** (type: [std_srvs/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html)) -- Restore camera settings from the internal ROM of the device.

## ROS2 topics

The following topics are published by the driver.

- **~/confidence_map** (type: [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)) -- A 2D map of values indicating reliability of 3D measurements at each pixel.
- **~/depth_map**  (type: [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)) -- A 2D map of depth values, i.e. z-coordinate values of point cloud, in meters.
- **~/event_map** (type: [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)) -- A 2D map
- **~/normal_map** (type: [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)) -- A 2D map of surface normals.
- **~/texture** (type: [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)) -- A 2D map of intensity/color values. The values are in 8/24bit unsigned integer format.
- **~/pointcloud** (type: [sensor_msgs/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)) -- A 2D map of 3D point coordinates in meters. Each 2D pixel has an associated intensity/color value in RGBA format if the parameter `send_texture` is true. In addition, each pixel will be associated with a 3D normal vector if the parameter `send_normal_map` is true.
- **~/camera_info** (type: [sensor_msgs/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html)) -- Intrinsic parameters of the depth sensor including a 3x3 calibration matrix and lens distortion coefficients.
- **~/color/image** (type: [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)) -- A 2D map of color values captured by the color sensor of the device. Available only for `MotionCam-3D Color`.
- **~/color/camera_info** (type: [sensor_msgs/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html)) -- Intrinsic parameters of the color sensor including a 3x3 calibration matrix and lens distortion coefficients. Available only for `MotionCam-3D Color`.

For `MotionCam-3D Color`, a transform from the frame at the color sensor to that at the depth sensor is broadcasted as a static `tf2` message as well. Thus you will have the pose of color sensor relative to the depth sensor by looking up a transform between them.

## ROS2 parameters

The driver maintains  node parameters including
- **id** (type: str) -- Unique ID of the camera (default: "`InstalledExamples-basic-example`")
- **rate** (type: double) -- Rate of getting frames when the scanner operates in `free run` mode (default: `20.0` Hz)
- **frame** (type: str) -- Frame ID of the depth sensor used for published images and pointcloud (default: "`sensor`")
- **color_camera_frame** (type: str) -- Frame ID of the color sensor used for published color images (default: "`color_sensor`")
- **intensity_scale** (type: double) -- Scale factor of intensity/color values published in the `texture` topic (default: `0.5`)
- **trigger_mode** (type: int) -- Switch between `free run`(0), `software trigger`(1), and `hardware trigger`(2) modes
- **output_settings.send_normal_map** (type: bool) -- Publish `normal_map` if true.
- **output_settings.send_depth_map** (type: bool) -- Publish `depth_map` if true.
- **output_settings.send_confidence_map** (type: bool) -- Publish `confidence_map` if true.
- **output_settings.send_event_map** (type: bool) -- Publish `event_map` if true.
- **output_settings.send_texture** (type: bool) -- Publish `texture` if true.
- **output_settings.send_color_image** (type: bool) -- Publish `color/image` and `color/camera_info` if true. Available only for `MotionCam-3D Color`.
- **output_settings.send_point_cloud** (type: bool) -- Publish `pointcloud` if true. Each pixel has its 3D coordinates with respect to the depth sensor. It will also be associated with an intensity/color value if `send_texture` is true, and with a 3D normal vector if `send_normal_map` is true.

and many others concerning with capturing, point cloud processing and color processing. You can view and interactively change these values with `rqt_reconfigure`.
