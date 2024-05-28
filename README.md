aist_phoxi_camera: ROS driver for PhoXi 3D Scanner and MotionCam-3D
==================================================

This package provides a ROS driver for controlling [PhoXi 3D
scanners](http://www.photoneo.com/product-showcase/phoxi_3d_scanners). Although it is inspired by [ROS
drvier](https://github.com/photoneo/phoxi_camera) by the manufacturer,
i.e. [Photoneo co.](https://www.photoneo.com), the structure of the code
is completely reorganized.

`aist_phoxi_control` has the following features.
- The driver supports not only [PhoXi 3D Scanners](https://www.photoneo.com/phoxi-3d-scanner/) but also [MotionCam-3D and MotionCam-3D Color](https://www.photoneo.com/motioncam-3d/).
- Most of the scanning parameters configurable with [PhoXiControl](https://www.photoneo.com/downloads/phoxi-control) can be changed also with [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) at runtime.
- The driver is implemented as a [nodelet](http://wiki.ros.org/nodelet) which allows transferring point cloud and/or various 2D maps in a zero-copy manner to the subscribers also implemented using `nodelet`.

The driver is tested under the following conditions.
- ROS noetic on Ubuntu-20.04
- PhoXiControl-1.12.3
- PhoXi 3D Scanner Gen1 with firmware-1.2.38, MotionCam-3D and MotionCam-3D Color both with firmware-1.13.0

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

After the preparation above, download the [driver source code](https://github.com/Automation-Research-Team/aist_phoxi_camera) as well as the [ddynamic_reconfigure](https://github.com/Automation-Research-Team/ddynamic_reconfigure) package by typing
```
$ cd (your-catkin-workspace)/src
$ git clone https://github.com/Automation-Research-Team/aist_phoxi_camera
$ git clone https://github.com/Automation-Research-Team/ddynamic_reconfigure
```
`aist_phoxi_camera` requires the latter which is a modified version of the [original](https://github.com/pal-robotics/ddynamic_reconfigure) to fix some bugs and add new features.

Finally, you can compile the ROS driver by typing
```bash
$ source (your-catkin-workspace)/devel/setup.bash
$ catkin build aist_phoxi_camera
```

## Testing

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
"InstalledExamples-basic-example". Therefore, you can
test the driver even if no real scanners are
connected to your host. You can launch the driver and establish
a connection to the virtual scanner by typing
```bash
$ roslaunch aist_phoxi_camera run.launch vis:=true
```
where `vis:=true` means that the ROS visualizer, `rviz`, and the parameter setting GUI, `rqt_reconfigure`, are invoked as well. Here, `rviz` is configured to subscribe two topics, `pointcloud`
and `texture`, published by the driver. As the driver is
started with `trigger_mode:=0` i.e. `Free Run` mode, you will see
continuously updated point cloud and image streams. You can interactively
change various capturing parameters through `rqt_reconfigure`.

When the driver is started with a virtual scanner, there will be many error messages complaining that some parameters cannot be set. This might be because not all the features of real scanners are emulated by the virtual scanner. You can safely ignore them.

If you wish to connect a real device, specify its unique ID;
```bash
$ roslaunch aist_phoxi_camera run.launch vis:=true id:="2018-09-016-LC3"
```
Here, the ID, "2018-09-016-LC3", varies for each device which can be
known from `Network Discovery` window of `PhoXiControl`.

## Starting the driver as a nodelet

You can launch the driver as a `nodelet` by
```
$ roslaunch aist_phoxi_camera run.launch manager:=<manager_name> [external_manager:=[true|false]]
```
where
- **manager** (type: str) -- Node name of the nodelet manager. If not specified, the driver is started as a separate node.
- **external_manager** (type: bool) If true, the driver will be loaded into the existing nodelet manager with a name specified by `manager` which has been started in advance. If false, a manager with the name specified by `manager` will be newly created.

Then zero-copy transfer will be realized if you load subscriber nodelets into the same manager.

## ROS services

The following services are available.

- **~/trigger_frame** (type: [std_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)) -- Capture an image frame and publish point cloud and/or images in it.
- **~/save_settings** (type: [std_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)) -- Save camera settings to the internal ROM of the device.
- **~/restore_settings** (type: [std_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html)) -- Restore camera settings from the internal ROM of the device.

## ROS topics

The following topics are published by the driver.

- **~/confidence_map** (type: [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) -- A 2D map of values indicating reliability of 3D measurements at each pixel.
- **~/depth_map**  (type: [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) -- A 2D map of depth values, i.e. z-coordinate values of point cloud, in meters.
- **~/event_map** (type: [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) -- A 2D map
- **~/normal_map** (type: [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) -- A 2D map of surface normals.
- **~/texture** (type: [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) -- A 2D map of intensity/color values. The values are in 8/24bit unsigned integer format.
- **~/pointcloud** (type: [sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)) -- A 2D map of 3D point coordinates in meters. Each 2D pixel has an associated intensity/color value in RGBA format if the parameter `send_texture` is true. In addition, each pixel will be associated with a 3D normal vector if the parameter `send_normal_map` is true.
- **~/camera_info** (type: [sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)) -- Intrinsic parameters of the depth sensor including a 3x3 calibration matrix and lens distortion coefficients.
- **~/color/image** (type: [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) -- A 2D map of color values captured by the color sensor of the device. Available only for `MotionCam-3D Color`.
- **~/color/camera_info** (type: [sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)) -- Intrinsic parameters of the color sensor including a 3x3 calibration matrix and lens distortion coefficients. Available only for `MotionCam-3D Color`.

For `MotionCam-3D Color`, a transform from the frame at the color sensor to that at the depth sensor is broadcasted as a static `tf2` message as well. Thus you will have the pose of color sensor relative to the depth sensor by looking up a transform between them.

## ROS parameters

The following parameters can be optionally specified 
- **~/id** (type: str) -- Unique ID of the scanner (default: "InstalledExamples-basic-example")
- **~/rate** (type: double) -- Rate of getting frames when the scanner operates in `free run` mode (default: 20.0 (Hz))
- **~/frame** (type: str) -- Frame ID of the depth sensor used for published images and pointcloud (default: "sensor")
- **~/color_camera_frame** (type: str) -- Frame ID of the color sensor used for published color images (default: "color_sensor")
- **~/intensity_scale** (type: double) -- Scale factor of intensity/color values published in the `texture` topic (default: 0.5)
- **~/trigger_mode** (type: int) -- Switch between `free run`(0), `software trigger`(1), and `hardware trigger`(2) modes
- **~/send_normal_map** (type: bool) -- Publish `normal_map` if true.
- **~/send_depth_map** (type: bool) -- Publish `depth_map` if true.
- **~/send_confidence_map** (type: bool) -- Publish `confidence_map` if true.
- **~/send_event_map** (type: bool) -- Publish `event_map` if true.
- **~/send_texture** (type: bool) -- Publish `texture` if true.
- **~/send_color_image** (type: bool) -- Publish `color/image` and `color/camera_info` if true. Available only for `MotionCam-3D Color`.
- **~/send_point_cloud** (type: bool) -- Publish `pointcloud` if true. Each pixel has its 3D coordinates with respect to the depth sensor. It will also be associated with an intensity/color value if `send_texture` is true, and with a 3D normal vector if `send_normal_map` is true.

when starting the driver by
```
$ roslaunch aist_phoxi_camera run.launch <parameter_name>:=<parameter_value>...
```
Many other scanning parameters configurable with `PhoXiControl` can also be interactively changed with [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) at runtime. If not specified above, the default values of all the parameters conform to the device state when the driver is started.