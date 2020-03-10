aist_phoxi_camera: ROS driver for PhoXi 3D Scanner
==================================================

This package provides a ROS driver for controlling [PhoXi 3D
scanners](http://www.photoneo.com/product-showcase/phoxi_3d_scanners). The
software is inspired by [ROS
drvier](https://github.com/photoneo/phoxi_camera) by the manufacturer,
i.e. [Photoneo co.](https://www.photoneo.com), however, its structure
is completely reorganized.

## Installation

Before compiling the ROS driver, you have to install
[PhoXiControl](http://www.photoneo.com/3d-scanning-software)
which is a GUI-based controller for PhoXi 3D scanners.

Currently `PhoXiControl` is formally supported on Ubuntu 14 and Ubuntu 16.
In order to be executed on Ubuntu 18, some additional packages are required.
You can install `PhoXiControl` as well as these extra packages by simply
typing;
```bash
$ cd (your-catkin-workspace)/src/aist_phoxi_sensor/install-scripts
$ sudo ./install-photoneo.sh
```
Then append the following lines to your `~/.bashrc`
```bash
if [ -d /opt/PhotoneoPhoXiControl ]; then
  export PHOXI_CONTROL_PATH=/opt/PhotoneoPhoXiControl
  export PATH=${PATH}:${PHOXI_CONTROL_PATH}/bin
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHOXI_CONTROL_PATH}/API/lib
  export CPATH=${CPATH}:${PHOXI_CONTROL_PATH}/API/include
fi
```
and update the environment variables by typing;
```bash
$ source ~/.bashrc
```
Finally, you can compile the ROS driver by typing;
```bash
$ catkin build aist_phoxi_camera
```

## Testing

You have to invoke `PhoXiControl` in advance of running
`aist_phoxi_camera`. It directly communicates
with one or more scanners. Our ROS driver, `aist_phoxi_camera`,
establishes a connection to one of the available scanners, sends
control commands to it and receives various kinds of 3D data,
ex. point cloud, texture map, depth map, confidence map, etc., via
`PhoXiControl`. You can start `PhoXiControl` by typing;
```bash
$ PhoXiControl
```

`PhoXiControl` provides a virtual scanner device named
"InstalledExamples-basic-example". Therefore, you can
test our ROS driver `aist_phoxi_camera` even if no real scanners are
connected to your host. You can launch the driver and establish
a connection to the virtual scanner by typing as follows;

```bash
$ roslaunch aist_phoxi_camera test.launch
```
If you wish to connect a real device, specify its unique ID;
```bash
$ roslaunch aist_phoxi_camera test.launch id:="2018-09-016-LC3"
```
The ID, "2018-09-016-LC3" here, varies for each device which can be
known from `Network Discovery` window of `PhoXiControl`.

By issueing the command above, the ROS visualizer, `rviz`, will start as
well. Here, `rviz` is configured to subscribe two topics, `pointcloud`
and `texture`, published by `aist_phoxi_camera`. Since the driver is
started with `trigger_mode:=0`, i.e. `Free Run` mode, you will see
continuously updated pointcloud and image streams.

Another GUI, `rqt_reconfigure`, will be also started. You can dynamically
change various capturing parameters through it.

## Available ROS services

- **~/start_acquisition** -- Make the scanner ready for image acquisition.
- **~/stop_acquisition** -- Make the scanner stop image acquisition.
- **~/trigger_frame** -- Capture an image frame and publish it as specified topics.
- **~/get_device_list** -- Enumerate all the PhoXi 3D scanner devices available including the virtual scanner.
- **~/get_hardware_identification** -- Show unique ID of the scanner currently connected to.
- **~/get_hardware_supported_capturing_modes** -- Enumerate all capturing modes supported by the scanner currently connected to.

## Published ROS topics

- **~/confidence_map** -- A 2D map of values indicating reliability of 3D measurements at each pixel.
- **~/depth_map** -- A 2D map of depth values, i.e. z-coordinate values of point cloud, in meters.
- **~/normal_map** -- A 2D map of surface normals.
- **~/texture** -- A 2D map of intensity values. The values are in 8bit unsigned integer format.
- **~/pointcloud** -- A 2D map of 3D points. Each 2D pixel has an associated intensity value in RGBA format as well as the corresponding 3D coordinates in meters.
- **~/camera_info** -- Camera parameters including a 3x3 calibration matrix and lens distortions.

## ROS parameters

- **~id** -- Unique ID of the scanner
- **~frame** -- Frame ID of the optical sensor used for published images and pointcloud
- **~trigger_mode** -- Switch between software trigger(1) and free run(0) modes
- **~intensity_scale** -- Change scale factor of intensity published in texture topic.
