aist_robotiq: ROS driver for Robotiq two-finger grippers
==================================================

This package provides ROS drivers and an action controller for [Robotiq](https://robotiq.com) two finger grippers, namely [2F-85, 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) and [Hand-E](https://robotiq.com/products/hand-e-adaptive-robot-gripper). 

## Driver
The following three drivers are available.

- **TCP driver** -- Save the original depth image with no filteres applied to `~/.ros/.tif`. This file is used as the backgroud in processing the subsequent input images.
- **RTU driver** -- First, capture the filtered intensity/color, depth and normal images. Then, create a PLY file from them and save it to `~/.ros/scene.ply`. Finally, its file path as well as the frame id of the camera is published to the topic `~/file_info`.
- **URCap driver** -- First, capture the filtered intensity/color, depth and normal images. Then, create a PLY file from them and save it to `~/.ros/scene.ply`. Finally, its file path as well as the frame id of the camera is published to the topic `~/file_info`.

## Controller

The node applies the filtering process to the input depth images in the following order;

## Gazebo plugin

## Usage
You can start both driver and controller by the following command;
```shell
$ roslaunch aist_robotiq run.launch ip:=<ip> [type:=<type>] [device:=<device>] [prefix:=<prefix>] 
```
where
- **ip** -- If `type = urcap`, specify IP address of the controller box of [Universal Robot](https://www.universal-robots.com) CB-series or e-Series with [URCap software](https://robotiq.com/support) installed. Otherwise, specify IP address of [Robotiq Universal Controller](https://assets.robotiq.com/website-assets/support_documents/document/online/Controller_UserManual_HTML5_20181120.zip/Controller_UserManual_HTML5/Default.htm).
- **type** -- Specify driver type. Currently `tcp`, `rtu` and `urcap` are supported. (default: `urcap`)
- **device** -- Specify gripper device. Currently `robotiq_85`, `robotiq_140` and `robotiq_hande` are supported. (default: `robotiq_85`)
- **prefix** -- Specify prefix string for identifying a specific device from multiple grippers. (default: `a_bot_gripper_`)

Then the gripper will be automatically calibrated by fully opening and then fully closing its fingers. Encoder readings at these two position are redorded by the controller and will be used in the subsequent grasping tasks.

Now, you can make a connection to the action server of the controller from any action clients of [control_msgs](http://wiki.ros.org/control_msgs).[GripperCommand](http://docs.ros.org/en/api/control_msgs/html/action/GripperCommand.html) type. The simplest way for testing is invoking [actionlib](http://wiki.ros.org/actionlib)'s `axclient.py` and connect it to the server;
```
$ roslaunch aist_robotiq test.launch [prefix:=<prefix>]
```
where
- **prefix** -- Specify gripper prefix which must be same as the one given to `run.launch`. (default: `a_bot_gripper_`)