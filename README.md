aist_robotiq: ROS action controller and driver for Robotiq two-finger grippers
==================================================

This package provides a ROS action controller and drivers for [Robotiq](https://robotiq.com) two finger grippers, namely [2F-85, 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) and [Hand-E](https://robotiq.com/products/hand-e-adaptive-robot-gripper). The package is forked from the [robotiq package developed by CRI group](https://github.com/crigroup/robotiq). And the URCap driver is borrowed from [the code by Felix von Drigalski](https://gist.github.com/felixvd/d538cad3150e9cac28dae0a3132701cf).

## Controller

The controller establishes an [ROS action](http://wiki.ros.org/actionlib) server of [control_msgs](http://wiki.ros.org/control_msgs).[GripperCommand](http://docs.ros.org/en/api/control_msgs/html/action/GripperCommand.html) type.

## Driver
The driver subscribes a command topic published by the controller and send them to the gripper. It also receives status from the gripper and publish them as a topic toward the controller. The following three drivers are available.

- **TCP driver** -- Will be used when the gripper is connected to the [Robotiq Universal Controller](https://assets.robotiq.com/website-assets/support_documents/document/online/Controller_UserManual_HTML5_20181120.zip/Controller_UserManual_HTML5/Default.htm) which acts as a converter between TCP/IP and Modbus. Not tested.
- **RTU driver** -- Not tested.
- **URCap driver** -- Will be used when the gripper is connected to the control box of [Universal Robot](https://www.universal-robots.com) CB-series or e-Series with [URCap software](https://robotiq.com/support) installed. The driver sends commands and receives status to/from the gripper via unix socket connected to the URCap server which is exposed to the port `63352` of the box.

## Gazebo plugin

## Usage
You can start both driver and controller by the following command;
```shell
$ roslaunch aist_robotiq run.launch ip:=<ip> [driver:=<driver>] [device:=<device>] [prefix:=<prefix>] 
```
where
- **ip** -- If `type = urcap`, specify IP address of the controller box of [Universal Robot](https://www.universal-robots.com) CB-series or e-Series with [URCap software](https://robotiq.com/support) installed. Otherwise, specify IP address of [Robotiq Universal Controller](https://assets.robotiq.com/website-assets/support_documents/document/online/Controller_UserManual_HTML5_20181120.zip/Controller_UserManual_HTML5/Default.htm).
- **driver** -- Specify driver type. Currently `tcp`, `rtu` and `urcap` are supported. (default: `urcap`)
- **device** -- Specify gripper device. Currently `robotiq_85`, `robotiq_140` and `robotiq_hande` are supported. (default: `robotiq_85`)
- **prefix** -- Specify a prefix string for identifying a specific device from multiple grippers. (default: `a_bot_gripper_`)

Then the gripper will be automatically calibrated by fully opening and then fully closing its fingers. Encoder readings at these two position are redorded by the controller and will be used in the subsequent grasping tasks.

Now, you can make a connection to the action server of the controller from any action clients of [control_msgs](http://wiki.ros.org/control_msgs).[GripperCommand](http://docs.ros.org/en/api/control_msgs/html/action/GripperCommand.html) type. The simplest way for testing is invoking [actionlib](http://wiki.ros.org/actionlib)'s `axclient.py` and connect it to the server;
```
$ roslaunch aist_robotiq test.launch [prefix:=<prefix>]
```
where
- **prefix** -- Specify gripper prefix which must be same as the one given to `run.launch`. (default: `a_bot_gripper_`)