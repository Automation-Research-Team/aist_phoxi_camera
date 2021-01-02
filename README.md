# ART ROS packages
ROS wrappers for various projects within ART lab

## What's so special about this branch?
* nothing, just little bit of instructions and a guide for others<br />

* added support for handeye calibaration using realsenseD435 rgbd camera with aruco marker ID=32, size=0.16, margin=0.01

* calibration accuracy achieved within `0.007m` in position and `0.69` in degrees

* main usage of this branch was to utilize realsenseD435 for pose detection and Pick-n-Place experiment with `UR5` using `MoveIt`

* tested on Ubuntu 18.04, ROS Melodic, RTX 2080-Ti, CUDA 10.1, Python2.7/3.7


# Usage
## 1. Installation, (I know its messy!)
* this will install everything, `UR5`, `MoveIt`, `realsense`, `aruco` packages
* cd ~/catkin_ws/src
    * git clone https://gitlab.com/art-aist-private/aist_aruco_ros.git
    * git clone https://gitlab.com/art-aist-private/artros.git
        * cd artros/
        * git checkout realsenseD435
        * git submodule update --init
        * rosdep install -i --from-paths .
        * cd ~/catkin_ws/src/artros/aist_phoxi_camera/install-scripts/
            * sudo ./install-phoxi-control.sh
        * cd ~/catkin_ws/src/artros/aist_localization/install-scripts/
            * sudo ./install-photoneo-localization.sh
        * sudo reboot (*restart your pc*)

        * copy these lines to your ~/.bashrc
            * *you don't need them but they are part of other pkgs*
            ```
            ###
            ### PhoXi settings
            ###
            if [ -d /opt/PhotoneoPhoXiControl-1.2.14 ]; then
            export PHOXI_CONTROL_PATH=/opt/PhotoneoPhoXiControl-1.2.14
            export PATH=${PATH}:${PHOXI_CONTROL_PATH}/bin
            export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHOXI_CONTROL_PATH}/API/lib
            export CPATH=${CPATH}:${PHOXI_CONTROL_PATH}/API/include
            fi

            ###
            ### PhoLocalization settings
            ###
            if [ -d /opt/PhotoneoSDK/Localization ]; then
            export PHO_LOCALIZATION_PATH=/opt/PhotoneoSDK/Localization
            export PATH=${PATH}:${PHO_LOCALIZATION_PATH}/bin
            export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHO_LOCALIZATION_PATH}/lib
            export CPATH=${CPATH}:${PHO_LOCALIZATION_PATH}/include
            fi
            ```
        * ***finally***, you can complie
        * cd ~/catkin_ws/
            * catkin_make
            * catkin_make install
            * cd devel
            * source ./setup.bash

        ** `Note:` if you get catkin_make `error` in *robotiq-cri* due to *`byte to str conversion`* do the following
        * go to line https://gitlab.com/art-aist/robotiq-cri/-/blob/devel-aist/robotiq_control/src/robotiq_control/cmodel_urscript.py#L125
        * in method ``buildCommandProgram`` replace line
        ```
        complete_program += program_line
        ```
        with
        ```
        encoding = 'utf-8'
        complete_program += program_line.decode(encoding)
        ```

## 2. if installation is successful, test it!
 * In simulation environment
    * roslaunch aist_bringup mocap_bringup.launch sim:=true
    * ![Alt text](images/scsim.png?raw=true "sim environment")
 * In real environment with UR5
    * roslaunch aist_bringup mocap_bringup.launch
    * ![Alt text](images/real.png?raw=true "real robot environment")


## 3. To perform handeye calibration using realsenseD435
* go to https://gitlab.com/art-aist-private/artros/-/tree/realsenseD435/aist_handeye_calibration and follow the instructions. or....(*follow below instructions*)
    * use config:=`mocap`
    * use camera_name:=`realsenseD435`

    ** **use this when calibrating realsenseD435**
    * run handeye calibration
    * roslaunch aist_handeye_calibration mocap_handeye_calibration.launch camera_name:=realsenseD435 scene:=mocap_calibration

    * start robot-control
        * roslaunch aist_handeye_calibration run_calibration.launch config:=mocap camera_name:=realsenseD435

    ** **use this when verifying calibration of realsenseD435**
    * to verify handeye calibration
        * roslaunch aist_handeye_calibration mocap_handeye_calibration.launch camera_name:=realsenseD435 check:=true

    * move UR5 ef to calibrated pose of aruco marker
        * roslaunch aist_handeye_calibration check_calibration.launch config:=mocap camera_name:=realsenseD435

## 4. If you wish to test further and perform object picking
* clone `yolo6d_ros` pkg https://github.com/avasalya/Yolo6D_ROS.git (***wt missing***? try password: `telexistence`)
* follow instructions and setup as per requirement
* by default, aruco marker pose is published as geometry_msgs/`PoseStamped`
* but Yolo6D ros-wrapper publishes pose as geometry_msgs/`PoseArray`
* so to adapt, change line https://gitlab.com/art-aist-private/artros/-/blob/realsenseD435/aist_routines/src/aist_routines/base.py#L379
```
    est_pose = target_pose.poses[n] #poseArray
    # est_pose = target_pose.pose #poseStamped
```
* `n` is the count of detected object, in my case I detect only 1 onigiri so, n is 0


## 5. Try out pick-n-place experiment
* `roslaunch aist_handeye_calibration mocap_handeye_calibration.launch camera_name:=realsenseD435 check:=true`
* activate `yolo6d` conda environment
    * `rosrun yolo6d_ros yolo6d_ros.py pnp`
* `roslaunch yolo6d_ros grasping.launch`
    * afterwards, follow instructions as displayed on the terminal.

* you should see output similar to this ![Alt text](images/onigiripick.png?raw=true "yolo6d pose")
