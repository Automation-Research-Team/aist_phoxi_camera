cmake_minimum_required(VERSION 2.8.3)
project(aist_robotiq)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
    CModelCommand.msg CModelStatus.msg
)

generate_messages()

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS
    gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include
)

add_library(
    gazebo_mimic_joint_plugin
    src/mimic_joint_plugin.cpp
)
target_link_libraries(
    gazebo_mimic_joint_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

add_library(
    gazebo_disable_link_plugin
    src/disable_link_plugin.cpp
)
target_link_libraries(
    gazebo_disable_link_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)
