from launch               import LaunchDescription
from launch.actions       import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  ThisLaunchFileDir)
from launch_ros.actions   import Node


def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'launch.py'])),
        Node(name='rviz', package='rviz2', executable='rviz2',
             output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution([
                     ThisLaunchFileDir(),
                     [LaunchConfiguration('camera_name'), '.rviz']
                 ])
             ]),
        Node(name='rqt_reconfigure', package='rqt_reconfigure',
             executable='rqt_reconfigure', output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
