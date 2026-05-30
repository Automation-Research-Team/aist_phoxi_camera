from launch                   import LaunchDescription
from launch.actions           import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions     import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions       import Node


def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_phoxi_camera'), 'launch',
                 'launch.py'])),
        Node(name='rviz', package='rviz2', executable='rviz2',
             output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution([
                     FindPackageShare('aist_phoxi_camera'), 'launch',
                     [LaunchConfiguration('camera_name'), '.rviz']
                 ])
             ]),
        Node(name='rqt_reconfigure', package='rqt_reconfigure',
             executable='rqt_reconfigure', output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
