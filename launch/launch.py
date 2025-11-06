from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction, GroupAction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               EqualsSubstitution)
from launch.conditions                 import IfCondition, UnlessCondition
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node, LoadComposableNodes
from launch_ros.descriptions           import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile

launch_arguments = [
    {
        'name':        'camera_name',
        'default':     'phoxi',
        'description': 'node name of the camera'
    },
    {
        'name':        'id',
        'default':     'InstalledExamples-basic-example',
        'description': 'unique ID of the camera'
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_phoxi_camera'), 'config',
                           'default.yaml']),
        'description': 'abolute path to YAML file for configuring camera'
    },
    {
        'name':        'external_container',
        'default':     'false',
        'description': 'use external container launched in advance',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'container',
        'default':     'camera_container',
        'description': 'name of internal or external component container'
    },
    {
        'name':        'vis',
        'default':     'false',
        'description': 'visualize camera outputs',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'screen',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]


def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):
    param_file = ParameterFile(LaunchConfiguration('param_file'),
                               allow_substs=True)
    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')],
             condition=UnlessCondition(
                           LaunchConfiguration('external_container'))),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name=LaunchConfiguration('camera_name'),
                    package='aist_phoxi_camera',
                    plugin='aist_phoxi_camera::Camera',
                    parameters=[param_file],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ]),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('vis')),
            actions=[
                Node(name='rviz', package='rviz2', executable='rviz2',
                     output='screen',
                     arguments=[
                         '-d',
                         PathJoinSubstitution([
                             FindPackageShare('aist_phoxi_camera'),
                             'launch',
                             [LaunchConfiguration('camera_name'), '.rviz']
                         ])
                     ]),
                Node(name='rqt_reconfigure', package='rqt_reconfigure',
                     executable='rqt_reconfigure', output='screen')
            ])
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
