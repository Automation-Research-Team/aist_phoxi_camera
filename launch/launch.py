from launch                  import LaunchDescription
from launch.actions          import (DeclareLaunchArgument, OpaqueFunction,
                                     GroupAction)
from launch.substitutions    import (LaunchConfiguration, ThisLaunchFileDir,
                                     PathJoinSubstitution, EqualsSubstitution,
                                     IfElseSubstitution)
from launch.conditions       import IfCondition, UnlessCondition
from launch_ros.actions      import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

launch_arguments = [
    {'name':        'namespace',
     'default':     '',
     'description': 'namespace of the camera node'},
    {'name':        'camera_name',
     'default':     'phoxi',
     'description': 'node name of the camera'},
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring camera'},
    {'name':        'external_container',
     'default':     'false',
     'description': 'use existing external container',
     'choices':     ['true', 'false']},
    {'name':        'container',
     'default':     '',
     'description': 'name of internal or external component container'},
    {'name':        'vis',
     'default':     'false',
     'description': 'visualize camera outputs',
     'choices':     ['true', 'false']},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level',
     'choices':     ['debug', 'info', 'warn', 'error', 'fatal']},
    {'name':        'output',
     'default':     'screen',
     'description': 'pipe node output',
     'choices':     ['screen', 'log', 'both']}]

parameter_arguments = [
    {'name':        'id',
     'default':     'InstalledExamples-basic-example',
     'description': 'unique ID of the camera'}]


def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def set_configurable_parameters(args):
    return {arg['name']: LaunchConfiguration(arg['name']) for arg in args}

def launch_setup(context, param_args):
    config_file   = IfElseSubstitution(
                        EqualsSubstitution(
                            LaunchConfiguration('config_file'), ''),
                        PathJoinSubstitution([ThisLaunchFileDir(), '..',
                                              'config', 'default.yaml']),
                        LaunchConfiguration('config_file'))
    config_params = set_configurable_parameters(param_args)
    return [Node(namespace=LaunchConfiguration('namespace'),
                 name=LaunchConfiguration('camera_name'),
                 package='aist_phoxi_camera',
                 executable='aist_phoxi_camera_node',
                 parameters=[config_file, config_params],
                 output=LaunchConfiguration('output'),
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')],
                 emulate_tty=True,
                 condition=IfCondition(
                               EqualsSubstitution(
                                   LaunchConfiguration('container'), ''))),
            GroupAction(
                condition=UnlessCondition(
                              EqualsSubstitution(
                                  LaunchConfiguration('container'), '')),
                actions=[
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
                                namespace=LaunchConfiguration('namespace'),
                                name=LaunchConfiguration('camera_name'),
                                package='aist_phoxi_camera',
                                plugin='aist_phoxi_camera::Camera',
                                parameters=[config_file, config_params],
                                extra_arguments=[
                                    {'use_intra_process_comms': True}])])]),
            GroupAction(
                condition=IfCondition(LaunchConfiguration('vis')),
                actions=[
                    Node(name='rviz', package='rviz2', executable='rviz2',
                         output='screen',
                         arguments=['-d',
                                    PathJoinSubstitution([
                                        ThisLaunchFileDir(),
                                        'aist_phoxi_camera.rviz'])]),
                    Node(name='rqt_reconfigure', package='rqt_reconfigure',
                         executable='rqt_reconfigure', output='screen')])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments +
                                                      parameter_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
