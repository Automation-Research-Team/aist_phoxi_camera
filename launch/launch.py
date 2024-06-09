import yaml
from launch                   import LaunchDescription
from launch.actions           import (DeclareLaunchArgument, OpaqueFunction,
                                      GroupAction)
from launch.substitutions     import LaunchConfiguration, PathJoinSubstitution
from launch.conditions        import (IfCondition, UnlessCondition,
                                      LaunchConfigurationEquals,
                                      LaunchConfigurationNotEquals)
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions  import ComposableNode

launch_arguments    = [{'name':        'namespace',
                        'default':     '',
                        'description': 'namespace for camera'},
                       {'name':        'camera_name',
                        'default':     'phoxi',
                        'description': 'camera unique name'},
                       {'name':        'config_file',
                        'default':     '',
                        'description': 'path to YAML file for configuring camera'},
                       {'name':        'external_container',
                        'default':     'false',
                        'description': 'use existing external container'},
                       {'name':        'container',
                        'default':     '',
                        'description': 'name of internal or external component container'},
                       {'name':        'vis',
                        'default':     'false',
                        'description': 'visualize camera outputs'},
                       {'name':        'log_level',
                        'default':     'info',
                        'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                       {'name':        'output',
                        'default':     'both',
                        'description': 'pipe node output [screen|log]'}]
parameter_arguments = [{'name':        'id',
                        'default':     'InstalledExamples-basic-example',
                        'description': 'choose device by serial number'},
                       {'name':        'rate',
                        'default':     '10.0',
                        'description': 'rate of publishing topics'}]

def declare_launch_arguments(args, defaults={}):
    return [DeclareLaunchArgument(
                arg['name'],
                default_value=str(defaults.get(arg['name'], arg['default'])),
                description=arg['description']) \
            for arg in args]

def set_configurable_parameters(args):
    return dict([(arg['name'], LaunchConfiguration(arg['name'])) \
                 for arg in args])

def load_parameters(config_file):
    if config_file == '':
        return {}
    with open(config_file, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, param_args):
    params   = load_parameters(
                   LaunchConfiguration('config_file').perform(context))
    actions  = declare_launch_arguments(param_args, params)
    params  |= set_configurable_parameters(param_args)
    actions += [Node(namespace=LaunchConfiguration('namespace'),
                     name=LaunchConfiguration('camera_name'),
                     package='aist_phoxi_camera',
                     executable='aist_phoxi_camera_node',
                     parameters=[params],
                     output=LaunchConfiguration('output'),
                     arguments=['--ros-args', '--log-level',
                                LaunchConfiguration('log_level')],
                     emulate_tty=True,
                     condition=LaunchConfigurationEquals('container', '')),
                GroupAction(
                    condition=LaunchConfigurationNotEquals('container', ''),
                    actions=[
                        Node(name=LaunchConfiguration('container'),
                             package='rclcpp_components',
                             executable='component_container',
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
                                    parameters=[params],
                                    extra_arguments=[
                                        {'use_intra_process_comms': True}])])]),
                GroupAction(
                    condition=IfCondition(LaunchConfiguration('vis')),
                    actions=[
                        Node(name='rviz', package='rviz2', executable='rviz2',
                             output='screen',
                             arguments=['-d',
                                 PathJoinSubstitution([
                                     FindPackageShare('aist_phoxi_camera'),
                                     'launch', 'aist_phoxi_camera.rviz'])]),
                        Node(name='rqt_reconfigure', package='rqt_reconfigure',
                             executable='rqt_reconfigure', output='screen')])]
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
