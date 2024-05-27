import os
import yaml
from launch                   import LaunchDescription
from launch.actions           import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions     import LaunchConfiguration, PathJoinSubstitution
from launch.conditions        import IfCondition, UnlessCondition
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions  import ComposableNode

launch_arguments    = [{'name':        'namespace',
                        'default':     '',
                        'description': 'namespace for camera'},
                       {'name':        'camera_name',
                        'default':     'phoxi',
                        'description': 'camera unique name'},
                       {'name':        'config_dir',
                        'default':     '',
                        'description': 'directory path containing yaml config file for camera'},
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

def load_parameters(config_dir, camera_name):
    if config_dir == '':
        return {}
    with open(os.path.join(config_dir, camera_name + '.yaml'), 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, param_args):
    namespace = LaunchConfiguration('namespace')
    name      = LaunchConfiguration('camera_name')
    params    = load_parameters(
                    LaunchConfiguration('config_dir').perform(context),
                    name.perform(context))
    actions   = declare_launch_arguments(param_args, params)
    params   |= set_configurable_parameters(param_args)
    output    = LaunchConfiguration('output')
    container = LaunchConfiguration('container').perform(context)
    if container == '':
        actions.append(Node(namespace=namespace, name=name,
                            package='aist_phoxi_camera',
                            executable='aist_phoxi_camera_node',
                            parameters=[params],
                            output=output,
                            arguments=['--ros-args', '--log-level',
                                       LaunchConfiguration('log_level')],
                            emulate_tty=True))
    else:
        actions += [Node(name=container,
                         package='rclcpp_components',
                         executable='component_container',
                         output=output,
                         condition=UnlessCondition(
                             LaunchConfiguration('external_container'))),
                    LoadComposableNodes(
                        target_container=container,
                        composable_node_descriptions=[
                            ComposableNode(namespace=namespace, name=name,
                                           package='aist_phoxi_camera',
                                           plugin='aist_phoxi_camera::Camera',
                                           parameters=[params],
                                           extra_arguments=[
                                             {'use_intra_process_comms': True}]
                                           )])]
    actions.append(Node(name='rviz',
                        package='rviz2', executable='rviz2', output='screen',
                        arguments=['-d',
                                   PathJoinSubstitution([
                                       FindPackageShare('aist_phoxi_camera'),
                                       'launch', 'aist_phoxi_camera.rviz'])],
                        condition=IfCondition(LaunchConfiguration('vis'))))
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
