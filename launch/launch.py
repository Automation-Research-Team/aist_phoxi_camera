import os
import yaml
from launch               import LaunchDescription
from launch_ros.actions   import Node
from launch.actions       import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


configurable_parameters = [{'name':        'camera_name',
                            'default':     'phoxi',
                            'description': 'camera unique name'},
                           {'name':        'camera_namespace',
                            'default':     '',
                            'description': 'namespace for camera'},
                           {'name':        'id',
                            'default':     'InstalledExamples-basic-example',
                            'description': 'choose device by serial number'},
                           {'name':        'config_file',
                            'default':     "''",
                            'description': 'yaml config file'},
                           {'name':        'log_level',
                            'default':     'info',
                            'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name':        'output',
                            'default':     'screen',
                            'description': 'pipe node output [screen|log]'}]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'],
                                  default_value=param['default'],
                                  description=param['description']) \
            for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) \
                 for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + \
                                       param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else \
                       yaml_to_dict(_config_file)
    return [Node(package='aist_phoxi_camera',
                 namespace=LaunchConfiguration('camera_namespace' + \
                                               param_name_suffix),
                 name=LaunchConfiguration('camera_name' + param_name_suffix),
                 executable='aist_phoxi_camera_node',
                 parameters=[params, params_from_file],
                 output=LaunchConfiguration('output' + param_name_suffix),
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level' + \
                                                param_name_suffix)],
                 emulate_tty=True)]

def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + \
        [OpaqueFunction(function=launch_setup,
                        kwargs={'params': set_configurable_parameters(
                                              configurable_parameters)})])
