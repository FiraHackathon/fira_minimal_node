from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from romea_mobile_base_description import get_mobile_base_description

import yaml


def get_control_parameters(filename):
    with open(filename) as file:
        return yaml.safe_load(file.read())

def get_control_node_name(robot_type):
    if robot_type == "adap2e":
        return "two_axle_steering_minimal_path_following_node"
    else:
        pass

def launch_setup(context, *args, **kargs):
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)

    config_file = LaunchConfiguration('configuration_file').perform(context)
    control_params = get_control_parameters(config_file)
    control_node_name = get_control_node_name(robot_type)

    
    return [
        Node(
            package='fira_minimal_node',
            executable=control_node_name,
            name='path_following2',
            exec_name='path_following2',
            namespace=robot_namespace,
            parameters=[control_params],
            remappings=[
                ('cmd_mux/subscribe','base/cmd_mux/subscribe'),
                ('cmd_mux/unsubscribe','base/cmd_mux/unsubscribe'),
                ('odometry', 'base/controller/odometry'),
                ('joy', 'joystick/joy'),
            ],
            # prefix='terminator -x gdbserver localhost:1337',
            # prefix='terminator -x gdb --args',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type'),
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('robot_namespace', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('configuration_file'),
        OpaqueFunction(function=launch_setup)
    ])
