# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from typing import List

from ament_index_python import get_package_share_directory
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    threshold_launch_arg = LaunchConfiguration('threshold')
    controller_launch_arg = LaunchConfiguration('controller')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    robot_model_launch_arg = LaunchConfiguration('robot_model')
 
    config_filepath_arg = LaunchConfiguration('config_filepath')
    joy_vel_arg = LaunchConfiguration('joy_vel')
    config_arg = TextSubstitution(text='${controller}.config.yaml')
    config_filepath_arg = PathJoinSubstitution(
        [FindPackageShare('liat_demo_robot'), 'config', '${controller}.config.yaml']
    )

    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    port_name_arg = LaunchConfiguration('port_name')           
    odom_frame_arg = LaunchConfiguration('odom_frame')
    base_frame_arg = LaunchConfiguration('base_frame')
    odom_topic_name_arg = LaunchConfiguration('odom_topic_name')
    simulated_robot_arg = LaunchConfiguration('simulated_robot')
    control_rate_arg = LaunchConfiguration('control_rate')
    
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        emulate_tty=True,
        )
    
    joy_node = Node(
        name='joy_node',
        package='joy',
        executable='joy_node',
        namespace=robot_name_launch,
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
        remappings=[
            ('joy', 'commands/joy_raw'),
        ],
    )

    xsarm_joy_node = Node(
        name='xsarm_joy',
        package='liat_demo_robot',
        executable='xsarm_joy',
        namespace=robot_name_launch,
        parameters=[{
            'threshold': threshold_launch_arg,
            'controller': controller_launch_arg
        }],
    )

    xsarm_robot_node = Node(
        package='interbotix_xsarm_joy',
        executable='xsarm_robot.py',
        namespace=robot_name_launch,
        parameters=[{
            'robot_model': robot_model_launch_arg,
        }],
        arguments=[
            '--robot_model', robot_model_launch_arg.perform(context),
            '--robot_name', robot_name_launch.perform(context),
        ],
    )

    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch,
            'base_link_frame': base_link_frame_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'use_sim': use_sim_launch_arg,
            'robot_description': robot_description_launch_arg,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg)
    )
    
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'config_filepath': config_filepath_arg}],
        remappings=[('joy_vel', joy_vel_arg)],
    )

    hunter_base_node = Node(
        package='hunter_base',
        executable='hunter_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': use_sim_time_arg,
                'port_name': port_name_arg,                
                'odom_frame': odom_frame_arg,
                'base_frame': base_frame_arg,
                'odom_topic_name': odom_topic_name_arg,
                'simulated_robot': simulated_robot_arg,
                'control_rate': control_rate_arg,
        }])

    return [
        xsarm_control_launch,
        joy_node,
        xsarm_joy_node,
        xsarm_robot_node,
        camera_node,

        teleop_twist_joy,
        hunter_base_node,
    ]


def generate_launch_description():
    declared_arguments: List[DeclareLaunchArgument] = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_joy'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'threshold',
            default_value='0.75',
            description=(
                'value from 0 to 1 defining joystick sensitivity; a larger number means the '
                'joystick should be less sensitive.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller',
            default_value='xbox',
            choices=('ps4', 'ps3', 'xbox'),
            description='type of controller.',

        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if xsarm_control should be launched - set to `false` if you would like to '
                'run your own version of this file separately.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's"
                ' motion; if `false`, the real DYNAMIXEL driver node is run.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'joy_vel',
            default_value='cmd_vel',
            description='velocity of cmd joy.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'port_name',
            default_value='can0',
            description='CAN bus name, e.g. can0.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry frame id.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base link frame id.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'odom_topic_name',
            default_value='odom',
            description='Odometry topic name.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'simulated_robot',
            default_value='false',
            description='Whether running with simulator.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'control_rate',
            default_value='50',
            description='Simulation control loop update rate.',
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments()
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
