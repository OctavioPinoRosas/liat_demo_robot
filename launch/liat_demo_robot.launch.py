import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    TextSubstitution,
)
from launch_ros.actions import Node
import launch_ros

from interbotix_xs_modules.xs_common import get_interbotix_xsarm_models
from interbotix_xs_modules.xs_launch import declare_interbotix_xsarm_robot_description_launch_arguments


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='can0',
        description='CAN bus name, e.g. can0'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame id'
    )

    base_link_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base link frame id'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic_name',
        default_value='odom',
        description='Odometry topic name'
    )

    simulated_robot_arg = DeclareLaunchArgument(
        'simulated_robot',
        default_value='false',
        description='Whether running with simulator'
    )

    sim_control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='50',
        description='Simulation control loop update rate'
    )

    declared_arguments = [
        DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
            PathJoinSubstitution([
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('liat_demo_robot'), 'config', '')),
                joy_config,
                TextSubstitution(text='.config.yaml')
            ])
        ]),
        use_sim_time_arg,
        port_name_arg,
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        simulated_robot_arg,
        sim_control_rate_arg,
    ]

    hunter_base_node = Node(
        package='hunter_base',
        executable='hunter_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'port_name': LaunchConfiguration('port_name'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_topic_name': LaunchConfiguration('odom_topic_name'),
            'simulated_robot': LaunchConfiguration('simulated_robot'),
            'control_rate': LaunchConfiguration('control_rate'),
        }]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath],
        remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))},
    )

    declared_arguments.extend(declare_interbotix_xsarm_robot_description_launch_arguments())

    return LaunchDescription(
        declared_arguments + [
            hunter_base_node,
            joy_node,
            teleop_twist_joy_node,
        ]
    )


if __name__ == '__main__':
    generate_launch_description()
