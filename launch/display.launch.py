"""ROS 2 RViz display with selectable standalone URDF and legal knee initialization."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('walker_c1_description'))
    variant = LaunchConfiguration('variant').perform(context)
    if variant not in ('no_hands', 'with_hands'):
        raise ValueError('variant must be no_hands or with_hands')
    stem = 'walker_c1_with_hands' if variant == 'with_hands' else 'walker_c1'
    description = (share / 'urdf' / f'{stem}.urdf').read_text()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'zeros.L_knee_pitch_joint': 0.08, 'zeros.R_knee_pitch_joint': 0.08}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config/display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('variant', default_value='no_hands'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
