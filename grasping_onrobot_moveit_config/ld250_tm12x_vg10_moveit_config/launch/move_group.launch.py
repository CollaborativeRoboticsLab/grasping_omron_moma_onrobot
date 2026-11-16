import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ld250_tm12x_onrobot_moveit_config')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'move_group.launch.py')),
            launch_arguments={
                'gripper': 'vg10',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items()),
    ])