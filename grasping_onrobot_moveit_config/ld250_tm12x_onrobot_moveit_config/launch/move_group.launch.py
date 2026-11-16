import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_parameters(gripper, mappings, use_sim_time=False):
    package = 'ld250_tm12x_onrobot_moveit_config'
    description = get_package_share_directory('grasping_description_onrobot_omron_moma')
    tm_share = get_package_share_directory('tm12x_moveit_config')
    robot = f'ld250_tm12x_{gripper}'
    model_share = get_package_share_directory(f'{robot}_moveit_config')
    config = (MoveItConfigsBuilder(robot, package_name=package)
              .robot_description(file_path=os.path.join(description, 'xacro', f'{robot}.urdf.xacro'), mappings=mappings)
              .robot_description_semantic(file_path=os.path.join(model_share, 'config', f'{robot}.srdf.xacro'))
              .robot_description_kinematics(file_path=os.path.join(model_share, 'config', 'kinematics.yaml'))
              .trajectory_execution(file_path=os.path.join(model_share, 'config', 'moveit_controllers.yaml'))
              .joint_limits(file_path=os.path.join(tm_share, 'config', 'joint_limits.yaml'))
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .to_moveit_configs())
    params = config.to_dict()
    params['use_sim_time'] = use_sim_time
    return params


def create_move_group(context):
    gripper = LaunchConfiguration('gripper').perform(context)
    if gripper not in ('rg2', 'rg6', 'vg10', 'vgc10'):
        raise ValueError(f'Unsupported gripper: {gripper}')
    mappings = {name: LaunchConfiguration(name).perform(context) for name in
                ('mount_xyz', 'mount_rpy', 'tcp_xyz', 'tcp_rpy')}
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    parameters = build_parameters(gripper, mappings, use_sim_time)
    rviz_parameters = {name: parameters[name] for name in
                       ('robot_description', 'robot_description_semantic',
                        'robot_description_kinematics', 'planning_pipelines', 'use_sim_time')}
    rviz_config = os.path.join(get_package_share_directory('ld250_tm12x_onrobot_moveit_config'),
                               'config', 'moveit.rviz')
    return [
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[parameters]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config], parameters=[rviz_parameters],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gripper', default_value='rg2', choices=['rg2', 'rg6', 'vg10', 'vgc10']),
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        OpaqueFunction(function=create_move_group),
    ])