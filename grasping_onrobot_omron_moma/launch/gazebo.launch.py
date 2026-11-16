import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import xacro


def build_description(gripper, mappings, controllers_file):
    share = get_package_share_directory('grasping_description_onrobot_omron_moma')
    path = os.path.join(share, 'xacro', f'ld250_tm12x_{gripper}.urdf.xacro')
    root = ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())
    for control in root.findall('ros2_control'):
        root.remove(control)
    for gazebo in root.findall('gazebo'):
        for plugin in gazebo.findall('plugin'):
            if plugin.get('filename') in ('libgz_ros2_control-system.so', 'libgazebo_ros_control.so'):
                gazebo.remove(plugin)
    ET.SubElement(root, 'link', {'name': 'world'})
    anchor = ET.SubElement(root, 'joint', {'name': 'simulation_world_anchor', 'type': 'fixed'})
    ET.SubElement(anchor, 'parent', {'link': 'world'})
    ET.SubElement(anchor, 'child', {'link': 'base_link'})
    control = ET.SubElement(root, 'ros2_control', {'name': 'GazeboSimSystem', 'type': 'system'})
    ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for joint_name in [f'joint_{number}' for number in range(1, 7)]:
        joint = ET.SubElement(control, 'joint', {'name': joint_name})
        ET.SubElement(joint, 'command_interface', {'name': 'position'})
        ET.SubElement(joint, 'state_interface', {'name': 'position'})
        ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
    if gripper in ('rg2', 'rg6'):
        joint = ET.SubElement(control, 'joint', {'name': 'finger_joint'})
        ET.SubElement(joint, 'command_interface', {'name': 'position'})
        ET.SubElement(joint, 'state_interface', {'name': 'position'})
        ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
        for joint_name in ('left_inner_knuckle_joint', 'left_inner_finger_joint',
                           'right_outer_knuckle_joint', 'right_inner_knuckle_joint', 'right_inner_finger_joint'):
            joint = ET.SubElement(control, 'joint', {'name': joint_name})
            ET.SubElement(joint, 'state_interface', {'name': 'position'})
            ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin', {
        'filename': 'libgz_ros2_control-system.so', 'name': 'gz_ros2_control::GazeboSimROS2ControlPlugin'})
    ET.SubElement(plugin, 'parameters').text = controllers_file
    return root


def create_simulation(context):
    gripper = LaunchConfiguration('gripper').perform(context)
    if gripper not in ('rg2', 'rg6', 'vg10', 'vgc10'):
        raise ValueError(f'Unsupported gripper: {gripper}')
    share = get_package_share_directory('grasping_onrobot_omron_moma')
    mappings = {name: LaunchConfiguration(name).perform(context) for name in
                ('mount_xyz', 'mount_rpy', 'tcp_xyz', 'tcp_rpy')}
    root = build_description(gripper, mappings, os.path.join(share, 'config', 'gazebo_controllers.yaml'))
    description = ET.tostring(root, encoding='unicode')
    resources = set()
    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith('package://'):
            resources.add(filename[len('package://'):].split('/', 1)[0])
    resource_paths = [os.path.dirname(get_package_share_directory(name)) for name in sorted(resources)]
    resource_paths.append(context.environment.get('GZ_SIM_RESOURCE_PATH', ''))
    world = os.path.join(get_package_share_directory('onrobot_gazebo'), 'worlds', 'empty.sdf')
    gazebo_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    nodes = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch),
                                 launch_arguments={'gz_args': [LaunchConfiguration('gz_args'), ' ', world]}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'default', '-name', f'ld250_tm12x_{gripper}', '-topic', '/robot_description'],
             output='screen'),
    ]
    for controller in ('joint_state_broadcaster', 'tmr_arm_with_base_controller'):
        nodes.append(Node(package='controller_manager', executable='spawner',
                          arguments=[controller, '--controller-manager-timeout', '120'], output='screen'))
    if gripper in ('rg2', 'rg6'):
        nodes.extend([
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_position_controller', '--controller-manager-timeout', '120'], output='screen'),
            Node(package='onrobot_control', executable='onrobot_simulated_rg_action_server',
                 namespace=f'onrobot_{gripper}', parameters=[{'model': gripper, 'use_sim_time': True}],
                 remappings=[('joint_states', '/joint_states'),
                             ('joint_position_controller/commands', '/joint_position_controller/commands')],
                 output='screen'),
        ])
    return nodes


def generate_launch_description():
    grasping_share = get_package_share_directory('grasping_control')
    share = get_package_share_directory('grasping_onrobot_omron_moma')
    names = ('gripper', 'mount_xyz', 'mount_rpy', 'tcp_xyz', 'tcp_rpy')
    return LaunchDescription([
        DeclareLaunchArgument('gripper', default_value='rg2', choices=['rg2', 'rg6', 'vg10', 'vgc10']),
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('tcp_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('gz_args', default_value='-r'),
        DeclareLaunchArgument('moveit', default_value='true'),
        DeclareLaunchArgument('motion_execution', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=create_simulation),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                         FindPackageShare(['ld250_tm12x_', LaunchConfiguration('gripper'), '_moveit_config']),
                         'launch', 'move_group.launch.py'])),
                                 condition=IfCondition(LaunchConfiguration('moveit')),
                                 launch_arguments={
                                     **{name: LaunchConfiguration(name) for name in names},
                                     'use_sim_time': 'true',
                                     'launch_rviz': LaunchConfiguration('rviz'),
                                 }.items()),
        GroupAction(condition=IfCondition(LaunchConfiguration('motion_execution')), actions=[
            SetParameter(name='use_sim_time', value=True),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(grasping_share, 'launch', 'motion_execution.launch.py')),
                                     launch_arguments={'motion_config_file': os.path.join(share, 'config', 'motion_simulation.yaml'),
                                                       'startup_activate_default_scene': 'false',
                                                       'launch_feasiblity_service': 'true'}.items()),
        ]),
    ])