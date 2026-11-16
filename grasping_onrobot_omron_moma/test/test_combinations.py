import importlib.util
import os
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory


def load_launch(package, filename):
    path = os.path.join(get_package_share_directory(package), 'launch', filename)
    spec = importlib.util.spec_from_file_location(package, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('gripper', ['rg2', 'rg6', 'vg10', 'vgc10'])
def test_combined_moveit_and_gazebo_models(gripper):
    moveit = load_launch('ld250_tm12x_onrobot_moveit_config', 'move_group.launch.py')
    gazebo = load_launch('grasping_onrobot_omron_moma', 'gazebo.launch.py')
    parameters = moveit.build_parameters(gripper, {})
    robot = ET.fromstring(parameters['robot_description'])
    semantic = ET.fromstring(parameters['robot_description_semantic'])
    links = {link.get('name') for link in robot.findall('link')}
    joints = {joint.get('name') for joint in robot.findall('joint')}
    children = [joint.find('child').get('link') for joint in robot.findall('joint')]
    assert robot.get('name') == semantic.get('name') == f'ld250_tm12x_{gripper}'
    assert len(children) == len(set(children))
    assert links - set(children) == {'base_link'}
    assert f'onrobot_{gripper}_base_link' in links
    assert 'tcp' in links
    for collision in semantic.findall('disable_collisions'):
        assert collision.get('link1') in links
        assert collision.get('link2') in links
    for joint in semantic.findall('.//joint') + semantic.findall('passive_joint'):
        assert joint.get('name') in joints
    simulation = gazebo.build_description(gripper, {}, '/tmp/controllers.yaml')
    plugins = [plugin.get('filename') for plugin in simulation.findall('.//plugin')
               if 'control' in plugin.get('filename', '')]
    assert plugins == ['libgz_ros2_control-system.so']
    assert len(simulation.findall('ros2_control')) == 1
    assert (simulation.find("ros2_control/joint[@name='finger_joint']") is not None) == (gripper in ('rg2', 'rg6'))