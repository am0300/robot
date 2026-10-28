#!/usr/bin/env python3
"""Fast structural validation, no Docker/Gazebo required."""
import ast
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

ROOT=Path(__file__).parent
c=yaml.safe_load((ROOT/'compose.yaml').read_text())
assert set(c['services']) == {'gazebo','ros2','lerobot'}
bridge=yaml.safe_load((ROOT/'ros2_app/bridge.yaml').read_text())
assert len(bridge) == 9
assert len([b for b in bridge if b['direction']=='ROS_TO_GZ']) == 6
s=ET.parse(ROOT/'worlds/pick_place.sdf').getroot()
assert len(s.findall('.//sensor[@type="camera"]')) == 1
assert len(s.findall('.//plugin[@filename="gz-sim-joint-position-controller-system"]')) == 6
assert len(s.findall('.//joint[@name="gripper"]')) == 1
# SDF Euler poses require x y z roll pitch yaw, not only xyz.
for pose in s.findall('.//pose'):
    assert len((pose.text or '').split()) == 6, f'Invalid SDF pose: {pose.text!r}'
assert 'self.publishers' not in (ROOT/'ros2_app/gateway.py').read_text()
for f in [ROOT/'worlds/generate_world.py',ROOT/'ros2_app/gateway.py',ROOT/'workspace/run_act.py']:
    ast.parse(f.read_text())
print('PASS: Compose, 9 bridges, SDF poses/camera, 6 controllers, gateway properties, Python syntax')
