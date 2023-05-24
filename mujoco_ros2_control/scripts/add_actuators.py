#!/bin/env python3
import xml.etree.ElementTree as ET
import sys

# Get the actuators element from the urdf (in robot/mujoco/actuators
in_tree = ET.parse(sys.argv[1])
in_root = in_tree.getroot()
mujoco = in_root.find('mujoco')
actuators = mujoco.find('actuator')

# Insert the actuator element in the mjcf file tree
out_tree = ET.parse(sys.argv[2])
out_root = out_tree.getroot()
out_root.insert(len(out_root), actuators)

# Write the resulting xml tree to the destination file
out_tree.write(sys.argv[3])