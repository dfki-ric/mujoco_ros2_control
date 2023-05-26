#!/bin/env python3
import subprocess
import xml.etree.ElementTree as ET
import sys
import os

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print('Usage: python script.py <input.xacro> <output.xml> [--compile_executable <compile_executable>]')
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    compile_executable = 'compile'

    if '--xacro_path' in sys.argv:
        xacro_index = sys.argv.index('--xacro_path')
        if xacro_index + 1 < len(sys.argv):
            xacro_path = sys.argv[xacro_index + 1]

    if '--compile_executable' in sys.argv:
        compile_index = sys.argv.index('--compile_executable')
        if compile_index + 1 < len(sys.argv):
            compile_executable = sys.argv[compile_index + 1]

    os.system('xacro ' + input_file + ' > /tmp/tmp_urdf.urdf')
    os.system(compile_executable + ' /tmp/tmp_urdf.urdf /tmp/tmp_mjcf.xml')

    # Get the actuators element from the urdf (in robot/mujoco/actuators
    in_tree = ET.parse('/tmp/tmp_urdf.urdf')
    in_root = in_tree.getroot()
    mujoco = in_root.find('mujoco')
    # Insert the actuator element in the mjcf file tree
    out_tree = ET.parse('/tmp/tmp_mjcf.xml')
    out_root = out_tree.getroot()
    for element in mujoco:
        if element.tag != 'compiler':
            out_root.insert(len(out_root), element)

    # Write the resulting xml tree to the destination file
    out_tree.write(output_file)
    os.system('rm /tmp/tmp_mjcf.xml /tmp/tmp_urdf.urdf')
    print("Saved mjcf xml file under " + output_file)

    exit(0)
