#!/bin/env python3
import getopt
import subprocess
import xml.etree.ElementTree as ET
import sys
import os
import uuid

if __name__ == "__main__":

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hs:i:o:", ["help", "scene=", "input=", "output=", "compile_executable=", "xacro_path="])
    except getopt.GetoptError:
        sys.exit(2)

    output_file = ""
    scene_file = ""
    input_files = []
    compile_executable = 'compile'

    for o, a in opts:
        if o in ("-h", "--help"):
            print('Usage: python script.py -i <input.xacro> -o <output.xml> [--compile_executable <compile_executable>]')
        if o in ("-s", "--scene"):
            scene_file = a
        if o in ("-i", "--input"):
            input_files.append(a)
        if o in ("-o", "--output"):
            output_file = a
        if o == "--compile_executable":
            compile_executable = a

    output_model_files = []

    for i, input_file in enumerate(input_files):
        name = str(uuid.uuid4())
        os.system('xacro ' + input_file + ' > /tmp/tmp_' + name + '.urdf')
        os.system(compile_executable + ' /tmp/tmp_' + name + '.xml')

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
        output_model_files.append('/tmp/' + name + '.xml')
        out_tree.write(output_model_files[-1])
        os.system('rm /tmp/tmp_' + name + '.*')

    for filename in output_model_files:
        in_tree = ET.parse(scene_file)
        in_root = in_tree.getroot()


    print("Saved mjcf xml file under " + output_file)

    exit(0)
