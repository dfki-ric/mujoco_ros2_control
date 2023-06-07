#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
import os
import uuid


class Xacro2Mjcf(Node):

    def __init__(self):
        super().__init__('xacro2mjcf')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_files', rclpy.Parameter.Type.STRING_ARRAY),
                ('output_file', rclpy.Parameter.Type.STRING),
                ('compile_executable', 'compile'),
                ('robot_descriptions', rclpy.Parameter.Type.STRING_ARRAY),
                ('mujoco_files_path', "/tmp/mujoco/")
            ]
        )
        input_files = self.get_parameter('input_files').value
        output_file = self.get_parameter('output_file').value
        compile_executable = self.get_parameter('compile_executable').value
        robot_descriptions = self.get_parameter('robot_descriptions').value
        mujoco_files_path = self.get_parameter('mujoco_files_path').value
        if mujoco_files_path.split("/")[-1] == '':
            mujoco_files_path = mujoco_files_path[:-1]
        output_model_files = []
        os.system("rm -r " + mujoco_files_path)
        os.system("mkdir " + mujoco_files_path)

        for robot_description in robot_descriptions:
            name = str(uuid.uuid4())
            with open(mujoco_files_path + '/' + name + '.urdf', 'w') as f:
                f.write(str(robot_description))
            input_files.append(mujoco_files_path + '/' + name + '.urdf')

        # Store assets because redundant mesh names cause in crash from mujoco
        out_assets = ET.Element('asset')

        for i, input_file in enumerate(input_files):
            filename = input_file
            if filename.split('.')[-1] == 'xacro' or filename.split('.')[-1] == 'urdf':
                name = filename.split('.')[-2].split('/')[-1]
                if filename.split('.')[-1] == 'xacro':
                    os.system('xacro ' + filename + ' > ' + mujoco_files_path + '/tmp_' + name + '.urdf')
                    os.system(compile_executable + ' ' + mujoco_files_path + '/tmp_' + name + '.urdf ' + mujoco_files_path + '/tmp_' + name + '.xml')

                    # Get the actuators element from the urdf (in robot/mujoco/actuators
                    in_tree = ET.parse(mujoco_files_path + '/tmp_' + name + '.urdf')
                else:
                    os.system(compile_executable + ' ' + filename + ' ' + mujoco_files_path +'/tmp_' + name + '.xml')
                    # Get the actuators element from the urdf (in robot/mujoco/actuators
                    in_tree = ET.parse(filename)

                in_root = in_tree.getroot()
                # Insert the elements in the mjcf file tree
                out_tree = ET.parse(mujoco_files_path + '/tmp_' + name + '.xml')
                out_root = out_tree.getroot()

                mujoco = in_root.find('mujoco')
                if mujoco is not None:
                    for element in mujoco:
                        if element.tag != 'compiler':
                            out_root.insert(len(out_root), element)
                if out_tree.find('asset') is not None:
                    for element in out_tree.find('asset'):
                        exist = False
                        for e in out_assets:
                            if e.attrib == element.attrib:
                                exist = True
                        if not exist:
                            out_assets.append(element)

                    out_root.remove(out_tree.find('asset'))
                # Write the resulting xml tree to the destination file
                out_tree.write(mujoco_files_path + '/' + name + '.xml')
                #os.system('rm /tmp/tmp_' + name + '.*')
                output_model_files.append(name + '.xml')
            elif input_file.split('.')[-1] == 'xml':
                name = input_file.split('/')[-1]
                os.system('ln -s ' + input_file + ' ' + mujoco_files_path + '/' + name)
                output_model_files.append(name)
            else:
                self.get_logger().error("Wrong file ending, are you using xacro or xml?")
                self.destroy_node()
                rclpy.shutdown()

        output_xml = ET.Element('mujoco')
        output_xml.append(out_assets)
        for filename in output_model_files:
            output_xml.append(ET.Element('include', {'file': filename}))
        output_tree = ET.ElementTree(output_xml)
        output_tree.write(output_file)

        self.get_logger().info("Saved mjcf xml file under " + output_file)
        self.destroy_node()
        exit(0)


def main(args=None):
    rclpy.init(args=args)

    xacro2mjcf = Xacro2Mjcf()
    xacro2mjcf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
