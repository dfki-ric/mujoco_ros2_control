#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
import os
import uuid


## @file xacro2mjcf.py
# @brief Converts Xacro and URDF files into Mujoco MJCF XML file.
# @author Adrian Danzglock
# @date 2023
#
# @license GNU General Public License, version 3 (GPL-3.0)
# @copyright Copyright (c) 2023, DFKI GmbH
#
# This file is governed by the GNU General Public License, version 3 (GPL-3.0).
# The GPL-3.0 is a copyleft license that allows users to use, modify, and distribute software
# while ensuring that these freedoms are passed on to subsequent users. It requires that any
#  derivative works or modifications of the software be licensed under the GPL-3.0 as well.
# You should have received a copy of the GNU General Public License along with this program.
# If not, see https://www.gnu.org/licenses/gpl-3.0.html.
#
# This script is a ROS node that converts Xacro and URDF files into a Mujoco MJCF (MuJoCo Composite Format) XML file.
# It takes a list of input files, including Xacro and URDF files, and a target output file path as parameters. Additionally,
# it allows specifying a custom executable for compilation and a list of robot descriptions. The script processes the input files,
# resolves Xacro macros, and combines them into a single MJCF XML file. It also handles redundant mesh names to prevent crashes
# when using Mujoco.
#
# The main functionality of the script includes:
#   1. Parsing ROS parameters to get the input files, output file path, compile executable, robot descriptions, and Mujoco files path.
#   2. Creating a directory for storing temporary Mujoco files.
#   3. Converting robot descriptions to URDF files and adding them to the input files list.
#   4. Processing each input file:
#      - Converting Xacro files to URDF using the specified compile executable.
#      - Parsing the input file to extract relevant elements.
#      - Inserting the extracted elements into the MJCF XML file tree.
#   5. Handling URDF files and symbolic links.
#   6. Writing the resulting MJCF XML file containing all the processed elements.
#
# Usage:
#   1. Set the necessary ROS parameters in the launch file or command line:
#      - input_files: A list of Xacro and URDF files to be processed.
#      - output_file: The path of the output MJCF XML file.
#      - compile_executable (optional): The custom executable for Xacro/URDF compilation (default: 'compile').
#      - robot_descriptions (optional): A list of robot descriptions to be converted to URDF and processed.
#      - mujoco_files_path (optional): The path to store temporary Mujoco files (default: '/tmp/mujoco/').
#   2. Run the script as a ROS node.
#
# @note This script requires ROS and the rclpy Python package to be installed.
# @note The Mujoco software and its dependencies must be installed separately.
# @note Make sure to set appropriate file permissions for the script to run as an executable.




class Xacro2Mjcf(Node):

    ## @brief Initializes the Xacro2Mjcf node.
    def __init__(self):
        super().__init__('xacro2mjcf')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_files', rclpy.Parameter.Type.STRING_ARRAY),
                ('output_file', rclpy.Parameter.Type.STRING),
                ('compile_executable', 'compile'),
                ('robot_descriptions', rclpy.Parameter.Type.STRING_ARRAY),
                ('mujoco_files_path', "/tmp/mujoco/"),
            ]
        )

        # Get parameters
        input_files = self.get_parameter('input_files').value
        output_file = self.get_parameter('output_file').value
        compile_executable = self.get_parameter('compile_executable').value
        robot_descriptions = self.get_parameter('robot_descriptions').value
        mujoco_files_path = self.get_parameter('mujoco_files_path').value

        # Remove trailing slash from mujoco_files_path
        if mujoco_files_path.split("/")[-1] == '':
            mujoco_files_path = mujoco_files_path[:-1]

        # Create directory for Mujoco files
        os.system("rm -r " + mujoco_files_path)
        os.system("mkdir -p " + mujoco_files_path + "/meshes")

        output_model_files = []

        # Convert robot descriptions to URDF files
        for robot_description in robot_descriptions:
            name = str(uuid.uuid4())
            # change fixed joints to revolute joints
            tmp_urdf_tree = ET.ElementTree(ET.fromstring(robot_description))
            tmp_urdf_root = tmp_urdf_tree.getroot()

            # Create symlinks to used meshes in the tmp folder
            for mesh in self.get_elements(tmp_urdf_root, "mesh"):
                filename = mesh.get('filename')
                if filename:
                    source_file = filename[7:]
                    target_file = mujoco_files_path + "/meshes/" + source_file.replace("/", "_")
                    if source_file[-3:] == "stl" or source_file[-3:] == "STL":
                        if not os.path.exists(target_file):
                            os.symlink(source_file, target_file)
                        mesh.attrib['filename'] = "file://" + target_file
            # Merge mujoco elements
            mujoco_elements = self.get_elements(tmp_urdf_root, "mujoco")
            if mujoco_elements:
                if len(mujoco_elements) > 1:
                    for mujoco_element in self.get_elements(tmp_urdf_root, "mujoco")[1:]:
                        for element in mujoco_element:
                            self.get_logger().info(str(element))
                            mujoco_elements[0].append(element)
                        tmp_urdf_root.remove(mujoco_element)

                mujoco = mujoco_elements[0]
                referenced_links = []
                reference_elements = self.get_elements(mujoco, "reference")
                if reference_elements is not None:
                    for reference in reference_elements:
                        self.get_logger().info(str(reference.get("name")))
                        referenced_links.append(reference.get("name"))

                for joint_element in self.get_elements(tmp_urdf_root, "joint", "type", "fixed"):
                    if joint_element.find('child').attrib['link'] in referenced_links:
                        # Update the attribute
                        joint_element.set("type", "revolute")

                        # Add a new child element <new_child> with some content
                        limit = ET.Element('limit')
                        limit.attrib['effort'] = '0'
                        limit.attrib['lower'] = '0'
                        limit.attrib['upper'] = '1e-10'
                        limit.attrib['velocity'] = '0'
                        joint_element.append(limit)
                for link_element in self.get_elements(tmp_urdf_root, "link"):
                    # add inertial element with small values to the links of the fixed joints
                    if len(list(link_element)) == 0:
                        if link_element.attrib['name'] in referenced_links:
                            inertial = ET.Element('inertial')
                            mass = ET.Element('mass')
                            mass.attrib['value'] = '1e-10'
                            inertial.append(mass)
                            origin = ET.Element('origin')
                            origin.attrib['xyz'] = '0 0 0'
                            inertial.append(origin)
                            inertia = ET.Element('inertia')
                            inertia.attrib['ixx'] = '1e-10'
                            inertia.attrib['ixy'] = '0.0'
                            inertia.attrib['ixz'] = '0.0'
                            inertia.attrib['iyy'] = '1e-10'
                            inertia.attrib['iyz'] = '0.0'
                            inertia.attrib['izz'] = '1e-10'
                            inertial.append(inertia)
                            link_element.append(inertial)

            	
            output_tree = ET.ElementTree(tmp_urdf_root)
            ET.indent(output_tree, space="\t", level=0)
            output_tree.write(mujoco_files_path + '/' + name + '.urdf')
            input_files.append(mujoco_files_path + '/' + name + '.urdf')

        out_assets = ET.Element('asset')
        

        # Process input files
        for i, input_file in enumerate(input_files):
            filename = input_file
            if filename.split('.')[-1] == 'xacro' or filename.split('.')[-1] == 'urdf':
                name = filename.split('.')[-2].split('/')[-1]
                if name == 'urdf':
                    name = filename.split('.')[-3].split('/')[-1]

                if filename.split('.')[-1] == 'xacro':
                    # Convert Xacro to URDF
                    os.system('xacro ' + filename + ' > ' + mujoco_files_path + '/tmp_' + name + '.urdf')
                    os.system(
                        compile_executable + ' ' + mujoco_files_path + '/tmp_' + name + '.urdf ' +
                        mujoco_files_path + '/tmp_' + name + '.xml')
                    urdf_tree = ET.parse(mujoco_files_path + '/tmp_' + name + '.urdf')
                else:
                    os.system(compile_executable + ' ' + filename + ' ' + mujoco_files_path + '/tmp_' + name + '.xml')
                    urdf_tree = ET.parse(filename)

                self.urdf_root = urdf_tree.getroot()
                mjcf_tree = ET.parse(mujoco_files_path + '/tmp_' + name + '.xml')
                self.mjcf_root = mjcf_tree.getroot()

                # Insert elements into the MJCF file tree
                mujoco = self.urdf_root.find('mujoco')
                if mujoco is not None:
                    for element in mujoco:
                        if element.tag == 'reference':
                            body_name = element.attrib['name']
                            mj_elements = self.get_elements(self.mjcf_root, 'body', 'name', body_name)
                            if mj_elements:
                                for child in element:
                                    if 'body' in child.tag:
                                        for attrib in child.attrib:
                                            mj_elements[0].set(attrib, child.attrib[attrib])
                                            self.get_logger().info("added attrib " + str(child.tag))
                                    else:
                                        tag_elements = self.get_elements(mj_elements[0], child.tag)
                                        if tag_elements:
                                            for attrib in child.attrib:
                                                for tag_element in tag_elements:
                                                    tag_element.set(attrib, child.attrib[attrib])
                                                    #self.get_logger().info("added attrib to " + str(joint.tag))
                                        else:
                                            mj_elements[0].insert(0, child)
                            else:
                                self.get_logger().error("Body " + body_name + " not found")
                                #rclpy.shutdown()
                        elif element.tag != 'compiler':
                            self.mjcf_root.insert(len(self.mjcf_root), element)

                if mjcf_tree.find('asset') is not None:
                    for element in mjcf_tree.find('asset'):
                        exist = False
                        for e in out_assets:
                            if e.attrib == element.attrib:
                                exist = True
                        if not exist:
                            out_assets.append(element)

                    self.mjcf_root.remove(mjcf_tree.find('asset'))
                # Write the resulting xml tree to the destination file
                mjcf_tree.write(mujoco_files_path + '/' + name + '.xml')
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

    def get_elements(self, parent, tag, attrib=None, value=None):
        elements = []
        if parent is None:
            return None
        for child in parent:
            if child is None:
                return None
            if child.tag == tag:
                if attrib is None or attrib in child.attrib.keys():
                    if value is None or child.attrib[attrib] == value:
                        elements.append(child)
            elements += self.get_elements(child, tag, attrib, value)
        return elements



## @brief Main function to initialize and run the Xacro2Mjcf node.
#  @param args: Command-line arguments.
def main(args=None):
    rclpy.init(args=args)

    xacro2mjcf = Xacro2Mjcf()
    xacro2mjcf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

