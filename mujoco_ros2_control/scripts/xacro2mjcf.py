#!/usr/bin/env python3
import os
import subprocess
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import xml.etree.ElementTree as ET
import uuid
import collections
import copy
import hashlib
import math

from dae2stl import MAX_TRIANGLES_PER_MESH, convert_dae_to_stl, extract_triangle_groups, write_binary_stl
from urdf2mjcf import create_mjcf_from_urdf


# @file xacro2mjcf.py
# @brief Converts Xacro and URDF files into Mujoco MJCF XML file.
# @author Adrian Danzglock
# @date 2023
#
# @license BSD 3-Clause License
# @copyright Copyright (c) 2023, DFKI GmbH
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
#    and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of DFKI GmbH nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
                ('robot_descriptions', rclpy.Parameter.Type.STRING_ARRAY),
                ('xacro_args', rclpy.Parameter.Type.STRING_ARRAY),
                ('mujoco_files_path', "/tmp/mujoco/"),
                ('floating', False),
                ('initial_position', "0 0 0"),
                ('initial_orientation', "0 0 0"),
                ('base_link', "")
            ]
        )

        # Get parameters
        input_files = self.get_parameter('input_files').value
        output_file = self.get_parameter('output_file').value
        robot_descriptions = self.get_parameter('robot_descriptions').value
        try:
            xacro_args = self.get_parameter('xacro_args').value
        except rclpy.exceptions.ParameterUninitializedException:
            xacro_args = []
        mujoco_files_path = self.get_parameter('mujoco_files_path').value

        initial_position = self.get_parameter('initial_position').value # x y z
        if len(initial_position.split(" ")) != 3:
            initial_position = "0 0 0"
        initial_orientation = self.get_parameter('initial_orientation').value # r p y
        if len(initial_orientation.split(" ")) != 3:
            initial_orientation = "0 0 0"
        base_link = self.get_parameter('base_link').value
        floating = self.get_parameter('floating').value

        # Remove trailing slash from mujoco_files_path
        if mujoco_files_path.split("/")[-1] == '':
            mujoco_files_path = mujoco_files_path[:-1]

        # Create directory for Mujoco files
        os.system("rm -r " + mujoco_files_path)
        os.system("mkdir -p " + mujoco_files_path + "/meshes")

        output_model_files = []
        combined_qpos = []

        # Convert robot descriptions to URDF files
        for i, robot_description in enumerate(robot_descriptions):
            tmp_urdf_tree = ET.ElementTree(ET.fromstring(robot_description))
            tmp_urdf_root = tmp_urdf_tree.getroot()
            name = tmp_urdf_root.attrib["name"]
            if not name:
                name = str(uuid.uuid4())

            output_tree = ET.ElementTree(tmp_urdf_root)
            ET.indent(output_tree, space="\t", level=0)
            output_tree.write(mujoco_files_path + '/' + name + '.urdf')
            input_files.insert(i, mujoco_files_path + '/' + name + '.urdf')

        out_assets = ET.Element('asset')
        out_compiler = ET.Element('compiler')
        out_option = ET.Element('option')


        # Process input files
        for i, input_file in enumerate(input_files):
            filename = input_file
            if filename.split('.')[-1] == 'xacro' or filename.split('.')[-1] == 'urdf':
                name = filename.split('.')[-2].split('/')[-1]
                if name == 'urdf':
                    name = filename.split('.')[-3].split('/')[-1]

                if filename.split('.')[-1] == 'xacro':
                    # Convert Xacro to URDF
                    with open(mujoco_files_path + '/tmp_' + name + '.urdf', 'w', encoding='utf-8') as output:
                        subprocess.run(
                            ['xacro', filename, *xacro_args],
                            check=True,
                            stdout=output,
                        )
                    urdf_tree = ET.parse(mujoco_files_path + '/tmp_' + name + '.urdf')
                    tmp_urdf_root = urdf_tree.getroot()

                    if len(base_link) > 0:
                        robot = tmp_urdf_root
                        link = ET.Element("link", {"name": "world"})
                        joint = ET.Element("joint", {"name": "world_to_base", "type": "floating" if floating else "fixed"})

                        parent = ET.Element("parent", {"link": "world"})
                        child = ET.Element("child", {"link": base_link})
                        origin = ET.Element("origin", {"rpy": initial_orientation, "xyz": initial_position})
                        joint.append(parent)
                        joint.append(child)
                        joint.append(origin)
                        robot.insert(0, link)
                        robot.insert(1, joint)

                    self.convert_camera_links(tmp_urdf_root)
                    self.correct_visual_mesh(tmp_urdf_root)
                    self.expand_dae_visuals(tmp_urdf_root, mujoco_files_path)
                    self.create_symlinks(tmp_urdf_root, mujoco_files_path)

                    output_tree = ET.ElementTree(tmp_urdf_root)
                    ET.indent(output_tree, space="\t", level=0)
                    output_tree.write(mujoco_files_path + '/tmp_' + name + '.urdf')

                    create_mjcf_from_urdf(mujoco_files_path + '/tmp_' + name + '.urdf', mujoco_files_path + '/tmp_' + name + '.xml')
                else:
                    urdf_tree = ET.parse(filename)
                    tmp_urdf_root = urdf_tree.getroot()

                    if base_link:
                        robot = tmp_urdf_root
                        link = ET.Element("link", {"name": "world"})
                        joint = ET.Element("joint", {"name": "world_to_base", "type": "floating" if floating else "fixed"})

                        parent = ET.Element("parent", {"link": "world"})
                        child = ET.Element("child", {"link": base_link})
                        origin = ET.Element("origin", {"rpy": initial_orientation, "xyz": initial_position})
                        joint.append(parent)
                        joint.append(child)
                        joint.append(origin)
                        robot.insert(0, link)
                        robot.insert(1, joint)

                    self.convert_camera_links(tmp_urdf_root)
                    self.correct_visual_mesh(tmp_urdf_root)
                    self.expand_dae_visuals(tmp_urdf_root, mujoco_files_path)
                    self.create_symlinks(tmp_urdf_root, mujoco_files_path)

                    output_tree = ET.ElementTree(tmp_urdf_root)
                    ET.indent(output_tree, space="\t", level=0)
                    output_tree.write(mujoco_files_path + '/tmp_' + filename.split('/')[-1])
                    create_mjcf_from_urdf(mujoco_files_path + '/tmp_' + filename.split('/')[-1], mujoco_files_path + '/tmp_' + name + '.xml')

                self.urdf_root = urdf_tree.getroot()
                mjcf_tree = ET.parse(mujoco_files_path + '/tmp_' + name + '.xml')

                self.mjcf_root = mjcf_tree.getroot()

                # Surface-only visual meshes (for example the exact NVIDIA
                # electrical fixture OBJs) have no enclosed volume. MuJoCo can
                # still render them when their mesh inertia mode is "shell".
                visual_meshes = {
                    geom.get('mesh')
                    for geom in self.mjcf_root.iter('geom')
                    if geom.get('group') == '0' and geom.get('mesh')
                }
                asset = self.mjcf_root.find('asset')
                if asset is not None:
                    for mesh in asset.findall('mesh'):
                        mesh_name = mesh.get('name', '')
                        mesh_file = mesh.get('file', '')
                        is_electrical_fixture = (
                            'electrical_connectors' in mesh_file
                            or mesh_name.startswith('nema_')
                        )
                        if (mesh_name in visual_meshes
                                and is_electrical_fixture):
                            mesh.set('inertia', 'shell')

                # Add limited=true to all joints with range (limits)
                joints = self.get_elements(self.mjcf_root, 'joint', 'range')
                for joint in joints:
                    joint.set("limited", "true")

                initial_positions = {}
                for ros2_control in self.urdf_root.iter('ros2_control'):
                    for ctrl_joint in ros2_control.findall('joint'):
                        jname = ctrl_joint.get('name')
                        if jname is None:
                            continue
                        for si in ctrl_joint.findall('state_interface'):
                            if si.get('name') != 'position':
                                continue
                            for param in si.findall('param'):
                                if param.get('name') == 'initial_value' and param.text:
                                    try:
                                        initial_positions[jname] = float(param.text)
                                    except ValueError:
                                        pass

                # Map every element to its parent so we can find the body that
                # owns a given joint.
                parent_map = {child: parent
                              for parent in self.mjcf_root.iter()
                              for child in parent}

                def free_joint_qpos(joint):
                    """Initial qpos for a free joint = owning body's compiled pose.
                    """
                    body = parent_map.get(joint)
                    pos = [0.0, 0.0, 0.0]
                    quat = [1.0, 0.0, 0.0, 0.0]
                    if body is not None:
                        if body.get('pos'):
                            pos = [float(v) for v in body.get('pos').split()]
                        if body.get('quat'):
                            quat = [float(v) for v in body.get('quat').split()]
                    return pos + quat

                # Assemble qpos in MJCF joint order (document order == qpos order).
                qpos = []
                for joint in self.mjcf_root.iter('joint'):
                    jtype = joint.get('type', 'hinge')
                    if jtype == 'free':
                        qpos += free_joint_qpos(joint)
                    elif jtype == 'ball':
                        qpos += [1.0, 0.0, 0.0, 0.0]
                    else:  # hinge or slide -> 1 dof
                        qpos.append(initial_positions.get(joint.get('name'), 0.0))

                combined_qpos.extend(qpos)

                # Insert elements into the MJCF file tree
                mujoco = self.urdf_root.find('mujoco')
                parent_map = {c: p for p in mjcf_tree.iter() for c in p}
                if mujoco is not None:
                    for element in mujoco:
                        # TODO: reference for geom by name
                        if element.tag == 'reference':
                            reference_name = element.attrib['name']
                            mj_elements = self.get_elements(self.mjcf_root, 'body', 'name', reference_name)
                            if mj_elements:
                                for child in element:
                                    if 'camera' in child.tag:
                                        mj_elements[0].insert(0, child)
                                    if 'body' in child.tag:
                                        for attrib in child.attrib:
                                            mj_elements[0].set(attrib, child.attrib[attrib])
                                            self.get_logger().debug("added attrib " + str(child.tag))
                                    if 'site' in child.tag:
                                        mj_elements[0].insert(0, child)

                                    else:
                                        if 'name' in child.attrib:
                                            tag_elements = self.get_elements(mj_elements[0], child.tag, 'name', child.attrib['name'])
                                        else:
                                            tag_elements = self.get_elements(mj_elements[0], child.tag)

                                        if tag_elements:
                                            for attrib in child.attrib:
                                                for tag_element in tag_elements:
                                                    if parent_map.get(tag_element) == mj_elements[0]:
                                                        tag_element.set(attrib, child.attrib[attrib])
                                                        self.get_logger().debug("added attrib " + str(child.attrib) + " to " + str(child.tag))
                                        elif not 'name' in child.attrib:
                                            mj_elements[0].insert(0, child)
                            else:
                                self.get_logger().error("Body " + reference_name + " not found")
                                #rclpy.shutdown()
                        elif element.tag != 'compiler':
                            self.mjcf_root.insert(len(self.mjcf_root), element)

                compiler_elements = self.get_elements(self.mjcf_root, "compiler")
                if compiler_elements:
                    for element in compiler_elements:
                        out_compiler.attrib.update(element.attrib)
                        self.mjcf_root.remove(element)

                option_elements = self.get_elements(self.mjcf_root, "option")
                if option_elements:
                    for element in option_elements:
                        out_option.attrib.update(element.attrib)
                        self.mjcf_root.remove(element)
                        for child in element:
                            # Check if a similar child element exists in the second element
                            matching_child2 = next((child2 for child2 in out_option if child2.tag == child.tag), None)
                            if matching_child2 is None:
                                # If the child element doesn't exist in the second element, add it to the second element
                                out_option.append(child)

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

                ET.indent(mjcf_tree, space="\t", level=0)
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

        self.get_logger().debug(str([item for item, count in collections.Counter(out_assets).items() if count > 1]))
        # mesh_names = []
        # assets = []
        # for mesh in out_assets:
        #     if mesh.attrib['name'] is not in mesh_names:
        #         mesh_names.append(mesh.attrib['name'])
        #         assets
        #     else:
        #         out_assets.
        output_xml = ET.Element('mujoco')
        output_xml.append(out_assets)
        output_xml.append(out_compiler)
        output_xml.append(out_option)
        for filename in output_model_files:
            output_xml.append(ET.Element('include', {'file': filename}))
        if combined_qpos:
            keyframe = ET.SubElement(output_xml, "keyframe")

            ET.SubElement(
                keyframe,
                "key",
                {
                    "name": "initial",
                    "qpos": " ".join(repr(v) for v in combined_qpos)
                }
            )
        output_tree = ET.ElementTree(output_xml)
        ET.indent(output_tree, space="\t", level=0)
        output_tree.write(output_file)

        self.get_logger().debug(f"Saved mjcf xml file under {output_file}")
        self.destroy_node()
        exit(0)

    def add_composite_collisions(self, urdf_root):
        for link in self.get_elements(urdf_root, "link"):
            collisions_to_replace = []
            filenames = []
            for collision in self.get_elements(link, "collision"):
                meshes = self.get_elements(collision, "mesh")
                for mesh in meshes:
                    mesh_filename = mesh.get('filename')
                    if mesh_filename:
                        folder_path = mesh_filename[7:-4]
                        if os.path.exists(folder_path):
                            collisions_to_replace.append(collision)
                            filenames.append([os.path.join(folder_path, filename) for filename in os.listdir(folder_path)])
            for i, collision in enumerate(collisions_to_replace):
                for filename in filenames[i]:
                    self.get_logger().debug(filename)
                    collision_replacement_template = copy.deepcopy(collision)
                    mesh = self.get_elements(collision_replacement_template, "mesh")[-1]
                    mesh.attrib['filename'] = "file://" + filename
                    link.append(collision_replacement_template)
                link.remove(collision)



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

    def resolve_mesh_source_file(self, filename):
        if not filename:
            return None
        if filename[:7] == "file://":
            return filename[7:]
        if filename[:10] == "package://":
            file_name = filename[10:].split('/')
            package_path = get_package_share_directory(file_name[0])
            return os.path.join(package_path, *file_name[1:])
        return filename

    def sanitize_name(self, value):
        sanitized = "".join(char if char.isalnum() else "_" for char in value)
        sanitized = sanitized.strip("_")
        return sanitized or "part"

    def mesh_asset_stub(self, source_file):
        """Short, unique basename stem for a file generated from a mesh.

        A full absolute source path does not fit in a single filename once it is
        encoded into one - and a converted DAE encodes one twice, since the
        expanded mesh is itself the source of a symlink. Keep the readable
        basename and add a deterministic digest of the full path for uniqueness.
        """
        stem = os.path.splitext(os.path.basename(source_file))[0]
        digest = hashlib.sha256(source_file.encode("utf-8")).hexdigest()[:16]
        return f"{self.sanitize_name(stem)[:96]}_{digest}"

    def expand_dae_visuals(self, urdf_root, mujoco_files_path):
        """Replace each DAE <visual> with one STL <visual> per material group.

        MuJoCo cannot load COLLADA and MJCF carries no per-face materials, so a
        multi-material DAE only keeps its colours if it is split into one mesh
        per material. Visuals that already declare an explicit rgba, and files
        whose materials carry no colour at all, are left for the plain
        whole-file conversion in create_symlinks().
        """
        expanded_mesh_dir = os.path.join(mujoco_files_path, "dae_expanded")
        os.makedirs(expanded_mesh_dir, exist_ok=True)

        for link in self.get_elements(urdf_root, "link"):
            original_visuals = [child for child in list(link) if child.tag == "visual"]
            replacements = []

            for visual in original_visuals:
                mesh = visual.find("./geometry/mesh")
                if mesh is None:
                    continue

                source_file = self.resolve_mesh_source_file(mesh.get("filename"))
                if source_file is None or os.path.splitext(source_file)[1].lower() != ".dae":
                    continue

                explicit_color = visual.find("./material/color")
                if explicit_color is not None and explicit_color.get("rgba"):
                    continue

                try:
                    triangle_groups = extract_triangle_groups(source_file)
                except Exception as error:
                    # Leave the visual as it is: create_symlinks() attempts the
                    # plain whole-file conversion next and drops the geometry if
                    # that fails too, so the failure is reported in one place.
                    self.get_logger().warning(
                        f'could not read dae mesh {source_file}: {error}'
                    )
                    continue
                if not triangle_groups:
                    continue

                if all(group.get("rgba") is None for group in triangle_groups):
                    continue

                visual_replacements = []
                scale = mesh.get("scale")
                base_origin = visual.find("origin")
                source_stub = self.mesh_asset_stub(source_file)

                for index, group in enumerate(triangle_groups):
                    material_name = group.get("material_id") or group.get("material_symbol") or f"part_{index}"
                    triangles = group["triangles"]
                    # A single material group can exceed MuJoCo's face limit;
                    # the stock RealSense D435 DAE has one that does. Split it
                    # into several visually equivalent meshes.
                    for chunk_index, chunk_start in enumerate(
                        range(0, len(triangles), MAX_TRIANGLES_PER_MESH)
                    ):
                        triangle_chunk = triangles[
                            chunk_start:chunk_start + MAX_TRIANGLES_PER_MESH
                        ]
                        target_file = os.path.join(
                            expanded_mesh_dir,
                            f"{source_stub}__{self.sanitize_name(material_name)[:64]}"
                            f"__chunk_{chunk_index:03d}.stl",
                        )
                        if not os.path.exists(target_file):
                            header_hint = (
                                f"{os.path.basename(source_file)}:{material_name}:"
                                f"{chunk_index}"
                            )
                            write_binary_stl(
                                triangle_chunk, target_file, header_hint
                            )

                        expanded_visual = ET.Element("visual")
                        if base_origin is not None:
                            expanded_visual.append(copy.deepcopy(base_origin))

                        geometry = ET.SubElement(expanded_visual, "geometry")
                        mesh_attrib = {"filename": "file://" + target_file}
                        if scale:
                            mesh_attrib["scale"] = scale
                        ET.SubElement(geometry, "mesh", mesh_attrib)

                        rgba = group.get("rgba")
                        if rgba is not None:
                            material = ET.SubElement(
                                expanded_visual,
                                "material",
                                {"name": self.sanitize_name(material_name)},
                            )
                            rgba_str = " ".join(f"{value:.9g}" for value in rgba)
                            ET.SubElement(material, "color", {"rgba": rgba_str})

                        visual_replacements.append(expanded_visual)

                if visual_replacements:
                    replacements.append((visual, visual_replacements))

            for original_visual, visual_replacements in replacements:
                insert_index = list(link).index(original_visual)
                link.remove(original_visual)
                for offset, replacement in enumerate(visual_replacements):
                    link.insert(insert_index + offset, replacement)

    def drop_mesh_geometries(self, meshes, parents):
        """Remove the <visual>/<collision> elements owning the given meshes.

        Used when a mesh could not be converted: the geometry has to go, or the
        MJCF would reference a mesh file that does not exist. A link that ends
        up with no visual or collision at all is valid - it becomes a body
        without geoms.
        """
        for mesh in meshes:
            element = mesh
            while element is not None and element.tag not in ("visual", "collision"):
                element = parents.get(element)
            if element is None:
                continue
            parent = parents.get(element)
            if parent is None or element not in list(parent):
                continue
            self.get_logger().warning(
                f'dropping <{element.tag}> that referenced the skipped mesh'
            )
            parent.remove(element)

    def create_symlinks(self, urdf_root, mujoco_files_path):
        self.add_composite_collisions(urdf_root)
        # ElementTree keeps no parent pointers, and the failure path below needs
        # the <visual>/<collision> that owns a mesh. Record the parents before
        # anything is removed.
        parents = {child: parent for parent in urdf_root.iter() for child in parent}
        unconvertible_meshes = []
        # Create symlinks to used meshes in the tmp folder
        for mesh in self.get_elements(urdf_root, "mesh"):
            filename = mesh.get('filename')
            if filename:
                source_file = self.resolve_mesh_source_file(filename)
                if source_file is None:
                    continue
                source_extension = os.path.splitext(source_file)[1].lower()
                target_file = os.path.join(
                    mujoco_files_path,
                    "meshes",
                    f"{self.mesh_asset_stub(source_file)}{source_extension}",
                )

                if source_extension in [".stl", ".obj", ".msh"]:
                    self.get_logger().debug(f'mesh source file: {source_file}, target_file: {target_file}')
                    if not os.path.exists(target_file):
                        os.symlink(source_file, target_file)
                    mesh.attrib['filename'] = "file://" + target_file
                elif source_extension == ".dae":
                    converted_target = os.path.splitext(target_file)[0] + ".stl"
                    self.get_logger().debug(
                        f'converting dae mesh: {source_file} -> {converted_target}'
                    )
                    if not os.path.exists(converted_target):
                        try:
                            convert_dae_to_stl(source_file, converted_target)
                        except Exception as error:
                            # One unreadable mesh used to take down the whole
                            # model. Keep converting the rest and drop just the
                            # geometry that referenced it - MuJoCo would reject
                            # a <mesh> pointing at a file that was never written.
                            self.get_logger().warning(
                                f'skipping mesh {source_file}: {error}'
                            )
                            if os.path.exists(converted_target):
                                # A conversion that failed mid-write leaves a
                                # truncated STL that would look done to the next
                                # run.
                                os.remove(converted_target)
                            unconvertible_meshes.append(mesh)
                            continue
                    mesh.attrib['filename'] = "file://" + converted_target

        self.drop_mesh_geometries(unconvertible_meshes, parents)

        compiler_elements = self.get_elements(urdf_root, "compiler")
        if compiler_elements:
            compiler_element = compiler_elements[0]
            compiler_element.attrib['meshdir'] = mujoco_files_path + "/meshes"
        else:
            compiler_element = ET.Element('compiler')
            compiler_element.set("meshdir", mujoco_files_path + "/meshes")

            mujoco_elements = self.get_elements(urdf_root, "mujoco")
            if mujoco_elements:
                mujoco_element = mujoco_elements[0]
                mujoco_element.append(compiler_element)
            else:
                mujoco_element = ET.Element('mujoco')
                mujoco_element.append(compiler_element)
                urdf_root.append(mujoco_element)

    # Removes element from mjcf file and add it to the top level file
    def move_assets_to_root_xml(self, name, output_element, mjcf_tree):
        if mjcf_tree.find(name) is not None:
            for element in mjcf_tree.find(name):
                exist = False
                for e in output_element:
                    if e.attrib == element.attrib:
                        exist = True
                if not exist:
                    output_element.append(element)
            self.mjcf_root.remove(mjcf_tree.find(name))

    def convert_camera_links(self, tmp_urdf_root):
        # Merge mujoco elements
        mujoco_elements = self.get_elements(tmp_urdf_root, "mujoco")
        if mujoco_elements:
            if len(mujoco_elements) > 1:
                for mujoco_element in self.get_elements(tmp_urdf_root, "mujoco")[1:]:
                    for element in mujoco_element:
                        self.get_logger().debug(str(element))
                        mujoco_elements[0].append(element)
                    tmp_urdf_root.remove(mujoco_element)

            mujoco = mujoco_elements[0]
            camera_links = []
            reference_elements = self.get_elements(mujoco, "reference")
            if reference_elements is not None:
                for reference in reference_elements:
                    self.get_logger().debug(str(reference.get("name")))
                    if reference.find("camera") is not None:
                        camera_links.append(reference.get("name"))

            for link_element in self.get_elements(tmp_urdf_root, "link"):
                # add inertial element with small values to the links of the fixed joints
                if len(list(link_element)) == 0:
                    if link_element.attrib['name'] in camera_links:
                        inertial = ET.Element('inertial')
                        mass = ET.Element('mass')
                        mass.attrib['value'] = '1'
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

    def correct_visual_mesh(self, urdf_root):
        # Keep visual meshes intact. DAE visuals are converted to STL later in
        # expand_dae_visuals() and create_symlinks().
        return




## @brief Main function to initialize and run the Xacro2Mjcf node.
#  @param args: Command-line arguments.
def main(args=None):
    rclpy.init(args=args)

    xacro2mjcf = Xacro2Mjcf()
    xacro2mjcf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
