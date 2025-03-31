from urdf_parser_py import urdf
import os
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
## @file xacro2mjcf.py
# @brief Script to convert a urdf to a mjcf
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


def build_joint_link_tree(robot):
    # Dictionary to store the joint-link relationships
    tree = {}

    # Iterate over all joints in the URDF
    for joint in robot.joints:
        parent_link = joint.parent
        child_link = joint.child

        # Create the entry for the joint's parent-child relationship
        if parent_link not in tree:
            tree[parent_link] = {"joints": [], "children": []}
        if child_link not in tree:
            tree[child_link] = {"joints": [], "children": []}

        # Add the joint to the parent link's list
        tree[parent_link]["joints"].append(joint.name)
        tree[parent_link]["children"].append(child_link)

        # Recursively add the child link to the tree
        if child_link not in tree:
            tree[child_link] = {"joints": [], "children": []}
        
    return tree

def create_mjcf(robot, robot_tree, mujoco_element):
    # Create the root element of the MJCF
    mjcf = ET.Element("mujoco", model=robot.name)
    compiler = mujoco_element.find('compiler')
    compiler.set("angle", "radian")
    mjcf.append(compiler)

    # Create a worldbody element
    worldbody = ET.SubElement(mjcf, "worldbody")
    asset = ET.SubElement(mjcf, "asset")
    # Create the root link, world is typically the root in MJCF
    #world_link = ET.SubElement(worldbody, "body", name="world")
    
    # Recursive function to create bodies and joints
    def add_body_and_joints(link_name, parent_body, robot, asset):
        if link_name not in robot_tree:
            return
        if link_name == "world":
            body = parent_body
            joint_type = "fixed"
        else:
            body = ET.SubElement(parent_body, "body", name=link_name)
            joint_type = robot.joint_map[robot.parent_map[link_name][0]].joint_type
            # Create joints for this link
            joint = robot.joint_map[robot.parent_map[link_name][0]]
            joint_type = joint.joint_type
            axis = joint.axis
            if joint_type == "revolute":
                joint_elem = ET.SubElement(body, "joint", name=joint.name, type="hinge", axis=f"{axis[0]} {axis[1]} {axis[2]}")
                if joint.limit:
                    joint_elem.set("range", f"{joint.limit.lower} {joint.limit.upper}")
                    joint_elem.set("limited", "true")
                if joint.dynamics:
                    joint_elem.set("damping", str(joint.dynamics.damping))
                    joint_elem.set("frictionloss", str(joint.dynamics.friction))
            elif joint_type == "continuous":
                joint_elem = ET.SubElement(body, "joint", name=joint.name, type="hinge", axis=f"{axis[0]} {axis[1]} {axis[2]}")
                joint_elem.set("limited", "false")
                if joint.dynamics:
                    joint_elem.set("damping", str(joint.dynamics.damping))
                    joint_elem.set("frictionloss", str(joint.dynamics.friction))
            elif joint_type == "prismatic":
                joint_elem = ET.SubElement(body, "joint", name=joint.name, type="slide", axis=f"{axis[0]} {axis[1]} {axis[2]}")
            elif joint_type == "floating":
                joint_elem = ET.SubElement(body, "joint", name=joint.name, type="free")
            else:
                print(joint_type)
        # Add the link's body
        if link_name in robot.parent_map:
            if robot.joint_map[robot.parent_map[link_name][0]].origin:
                parent_joint = robot.joint_map[robot.parent_map[link_name][0]]
                body.set("pos", f"{parent_joint.origin.position[0]} {parent_joint.origin.position[1]} {parent_joint.origin.position[2]}")
                if not all(element == 0 for element in parent_joint.origin.rpy):
                    quat = R.from_euler('xyz', parent_joint.origin.rpy).as_quat()
                    norm = np.linalg.norm(quat)
                    quat = quat / norm
                    body.set("quat", f"{quat[3]} {quat[0]} {quat[1]} {quat[2]}")
        if robot.link_map[link_name].inertial:
            inertial = ET.SubElement(body, "inertial")
            #if robot.link_map[link_name].inertial.mass > 0:
            inertial.set("mass", str(robot.link_map[link_name].inertial.mass))
            origin = robot.link_map[link_name].inertial.origin
            #if sum(origin.position) > 0:
            inertial.set("pos", f"{origin.position[0]} {origin.position[1]} {origin.position[2]}")
            if not all(element == 0 for element in origin.rpy):
                quat = R.from_euler('xyz', origin.rpy).as_quat()
                norm = np.linalg.norm(quat)
                quat = quat / norm
                inertial.set("quat", f"{quat[3]} {quat[0]} {quat[1]} {quat[2]}")
            inertia = robot.link_map[link_name].inertial.inertia
            # Construct inertia matrix
            inertia_matrix = np.array([
                [inertia.ixx, inertia.ixy, inertia.ixz],
                [inertia.ixy, inertia.iyy, inertia.iyz],
                [inertia.ixz, inertia.iyz, inertia.izz]
            ])
            # Compute eigenvalues and eigenvectors
            eigenvalues, _ = np.linalg.eigh(inertia_matrix)
            inertial.set("diaginertia", f"{eigenvalues[2]} {eigenvalues[1]} {eigenvalues[0]}")
        if robot.link_map[link_name].collisions:
            for i, collision in enumerate(robot.link_map[link_name].collisions):
                geom = ET.SubElement(body, "geom", group="1", name=f"{link_name}_collision_{i}")
                origin = collision.origin
                if origin:
                    geom.set("pos", f"{origin.position[0]} {origin.position[1]} {origin.position[2]}")
                    if not all(element == 0 for element in origin.rpy):
                        quat = R.from_euler('xyz', origin.rpy).as_quat()
                        norm = np.linalg.norm(quat)
                        quat = quat / norm
                        geom.set("quat", f"{quat[3]} {quat[0]} {quat[1]} {quat[2]}")
                if type(collision.geometry) == urdf.Mesh:
                    filename = collision.geometry.filename
                    mesh = ET.SubElement(asset, "mesh", name=f"{link_name}_collision_mesh{i}", file=filename.split("/")[-1])
                    if collision.geometry.scale:
                        mesh.set("scale", f"{collision.geometry.scale[0]} {collision.geometry.scale[1]} {collision.geometry.scale[2]}")
                    geom.set("type", "mesh")
                    geom.set("mesh", f"{link_name}_collision_mesh{i}")
                elif type(collision.geometry) == urdf.Box:
                    size = collision.geometry.size
                    geom.set("type", "box")
                    geom.set("size", f"{size[0]/2} {size[1]/2} {size[2]/2}")
                elif type(collision.geometry) == urdf.Sphere:
                    radius = collision.geometry.radius
                    geom.set("type", "sphere")
                    geom.set("size", f"{radius}")
                elif type(collision.geometry) == urdf.Cylinder:
                    radius = collision.geometry.radius
                    length = collision.geometry.length
                    geom.set("type", "sphere")
                    geom.set("size", f"{radius} {length/2}")
                else:
                    print(type(collision.geometry))
                    
        if robot.link_map[link_name].visual:
            for i, visual in enumerate(robot.link_map[link_name].visuals):
                geom = ET.SubElement(body, "geom", group="0", contype="0", conaffinity="0", name=f"{link_name}_visual_{i}")
                origin = visual.origin
                if origin:
                    geom.set("pos", f"{origin.position[0]} {origin.position[1]} {origin.position[2]}")
                    if not all(element == 0 for element in origin.rpy):
                        quat = R.from_euler('xyz', origin.rpy).as_quat()
                        norm = np.linalg.norm(quat)
                        quat = quat / norm
                        geom.set("quat", f"{quat[3]} {quat[0]} {quat[1]} {quat[2]}")
                if type(visual.geometry) == urdf.Mesh:
                    filename = visual.geometry.filename
                    mesh = ET.SubElement(asset, "mesh", name=f"{link_name}_visual_mesh{i}", file=filename.split("/")[-1])
                    if visual.geometry.scale:
                        mesh.set("scale", f"{visual.geometry.scale[0]} {visual.geometry.scale[1]} {visual.geometry.scale[2]}")

                    geom.set("type", "mesh")
                    geom.set("mesh", f"{link_name}_visual_mesh{i}")
                elif type(visual.geometry) == urdf.Box:
                    size = visual.geometry.size
                    geom.set("type", "box")
                    geom.set("size", f"{size[0]/2} {size[1]/2} {size[2]/2}")
                else:
                    print(type(visual.geometry))
        
        
        # Recursively add child links and their joints
        for child in robot_tree[link_name]["children"]:
            add_body_and_joints(child, body, robot, asset)
    
    # Start with the root link 'world' and recursively add bodies and joints
    add_body_and_joints("world", worldbody, robot, asset)


    seen = set()  # To track unique elements
    for child in list(asset):  # Use list to avoid issues when modifying the tree
        # Create a tuple to uniquely identify an element (tag, sorted attributes)
        identifier = (child.tag, tuple(sorted(child.attrib.items())))
        if identifier in seen:
            asset.remove(child)  # Remove duplicate
        else:
            seen.add(identifier)
    
    # Convert the tree to an XML string
    ET.indent(mjcf, space="\t", level=0)
    return ET.tostring(mjcf, encoding="unicode", method="xml")

def create_mjcf_from_urdf(input_file, output_file):
    with open(input_file, 'r') as file:
        robot_description = file.read()

    # Load URDF
    robot = urdf.Robot.from_xml_string(robot_description)

    urdf_tree = ET.ElementTree(ET.fromstring(robot_description))
    urdf_root = urdf_tree.getroot()
    mujoco_element = urdf_root.find('mujoco')
    # Convert the URDF-like structure to MJCF
    tree = build_joint_link_tree(robot)
    mjcf_string = create_mjcf(robot, tree, mujoco_element)

    # Print the resulting MJCF model
    with open(output_file, 'w') as file:
        file.write(mjcf_string)

if __name__ == "__main__":
    file_path = "/tmp/mujoco/tmp_tofas_disc.urdf"
    create_mjcf_from_urdf(file_path, "/tmp/mujoco/test.xml")
