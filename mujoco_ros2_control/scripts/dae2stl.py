import collections
import math
import os
import struct
import xml.etree.ElementTree as ET
## @file dae2stl.py
# @brief Minimal COLLADA (.dae) reader and binary STL writer
#
# MuJoCo cannot load COLLADA, so every .dae mesh referenced by a URDF has to be
# converted before the MJCF is compiled. This module does that conversion and
# nothing else: it knows about COLLADA and STL, but not about URDF, ROS or
# MuJoCo. xacro2mjcf.py imports it the same way it imports urdf2mjcf.py.
#
# @license BSD 3-Clause License
# @copyright Copyright (c) 2026, DFKI GmbH
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

COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}

## MuJoCo rejects a mesh with more than 200,000 faces. Callers split larger
#  material groups at this size, which keeps a little headroom.
MAX_TRIANGLES_PER_MESH = 190000


## @brief Unit normal of a triangle, or a zero vector for a degenerate facet.
def facet_normal(v0, v1, v2):
    ux = v1[0] - v0[0]
    uy = v1[1] - v0[1]
    uz = v1[2] - v0[2]
    vx = v2[0] - v0[0]
    vy = v2[1] - v0[1]
    vz = v2[2] - v0[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (nx / norm, ny / norm, nz / norm)


## @brief Write a list of triangles as a binary STL file.
#  @param triangles: list of triangles, each a sequence of three xyz vertices.
#  @param target_file: path of the STL file to create.
#  @param header_hint: text placed in the 80-byte STL header, for traceability.
def write_binary_stl(triangles, target_file, header_hint):
    with open(target_file, "wb") as stl_file:
        header = header_hint.encode("ascii", errors="ignore")
        stl_file.write(header[:80].ljust(80, b"\0"))
        stl_file.write(struct.pack("<I", len(triangles)))

        for triangle in triangles:
            normal = facet_normal(*triangle)
            stl_file.write(struct.pack("<3f", *normal))
            for vertex in triangle:
                stl_file.write(struct.pack("<3f", *vertex))
            stl_file.write(struct.pack("<H", 0))


def _identity_matrix():
    return [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]


def _multiply_matrices(lhs, rhs):
    return [
        sum(lhs[row * 4 + k] * rhs[k * 4 + col] for k in range(4))
        for row in range(4)
        for col in range(4)
    ]


def _transform_point(matrix, point):
    x, y, z = point
    return [
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    ]


def _node_matrix(node):
    matrix = node.find("c:matrix", COLLADA_NS)
    if matrix is None or not matrix.text:
        return _identity_matrix()

    values = [float(value) for value in matrix.text.split()]
    if len(values) != 16:
        return _identity_matrix()

    # COLLADA exports used by Franka put scale/rotation/translation in
    # row-major matrices. Applying these is required for meter-scale visuals.
    return values


## @brief Collect every instantiation of each geometry in the visual scene.
#  @return dict of geometry id -> list of {"matrix", "material_targets"}.
def _geometry_instances(dae_root):
    instances = collections.defaultdict(list)

    def collect(node, parent_matrix):
        local_matrix = _node_matrix(node)
        node_matrix = _multiply_matrices(parent_matrix, local_matrix)

        for instance_geometry in node.findall("c:instance_geometry", COLLADA_NS):
            url = instance_geometry.get("url", "")
            if not url.startswith("#"):
                continue

            material_targets = {}
            for instance_material in instance_geometry.findall(
                ".//c:instance_material", COLLADA_NS
            ):
                symbol = instance_material.get("symbol")
                target = instance_material.get("target", "")
                if not symbol:
                    continue
                if target.startswith("#"):
                    target = target[1:]
                material_targets[symbol] = target

            instances[url[1:]].append({
                "matrix": node_matrix,
                "material_targets": material_targets,
            })

        for child in node.findall("c:node", COLLADA_NS):
            collect(child, node_matrix)

    for scene in dae_root.findall(".//c:visual_scene", COLLADA_NS):
        for node in scene.findall("c:node", COLLADA_NS):
            collect(node, _identity_matrix())

    return instances


## @brief Read a COLLADA file and group its triangles per material.
#
#  Node transforms are composed down the visual scene, so unit scaling and node
#  placement are baked into the returned vertices. The diffuse (or ambient)
#  colour is resolved through instance_material -> material -> effect.
#
#  @param source_file: path of the .dae file to read.
#  @return list of {"material_symbol", "material_id", "rgba", "triangles"}
#          for every material group that contains at least one triangle.
def extract_triangle_groups(source_file):
    dae_tree = ET.parse(source_file)
    dae_root = dae_tree.getroot()

    effect_colors = {}
    for effect in dae_root.findall(".//c:effect", COLLADA_NS):
        effect_id = effect.get("id")
        color = effect.find(".//c:diffuse/c:color", COLLADA_NS)
        if color is None:
            color = effect.find(".//c:ambient/c:color", COLLADA_NS)
        if effect_id and color is not None and color.text:
            rgba = [float(value) for value in color.text.split()]
            if len(rgba) == 3:
                rgba.append(1.0)
            if len(rgba) == 4:
                effect_colors[effect_id] = tuple(rgba)

    material_colors = {}
    for material in dae_root.findall(".//c:material", COLLADA_NS):
        material_id = material.get("id")
        instance_effect = material.find("c:instance_effect", COLLADA_NS)
        if not material_id or instance_effect is None:
            continue
        effect_url = instance_effect.get("url", "")
        if effect_url.startswith("#"):
            effect_url = effect_url[1:]
        if effect_url in effect_colors:
            material_colors[material_id] = effect_colors[effect_url]

    default_material_targets = {}
    for instance_material in dae_root.findall(".//c:instance_material", COLLADA_NS):
        symbol = instance_material.get("symbol")
        target = instance_material.get("target", "")
        if not symbol:
            continue
        if target.startswith("#"):
            target = target[1:]
        default_material_targets[symbol] = target

    geometry_instances = _geometry_instances(dae_root)
    triangle_groups = collections.OrderedDict()
    for geometry in dae_root.findall(".//c:geometry", COLLADA_NS):
        geometry_id = geometry.get("id")
        mesh = geometry.find("c:mesh", COLLADA_NS)
        if mesh is None:
            continue

        sources = {}
        for source in mesh.findall("c:source", COLLADA_NS):
            float_array = source.find("c:float_array", COLLADA_NS)
            if float_array is None or not float_array.text:
                continue

            accessor = source.find("c:technique_common/c:accessor", COLLADA_NS)
            stride = 3
            if accessor is not None:
                stride = int(accessor.get("stride", "3"))

            sources[source.get("id")] = (
                [float(value) for value in float_array.text.split()],
                stride,
            )

        vertices_map = {}
        for vertices in mesh.findall("c:vertices", COLLADA_NS):
            position_input = vertices.find("c:input[@semantic='POSITION']", COLLADA_NS)
            if position_input is not None:
                vertices_map[vertices.get("id")] = position_input.get("source", "")[1:]

        for triangles_element in mesh.findall("c:triangles", COLLADA_NS):
            vertex_offset = None
            position_source_id = None
            max_offset = 0

            for input_element in triangles_element.findall("c:input", COLLADA_NS):
                offset = int(input_element.get("offset", "0"))
                max_offset = max(max_offset, offset)
                if input_element.get("semantic") == "VERTEX":
                    vertex_offset = offset
                    position_source_id = vertices_map.get(input_element.get("source", "")[1:])

            if vertex_offset is None or position_source_id not in sources:
                continue

            position_values, stride = sources[position_source_id]
            point_list = triangles_element.find("c:p", COLLADA_NS)
            if point_list is None or not point_list.text:
                continue

            indices = [int(value) for value in point_list.text.split()]
            step = max_offset + 1

            instances = geometry_instances.get(
                geometry_id,
                [{"matrix": _identity_matrix(), "material_targets": {}}],
            )
            for instance in instances:
                material_symbol = triangles_element.get("material", "default")
                material_id = instance["material_targets"].get(
                    material_symbol,
                    default_material_targets.get(material_symbol, material_symbol),
                )
                group_key = material_id or material_symbol
                if group_key not in triangle_groups:
                    triangle_groups[group_key] = {
                        "material_symbol": material_symbol,
                        "material_id": material_id,
                        "rgba": material_colors.get(material_id),
                        "triangles": [],
                    }

                for index in range(0, len(indices), step * 3):
                    face_vertices = []
                    for vertex_index in range(3):
                        position_index = indices[index + vertex_index * step + vertex_offset]
                        start = position_index * stride
                        vertex = position_values[start:start + 3]
                        if len(vertex) == 3:
                            vertex = _transform_point(instance["matrix"], vertex)
                        face_vertices.append(vertex)
                    if all(len(vertex) == 3 for vertex in face_vertices):
                        triangle_groups[group_key]["triangles"].append(face_vertices)

    return [group for group in triangle_groups.values() if group["triangles"]]


## @brief Convert a COLLADA file into a single binary STL, merging all materials.
#  @param source_file: path of the .dae file to read.
#  @param target_file: path of the STL file to create.
def convert_dae_to_stl(source_file, target_file):
    triangles = []
    for group in extract_triangle_groups(source_file):
        triangles.extend(group["triangles"])

    if not triangles:
        raise RuntimeError(f"No triangle mesh data found in {source_file}")
    write_binary_stl(
        triangles,
        target_file,
        f"Converted from {os.path.basename(source_file)}",
    )
