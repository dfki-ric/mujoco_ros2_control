#!/usr/bin/env python3
"""Convert a binary STL to a Wavefront OBJ, losslessly.

MuJoCo's STL decoder rejects any file with more than 200000 faces, which is a
limit of that decoder and not of the model: the same geometry loads through the
OBJ path. Converting is the better answer whenever the mesh has been measured
against something a decimation would no longer match - a visual origin, a box
collision stand-in, a grasp pose. The mujoco_ros2_control_examples build does
this for Franka's 295768-face printable D435 camera mount.

Only the triangle soup is carried over. STL has no normals worth keeping (the
per-facet normal is recomputed by every consumer) and no texture coordinates, so
the OBJ holds welded vertices and faces alone.
"""

# @file stl_to_obj.py
# @brief Lossless binary STL to Wavefront OBJ converter
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

import argparse
import os
import struct
import sys

# 80-byte header, uint32 triangle count, then 50 bytes per triangle:
# 3 floats of facet normal, 3x3 floats of vertices, uint16 attribute count.
HEADER_SIZE = 80
COUNT_SIZE = 4
TRIANGLE_SIZE = 50
MAX_TRIANGLES = 20_000_000  # ~1 GB of STL; beyond this the input is not a mesh.


def read_binary_stl(path):
    """Return (triangle_count, [(x, y, z), ...]) with 3 vertices per triangle."""
    with open(path, "rb") as stl:
        blob = stl.read()

    if len(blob) < HEADER_SIZE + COUNT_SIZE:
        raise ValueError(f"{path} is too short to be a binary STL")

    # An ASCII STL starts with "solid", but so do some binary ones (the exporter
    # is free to write anything in the header), so trust the length instead.
    count = struct.unpack_from("<I", blob, HEADER_SIZE)[0]
    expected = HEADER_SIZE + COUNT_SIZE + count * TRIANGLE_SIZE
    if count > MAX_TRIANGLES or len(blob) != expected:
        raise ValueError(
            f"{path} is not a binary STL: its triangle count ({count}) implies "
            f"{expected} bytes but the file is {len(blob)}"
        )

    # Skip each facet normal and trailing attribute count; keep the 9 floats.
    unpack = struct.Struct("<12x9f2x").iter_unpack
    body = memoryview(blob)[HEADER_SIZE + COUNT_SIZE:]
    vertices = []
    for triangle in unpack(body):
        vertices.append(triangle[0:3])
        vertices.append(triangle[3:6])
        vertices.append(triangle[6:9])
    return count, vertices


def write_obj(path, source, vertices):
    """Weld duplicate vertices and write one OBJ face per input triangle."""
    index_of = {}
    unique = []
    indices = []
    for vertex in vertices:
        index = index_of.get(vertex)
        if index is None:
            unique.append(vertex)
            index = len(unique)  # OBJ indices are 1-based.
            index_of[vertex] = index
        indices.append(index)

    with open(path, "w") as obj:
        obj.write(f"# converted from {os.path.basename(source)} by stl_to_obj.py\n")
        obj.writelines(f"v {x:.6g} {y:.6g} {z:.6g}\n" for x, y, z in unique)
        obj.writelines(
            f"f {indices[i]} {indices[i + 1]} {indices[i + 2]}\n"
            for i in range(0, len(indices), 3)
        )
    return len(unique)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stl", help="input binary STL")
    parser.add_argument("obj", help="output OBJ")
    args = parser.parse_args()

    try:
        count, vertices = read_binary_stl(args.stl)
    except (OSError, ValueError) as error:
        print(f"stl_to_obj: {error}", file=sys.stderr)
        return 1

    unique = write_obj(args.obj, args.stl, vertices)
    print(f"stl_to_obj: {args.stl}: {count} faces, {unique} vertices -> {args.obj}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
