# Franka assets

## RealSense D435 wrist mount

`realsense_d435/RealSenseD435_Camera_Mount.obj` is **not** stored here. It is
downloaded, extracted and converted at configure time from

    https://download.franka.de/camera_mount_guide.zip

by `DOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET` in `../../CMakeLists.txt`, so the mesh
exists only in the install tree.

It is the mount that Franka Robotics documents in *3D Printable Camera Mount
Guide*, document `R02241` (revision 1.1, January 2026). The archive holds four
files - an STL and a STEP for both the D435 mount and a generic 1/4"-20 mount -
and only the D435 STL is used. Its internal CAD name is
`Panda_RealSenseD435_Camera_Mount`, exported from SolidWorks on 2019-07-17. The
part measures 48 x 60 x 49.95 mm and the STL carries 295768 triangles; the
visual origin in `../../urdf/franka/realsense_d435_wrist.urdf.xacro` and the box
collision that stands in for it are measured against exactly that geometry.

### Why it is converted to OBJ

MuJoCo's STL decoder refuses any file with more than 200000 faces:

    stl_decoder: number of faces should be between 1 and 200000

That is a limit of the decoder, not of the model - the identical triangles load
through MuJoCo's OBJ path and compile in well under a second. So
`../../scripts/stl_to_obj.py` welds the STL's triangle soup into an OBJ (295768
faces, 147878 vertices) instead of decimating the part, which would move the
surface the visual origin was measured against. The conversion is lossless:
STL carries no texture coordinates, and its per-facet normals are recomputed by
every consumer anyway.

The download URL is unversioned, so it cannot be pinned to a revision the way
the IndustReal peg meshes are pinned to an upstream commit. Instead CMake
records the archive's SHA256

    0b8c53aadee5978132b1a7f2e2ef3a127b2a6e5321a1da31f27dede87b637913

and warns - rather than fails - when it no longer matches, since a revised mount
would move the camera without any other symptom.

### Why it is not redistributed

The archive ships no licence file, and the guide carries only
`(c) Copyright 2025 Franka Robotics GmbH` with no grant to copy, modify or
redistribute. A public download is not a redistribution licence, and a decimated
re-export would still be a derivative work, so the mesh is fetched by each user
from Franka directly instead of being vendored. If Franka grants written
redistribution permission, this can become an in-tree asset with that permission
recorded alongside it.
