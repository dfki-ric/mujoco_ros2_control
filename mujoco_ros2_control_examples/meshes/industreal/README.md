# NVIDIA IndustReal assets

The peg, pick-tray, and insertion-tray OBJ meshes are not stored here. They are
downloaded into `pegs/` at configure time from `isaac-sim/IsaacGymEnvs` commit
`aeed298638a1f7b5421b38f5f3cc2d1079b6d9c3`, which serves them as plain files, and
are the post-processed simulator assets used by the IndustReal peg task. See
`DOWNLOAD_INDUSTREAL_PEG_ASSETS` in `../../CMakeLists.txt`.

The `gears/` meshes are derived from `NVlabs/industrealkit` CAD and cannot be
re-fetched: the decimated visual meshes and their convex decompositions were
generated locally with Phobos and CoACD from the upstream printable parts.
IndustRealKit tracks every mesh in Git LFS and `raw.githubusercontent.com`
serves the pointer file rather than the mesh, so they could not be downloaded at
build time even if they were unmodified.

See `licenses/` for the upstream NVIDIA and IsaacGymEnvs license texts. In
particular, the IndustRealKit license restricts these assets and derivatives to
non-commercial research or evaluation use.
