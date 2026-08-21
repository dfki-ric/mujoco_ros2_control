# NVIDIA IndustReal assets

The peg, pick-tray, and insertion-tray OBJ meshes are not stored here. They are
downloaded into `pegs/` at configure time from `isaac-sim/IsaacGymEnvs` commit
`aeed298638a1f7b5421b38f5f3cc2d1079b6d9c3`, which serves them as plain files, and
are the post-processed simulator assets used by the IndustReal peg task. See
`DOWNLOAD_INDUSTREAL_PEG_ASSETS` in `../../CMakeLists.txt`.

The electrical connector fixture meshes are copied from `NVlabs/industrealkit`
commit `0c91cb5f7fb391e0cc8515fcdf669aab7e9ffe4b`. The physical plugs and sockets
are commercial components and are not included by upstream, so this package
intentionally includes only their printable trays. These are kept in-tree rather
than downloaded: IndustRealKit tracks every mesh in Git LFS, and
`raw.githubusercontent.com` serves the pointer file rather than the mesh, so a
`file(DOWNLOAD)` fetch would need `git-lfs` and would pull the repository's full
LFS payload to extract these few files.

The `gears/` meshes are derived assets and cannot be re-fetched: the decimated
visual meshes and their convex decompositions were generated locally with Phobos
and CoACD from the upstream printable CAD.

See `licenses/` for the upstream NVIDIA and IsaacGymEnvs license texts. In
particular, the IndustRealKit license restricts these assets and derivatives to
non-commercial research or evaluation use.
