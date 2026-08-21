# NVIDIA IndustReal assets

## What is here, and under which licence

`gears/` is the only IndustReal asset stored in this repository, and it is **not
covered by the package licence**. It is a derivative work of the printable gear
CAD in [`NVlabs/industrealkit`](https://github.com/NVlabs/industrealkit) and is
distributed under the NVIDIA License in `licenses/INDUSTREALKIT_LICENSE.txt`,
whose section 3.3 limits the work and any derivative of it to **non-commercial
use - research or evaluation purposes only**.

That limitation is named here deliberately. Section 3.2 of the NVIDIA License
permits a derivative work to carry different terms only if those terms keep the
section 3.3 use limitation and if the specific derivative works subject to them
are identified. The identified works are every file under `gears/`: the
decimated visual meshes and the convex-decomposition pieces beside them, which
were generated locally with [Phobos](https://github.com/dfki-ric/phobos) and
[CoACD](https://github.com/SarahWeiii/CoACD) from the upstream parts. They
cannot be re-fetched at build time: IndustRealKit tracks every mesh in Git LFS
and `raw.githubusercontent.com` serves the pointer file rather than the mesh, so
they would not be downloadable even unmodified.

Anyone who needs the rest of the package under its own licence, without the
non-commercial restriction attached, should build with the gears disabled - a
`task_board_config` whose `gears.enabled` is `false`.

## What is downloaded instead

The peg, pick-tray, and insertion-tray OBJ meshes are **not** stored here. They
are downloaded into `pegs/` at configure time from `isaac-sim/IsaacGymEnvs`
commit `aeed298638a1f7b5421b38f5f3cc2d1079b6d9c3`, which serves them as plain
files, and are the post-processed simulator assets used by the IndustReal peg
task. See `DOWNLOAD_INDUSTREAL_PEG_ASSETS` in `../../CMakeLists.txt`.

Because this repository never redistributes those files, it carries no copy of
the IsaacGymEnvs licence: BSD-3-Clause requires the notice in redistributions,
and a configure-time download from NVIDIA is not one. IsaacGymEnvs is
BSD-3-Clause, Copyright (c) 2018-2023 NVIDIA Corporation. If you redistribute a
**built** install tree, you are redistributing those meshes and the notice
becomes your obligation, so fetch it from upstream and ship it alongside.

The IndustReal electrical-connector task is not modelled at all. Its plugs and
sockets are commercial parts that NVIDIA identifies by part number without
redistributing geometry, so only the printable trays were ever public and
nothing existed for the parts that actually mate.
