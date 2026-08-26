#!/usr/bin/env python3
# Adapted from https://github.com/SarahWeiii/CoACD/blob/main/python/package/bin/coacd

try:
    import trimesh
except ModuleNotFoundError:
    print("trimesh is required. Install it and coacd with `uv pip install --system coacd trimesh`")
    exit(1)

import hashlib
import json
import os
import shutil
import argparse
import coacd


def _settings_path(folder_path):
    # A sibling of the output folder, not a file inside it:
    # add_composite_collisions() in xacro2mjcf.py treats every file in that
    # folder as a collision mesh piece with no extension filtering, so a
    # settings file living inside it would get fed into MuJoCo as one.
    return folder_path + ".coacd_settings.json"


def _ongoing_path(folder_path):
    # Same reasoning as _settings_path(): a sibling, not a file inside it.
    return folder_path + ".coacd_ongoing"


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def decompose_mesh(
        input_path, output_ext="obj", threshold=0.05, preprocess_mode="auto",
        resolution=2000, merge=True, max_convex_hull=-1, mcts_iteration=150,
        mcts_max_depth=3, mcts_node=20, prep_resolution=50, pca=False, seed=0,
        quiet=False):
    """Convex-decompose one mesh with CoaCD.

    Writes the pieces into a folder beside the input, named after it with the
    extension stripped (mujoco_ros2_control_examples/xacro2mjcf.py's
    add_composite_collisions() picks this convention up automatically), plus
    a settings file next to that folder (see _settings_path()) recording the
    input mesh's hash and every argument above. A folder whose settings file
    matches is left alone -- CoaCD is too slow to re-run unconditionally on
    every call -- and one whose settings file does not match (the mesh or an
    argument changed since) is rebuilt.

    A folder with no settings file at all is left alone too, untouched and
    unregenerated: that is a decomposition this function did not produce --
    meshes/industreal/gears/ ships ones generated offline with Phobos, not
    this function -- and it is not this function's place to judge or discard
    it.

    While a mesh is being worked on, a "<name>.coacd_ongoing" marker sits
    beside it (see _ongoing_path()) -- `ls` the output directory during a
    slow multi-mesh batch to see which one is currently running. It is
    removed once that mesh finishes; a leftover one means the process that
    made it never got that far, which is deliberate -- see the note below.
    """
    if not os.path.isfile(input_path):
        raise FileNotFoundError(input_path)

    settings = {
        "input_sha256": _sha256(input_path),
        "output_ext": output_ext,
        "threshold": threshold,
        "preprocess_mode": preprocess_mode,
        "resolution": resolution,
        "merge": merge,
        "max_convex_hull": max_convex_hull,
        "mcts_iteration": mcts_iteration,
        "mcts_max_depth": mcts_max_depth,
        "mcts_node": mcts_node,
        "prep_resolution": prep_resolution,
        "pca": pca,
        "seed": seed,
    }

    base_filename = os.path.splitext(os.path.basename(input_path))[0]
    folder_path = os.path.join(os.path.dirname(input_path), base_filename)
    settings_path = _settings_path(folder_path)
    if os.path.isdir(folder_path):
        if not os.path.isfile(settings_path):
            return folder_path
        try:
            with open(settings_path) as f:
                cached_settings = json.load(f)
        except (OSError, ValueError):
            cached_settings = None
        if cached_settings == settings:
            return folder_path
        shutil.rmtree(folder_path)
        os.remove(settings_path)

    ongoing_path = _ongoing_path(folder_path)
    with open(ongoing_path, "w"):
        pass

    if quiet:
        coacd.set_log_level("error")

    mesh = trimesh.load(input_path, force="mesh")
    mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    result = coacd.run_coacd(
        mesh,
        threshold=threshold,
        max_convex_hull=max_convex_hull,
        preprocess_mode=preprocess_mode,
        preprocess_resolution=prep_resolution,
        resolution=resolution,
        mcts_nodes=mcts_node,
        mcts_iterations=mcts_iteration,
        mcts_max_depth=mcts_max_depth,
        pca=pca,
        merge=merge,
        seed=seed,
    )

    # Built under a temporary name and moved into place with one atomic
    # rename, done *last*, so an interrupted run (crash, Ctrl+C) never leaves
    # folder_path holding a partial set of pieces: until the rename, the
    # folder this function looks for does not exist yet, so an interrupted
    # attempt is indistinguishable from never having tried, not from having
    # finished. The settings file goes to its final path first, and would
    # only ever be found beside a not-yet-complete folder if the process died
    # in the instant between these two renames.
    tmp_folder_path = folder_path + ".tmp"
    if os.path.isdir(tmp_folder_path):
        shutil.rmtree(tmp_folder_path)
    os.mkdir(tmp_folder_path)
    for i, (vs, fs) in enumerate(result):
        mesh_part = trimesh.Trimesh(vs, fs)
        mesh_part.export(os.path.join(tmp_folder_path, f"{base_filename}_{i}.{output_ext}"))

    tmp_settings_path = settings_path + ".tmp"
    with open(tmp_settings_path, "w") as f:
        json.dump(settings, f)
    os.rename(tmp_settings_path, settings_path)
    os.rename(tmp_folder_path, folder_path)
    # Only removed once every step above has actually succeeded -- not in a
    # finally block, which would erase the one trace of an interrupted run
    # this function leaves behind.
    os.remove(ongoing_path)
    return folder_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        required=True,
        help="input model loaded by trimesh. Supported formats: glb, gltf, obj, off, ply, stl, etc.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="obj",
        help="output model exported by trimesh. Supported formats: glb, gltf, obj, off, ply, stl, etc.",
    )
    parser.add_argument("--quiet", action="store_true", help="do not print logs")
    parser.add_argument(
        "-t",
        "--threshold",
        type=float,
        default=0.05,
        help="termination criteria in [0.01, 1] (0.01: most fine-grained; 1: most coarse)",
    )
    parser.add_argument(
        "-pm",
        "--preprocess-mode",
        type=str,
        default="auto",
        help="No remeshing before running CoACD. Only suitable for manifold input.",
    )
    parser.add_argument(
        "-r",
        "--resolution",
        type=int,
        default=2000,
        help="surface sampling resolution for Hausdorff distance computation",
    )
    parser.add_argument(
        "-nm",
        "--no-merge",
        action="store_true",
        help="If merge is enabled, try to reduce total number of parts by merging.",
    )
    parser.add_argument(
        "-c",
        "--max-convex-hull",
        type=int,
        default=-1,
        help="max # convex hulls in the result, -1 for no limit, works only when merge is enabled",
    )
    parser.add_argument(
        "-mi",
        "--mcts_iteration",
        type=int,
        default=150,
        help="Number of MCTS iterations.",
    )
    parser.add_argument(
        "-md",
        "--mcts-max-depth",
        type=int,
        default=3,
        help="Maximum depth for MCTS search.",
    )
    parser.add_argument(
        "-mn",
        "--mcts-node",
        type=int,
        default=20,
        help="Number of cut candidates for MCTS.",
    )
    parser.add_argument(
        "-pr",
        "--prep-resolution",
        type=int,
        default=50,
        help="Preprocessing resolution.",
    )
    parser.add_argument(
        "--pca",
        action="store_true",
        help="Use PCA to align input mesh. Suitable for non-axis-aligned mesh.",
    )
    parser.add_argument("--seed", type=int, default=0, help="Random seed.")

    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(args.input, "is not a file")
        exit(1)

    folder_path = decompose_mesh(
        args.input,
        output_ext=args.output,
        threshold=args.threshold,
        preprocess_mode=args.preprocess_mode,
        resolution=args.resolution,
        merge=not args.no_merge,
        max_convex_hull=args.max_convex_hull,
        mcts_iteration=args.mcts_iteration,
        mcts_max_depth=args.mcts_max_depth,
        mcts_node=args.mcts_node,
        prep_resolution=args.prep_resolution,
        pca=args.pca,
        seed=args.seed,
        quiet=args.quiet,
    )
    print(f"Folder '{os.path.basename(folder_path)}' ready.")
