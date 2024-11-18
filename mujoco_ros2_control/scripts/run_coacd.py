#!/usr/bin/env python3
# Adapted from https://github.com/SarahWeiii/CoACD/blob/main/python/package/bin/coacd

try:
    import trimesh
except ModuleNotFoundError:
    print("trimesh is required. Please install with `pip install trimesh`")
    exit(1)

import sys
import os
import argparse
import coacd

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

    if args.quiet:
        coacd.set_log_level("error")

    mesh = trimesh.load(args.input, force="mesh")
    mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    result = coacd.run_coacd(
        mesh,
        threshold=args.threshold,
        max_convex_hull=args.max_convex_hull,
        preprocess_mode=args.preprocess_mode,
        preprocess_resolution=args.prep_resolution,
        resolution=args.resolution,
        mcts_nodes=args.mcts_node,
        mcts_iterations=args.mcts_iteration,
        mcts_max_depth=args.mcts_max_depth,
        pca=args.pca,
        merge=not args.no_merge,
        seed=args.seed,
    )

    base_filename = os.path.splitext(os.path.basename(args.input))[0]
    folder_path = os.path.join(os.path.dirname(args.input), base_filename)
    if not os.path.exists(folder_path):
        os.mkdir(folder_path)
        print(f"Folder '{base_filename}' created successfully.")
    else:
        print(f"Folder '{base_filename}' already exists.")
    i = 0
    for vs, fs in result:
        mesh_part = trimesh.Trimesh(vs, fs)
        mesh_part.export(folder_path + "/" + base_filename + "_" + str(i) + "." + args.output)
        i += 1
