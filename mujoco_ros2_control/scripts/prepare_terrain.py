#!/usr/bin/env python3
"""Generate a Voronoi stepping-stones MuJoCo scene.

Writes a self-contained MJCF scene: a flat spawn platform at the origin
surrounded by irregular stones separated by gaps, over a pit floor. The layout
follows Isaac Lab's Voronoi stepping stones - Poisson-disk seed points, one
Voronoi cell per stone, shrunk to leave a gap - but nothing here depends on
Isaac Lab and the geometry is not claimed to match it cell for cell.

Run it as a CLI, or call build_scene() from a launch file; the unitree_g1
example does the latter (see the `terrain` launch argument).
"""

# @file prepare_terrain.py
# @brief Voronoi stepping-stones terrain generator for MuJoCo scenes
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
from __future__ import annotations

import argparse
import os
import struct
from dataclasses import dataclass

import numpy as np
from scipy.spatial import KDTree

# Defaults shared by the CLI and build_scene(), so both produce the same terrain.
DEFAULT_DIFFICULTY = 0.5
DEFAULT_SEED = 42
DEFAULT_FIELD_HALF = 5.0
DEFAULT_HORIZONTAL_SCALE = 0.01
DEFAULT_HFIELD_SCALE = 0.05


@dataclass
class SteppingStonesCfg:
    """Everything that determines a stepping-stones field, apart from difficulty."""

    size: tuple[float, float] = (10.0, 10.0)  # field extent (x, y), m
    horizontal_scale: float = 0.01            # sampling grid resolution, m
    vertical_scale: float = 0.005             # heightfield quantisation, m

    platform_width: float = 1.0               # flat spawn square at the origin, m
    stone_height: float = 0.04                # stone top above nominal ground, m
    holes_depth: float = -1.0                 # pit floor below nominal ground, m
    border_width: float = 0.05                # stone-free margin at the field edge, m
    seed: int | None = DEFAULT_SEED

    # Curriculum. `difficulty` (0..1) is first snapped to one of `num_levels`
    # discrete bands, then every parameter below is linearly interpolated
    # between its difficulty=0 and difficulty=1 endpoint. Give a parameter two
    # equal endpoints to hold it constant. See resolve_curriculum().
    num_levels: int = 1                       # discrete bands; 1 = continuous

    max_stone_radius: float = 0.60            # stone radius at difficulty 0, m
    min_stone_radius: float = 0.40            # stone radius at difficulty 1, m

    spacing: float = 0.8                      # Poisson centre spacing at difficulty 0, m
    min_spacing: float = 0.8                  # Poisson centre spacing at difficulty 1, m

    # Edge-to-edge clearance between neighbouring stones, and between a stone
    # and the spawn platform. This is the real walkable gap, not a bisector
    # margin: at difficulty 1 with the defaults a foot must clear 0.35 m.
    min_gap: float = 0.25                     # clearance at difficulty 0, m
    max_gap: float = 0.35                     # clearance at difficulty 1, m

    # Holes: stones dropped at random, leaving a gap straight down to the pit.
    # Scattered over the whole field rather than clustered anywhere in
    # particular. Set to 0 for no holes at all.
    max_drop_frac: float = 0.15               # fraction of stones dropped at difficulty 1

    # Stone tops are neither level with each other nor level with the ground.
    # Each stone's top is a plane, offset from z=0 by up to +-max_height_var and
    # tilted by up to max_tilt_deg about a random horizontal axis. The spawn
    # platform stays flat at z=0 regardless, so a robot's spawn height does not
    # depend on either. Set both to 0 for one flat walkable level.
    max_height_var: float = 0.10              # top offset at difficulty 1, +-m
    max_tilt_deg: float = 8.0                 # top tilt at difficulty 1, degrees


def make_cfg(**overrides) -> SteppingStonesCfg:
    """Build a cfg, accepting `size` as a scalar (square field) or an (sx, sy) pair.

    Unknown keys raise TypeError, so a typo in a launch file surfaces immediately
    instead of silently generating the default terrain.
    """
    size = overrides.get("size")
    if isinstance(size, (int, float)):
        overrides["size"] = (float(size), float(size))
    return SteppingStonesCfg(**overrides)


def _cells(length: float, scale: float) -> int:
    """Length in metres -> a whole number of grid cells.

    round(), not int(): 2.3 / 0.01 is 229.99999999999997 in binary floating
    point, and truncating it drops a cell, which shifts the field off centre by
    half a cell relative to the platform.
    """
    return int(round(length / scale))


def _snap_level(difficulty: float, num_levels: int) -> float:
    """Snap a continuous 0..1 difficulty onto one of `num_levels` evenly spaced bands."""
    d = float(np.clip(difficulty, 0.0, 1.0))
    if num_levels and num_levels > 1:
        return round(d * (num_levels - 1)) / (num_levels - 1)
    return d


def resolve_curriculum(difficulty: float, cfg: SteppingStonesCfg) -> dict:
    """Map a 0..1 difficulty onto the effective terrain parameters.

    The single source of truth for both the polygon builder and the heightfield
    painter, so the mesh and hfield colliders always describe the same terrain.
    """
    d = _snap_level(difficulty, cfg.num_levels)
    return {
        "difficulty": d,
        "gap": float(np.interp(d, [0.0, 1.0], [cfg.min_gap, cfg.max_gap])),
        "stone_radius": float(np.interp(d, [0.0, 1.0],
                                        [cfg.max_stone_radius, cfg.min_stone_radius])),
        "spacing": float(np.interp(d, [0.0, 1.0], [cfg.spacing, cfg.min_spacing])),
        "drop_frac": float(d * cfg.max_drop_frac),
        "height_var": float(d * cfg.max_height_var),
        "tilt_deg": float(d * cfg.max_tilt_deg),
    }


class TerrainError(RuntimeError):
    """Raised when the requested parameters cannot produce a usable field."""


# ─────────────────────────────────────────────────────────────────────────────
# Stone centres (Poisson-disk sampling)
# ─────────────────────────────────────────────────────────────────────────────
def stone_centers(difficulty: float, cfg: SteppingStonesCfg) -> dict:
    """Poisson-disk stone centres in metres, with the field centred on the origin.

    Bridson's algorithm: seed one point, then repeatedly try to place a new one
    in the annulus around a random active point, retiring points that no longer
    admit a neighbour. Returns dict(centers, kept, cur, platform_half), where
    `kept` indexes the stones the curriculum did not drop. Dropped stones still
    partition space - removing one leaves a hole, it does not enlarge its
    neighbours.
    """
    cur = resolve_curriculum(difficulty, cfg)
    radius = cur["stone_radius"]
    spacing = cur["spacing"]

    half_x = cfg.size[0] / 2.0
    half_y = cfg.size[1] / 2.0
    # Snapped to the sampling grid so the mesh and the painted heightfield put
    # the platform edge in the same place.
    platform_half = _cells(cfg.platform_width / 2.0, cfg.horizontal_scale) * cfg.horizontal_scale

    lo = np.array([-half_x + cfg.border_width + radius, -half_y + cfg.border_width + radius])
    hi = np.array([half_x - cfg.border_width - radius, half_y - cfg.border_width - radius])
    if np.any(lo >= hi):
        raise TerrainError(
            f"stone radius {radius:.2f} m leaves no room in a "
            f"{cfg.size[0]:.1f}x{cfg.size[1]:.1f} m field with a "
            f"{cfg.border_width:.2f} m border")

    # Keep centres far enough out that a stone cannot be born inside the spawn
    # platform. Cells can still reach over it; stone_polygons() clips that away.
    center_excl = platform_half + cur["gap"] + radius

    rng = np.random.default_rng(cfg.seed)

    def _valid(p: np.ndarray, tree: KDTree | None) -> bool:
        if not np.all((lo <= p) & (p <= hi)):
            return False
        if abs(p[0]) < center_excl and abs(p[1]) < center_excl:
            return False
        return tree is None or tree.query(p, k=1)[0] >= spacing

    centers: list[np.ndarray] = []
    active: list[np.ndarray] = []
    tree: KDTree | None = None

    for _ in range(2000):
        p0 = rng.uniform(lo, hi)
        if _valid(p0, tree):
            centers.append(p0)
            active.append(p0)
            tree = KDTree(centers)
            break
    else:
        raise TerrainError(
            "could not place a single stone in 2000 attempts; the platform "
            f"exclusion zone ({2 * center_excl:.1f} m) likely covers the whole "
            f"{cfg.size[0]:.1f}x{cfg.size[1]:.1f} m field")

    while active:
        idx = int(rng.integers(0, len(active)))
        base = active[idx]
        for _ in range(30):
            angle = rng.uniform(0.0, 2.0 * np.pi)
            dist = rng.uniform(spacing, 2.0 * spacing)
            cand = base + np.array([np.cos(angle), np.sin(angle)]) * dist
            if _valid(cand, tree):
                centers.append(cand)
                active.append(cand)
                tree = KDTree(centers)
                break
        else:
            active.pop(idx)

    centers_arr = np.array(centers)
    n = len(centers_arr)

    kept = np.arange(n)
    if cur["drop_frac"] > 0.0 and n > 1:
        n_keep = max(1, round(n * (1.0 - cur["drop_frac"])))
        if n_keep < n:
            kept = np.sort(rng.choice(n, size=n_keep, replace=False))

    # One top plane per stone: a height offset at the centre plus a tilt, drawn
    # as a magnitude and a direction so the tilt is isotropic. Drawn for every
    # stone rather than only the kept ones, so dropping stones does not reshuffle
    # the tops of the survivors.
    heights = rng.uniform(-cur["height_var"], cur["height_var"], n)
    tilt = rng.uniform(0.0, np.radians(cur["tilt_deg"]), n)
    azimuth = rng.uniform(0.0, 2.0 * np.pi, n)
    # Gradient of the top plane, i.e. rise per metre along x and y.
    slopes = np.tan(tilt)[:, None] * np.column_stack([np.cos(azimuth), np.sin(azimuth)])

    return {"centers": centers_arr, "kept": kept, "cur": cur,
            "heights": heights, "slopes": slopes,
            "platform_half": platform_half,
            "field_half": (half_x, half_y)}


def _stone_top_z(x, y, center: np.ndarray, height: float,
                 slope: np.ndarray):
    """Height of a stone's tilted top surface at (x, y).

    Elementwise, so the same plane serves the mesh (evaluated at the footprint
    vertices) and the heightfield (evaluated at every raster cell) - which is
    what keeps the two colliders describing the same terrain.
    """
    return height + slope[0] * (x - center[0]) + slope[1] * (y - center[1])


# ─────────────────────────────────────────────────────────────────────────────
# Per-stone convex footprints
#
# Each stone is its Voronoi cell - the intersection of the perpendicular
# bisectors against every other centre, which is convex by construction - inset
# on every edge so the neighbours end up `gap` apart. Convexity is what makes
# the mesh collider work: MuJoCo collides a mesh as its convex hull, so one
# prism per stone reproduces the field exactly, whereas a single mesh of all
# stones would collide as one solid block with the gaps filled in.
# ─────────────────────────────────────────────────────────────────────────────
def _clip_halfplane(poly: list[np.ndarray], u: np.ndarray, d: float) -> list[np.ndarray]:
    """Sutherland-Hodgman clip of a convex polygon by the half-plane u.x <= d."""
    out: list[np.ndarray] = []
    for k in range(len(poly)):
        a = poly[k]
        b = poly[(k + 1) % len(poly)]
        fa = float(u @ a) - d
        fb = float(u @ b) - d
        if fa <= 0.0:
            out.append(a)
        if (fa < 0.0) != (fb < 0.0):  # the edge crosses the boundary
            out.append(a + (b - a) * (fa / (fa - fb)))
    return out


def _clip_off_platform(poly: list[np.ndarray], center: np.ndarray,
                       guard: float) -> list[np.ndarray]:
    """Push a stone out of the square |x|,|y| <= `guard` around the platform.

    Only stones that actually overlap that square are touched, and each is cut
    by a single half-plane along whichever axis its centre is furthest out on.
    One cut is enough - forcing the whole footprint past one edge of the square
    already separates it from the square entirely - and one cut is all a convex
    prism can take, since subtracting a square from a polygon is not convex in
    general. Applying both axes instead would needlessly clip corners, and
    applying an axis to stones that never reach the platform carves a band
    across the whole field.
    """
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    if (min(xs) >= guard or max(xs) <= -guard
            or min(ys) >= guard or max(ys) <= -guard):
        return poly  # nowhere near the platform

    axis = 0 if abs(center[0]) >= abs(center[1]) else 1
    u = np.zeros(2)
    u[axis] = -1.0 if center[axis] > 0.0 else 1.0
    return _clip_halfplane(poly, u, -guard)


def _polygon_area(poly: np.ndarray) -> float:
    """Shoelace area of a simple polygon."""
    return 0.5 * abs(np.dot(poly[:, 0], np.roll(poly[:, 1], -1))
                     - np.dot(poly[:, 1], np.roll(poly[:, 0], -1)))


# Voronoi neighbours to clip against. A cell can only be bounded by centres
# closer than twice its own extent, and Poisson-disk sampling caps how many of
# those there can be; 32 is well past that. Checked empirically: total stone
# area is identical at 32 and at 256 neighbours (it is not at 8).
_MAX_VORONOI_NEIGHBOURS = 32


def stone_polygons(difficulty: float, cfg: SteppingStonesCfg) -> dict:
    """Per-stone convex footprints in metres, plus the platform and field extents.

    Returns dict with:
      polygons      list of (K, 2) vertex arrays, metres, origin-centred
      top_z         list of (K,) arrays, the top-surface height at each vertex
      platform_half half-extent of the flat spawn square, m
      field_half    (hx, hy) field half-extents, m
      depth         platform top (z=0) to pit floor, m
      gap           edge-to-edge clearance the stones were built for, m
    """
    seeds = stone_centers(difficulty, cfg)
    centers = seeds["centers"]
    cur = seeds["cur"]
    gap = cur["gap"]
    field_hx, field_hy = seeds["field_half"]
    platform_half = seeds["platform_half"]

    out = {"polygons": [], "top_z": [], "platform_half": platform_half,
           "field_half": (field_hx, field_hy),
           "depth": float(cfg.stone_height - cfg.holes_depth),
           "gap": gap}

    n = len(centers)
    if n == 0:
        return out

    # Insetting both sides of a shared bisector by gap/2 leaves `gap` between
    # the two stones. The platform is a fixed box rather than a shrinking cell,
    # so a stone is inset by the full `gap` from the platform edge to leave the
    # same clearance there.
    inset = gap / 2.0
    inner_x = field_hx - cfg.border_width
    inner_y = field_hy - cfg.border_width
    # Slivers this small are what is left when the gap has eaten a whole cell;
    # they would be sub-foot-sized stones that only add collision geoms.
    min_area = (cur["stone_radius"] ** 2) * 0.25

    tree = KDTree(centers)
    n_neighbours = min(n, _MAX_VORONOI_NEIGHBOURS)

    polygons: list[np.ndarray] = []
    top_z: list[np.ndarray] = []
    for i in seeds["kept"]:
        ci = centers[i]
        poly = [np.array([-inner_x, -inner_y]), np.array([inner_x, -inner_y]),
                np.array([inner_x, inner_y]), np.array([-inner_x, inner_y])]

        for j in np.atleast_1d(tree.query(ci, k=n_neighbours)[1]):
            if j == i:
                continue
            diff = centers[j] - ci
            dist = float(np.hypot(*diff))
            if dist < 1e-9:
                continue
            u = diff / dist
            poly = _clip_halfplane(poly, u, float(u @ ci) + dist / 2.0 - inset)
            if len(poly) < 3:
                break

        # Keep the stones out of the spawn platform, but only the ones that
        # actually reach it: the platform is a finite square, so its edge lines
        # must never be applied as infinite half-planes. Clipping every stone
        # whose centre is merely past a platform edge shaves a band off the
        # whole field and leaves a cross-shaped void through it.
        if len(poly) >= 3:
            poly = _clip_off_platform(poly, ci, platform_half + gap)

        if len(poly) < 3:
            continue
        vertices = np.array(poly)
        if _polygon_area(vertices) < min_area:
            continue
        polygons.append(vertices)
        # The top face is planar, so the prism stays convex however it is
        # tilted - which is what lets one mesh geom per stone stay exact.
        top_z.append(_stone_top_z(vertices[:, 0], vertices[:, 1], ci,
                                  seeds["heights"][i], seeds["slopes"][i]))

    out["polygons"] = polygons
    out["top_z"] = top_z
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Heightfield raster (the alternative single-geom collider)
# ─────────────────────────────────────────────────────────────────────────────
def stone_heightfield(difficulty: float, cfg: SteppingStonesCfg) -> np.ndarray:
    """Paint the same field into a [nx, ny] height raster in metres.

    A pixel belongs to a stone when it is at least gap/2 closer to its own
    centre than to the next one - the distance form of the same inset the
    polygons use - and takes its height from that stone's top plane, so tilted
    and offset tops survive the rasterisation. Heights are quantised to
    `vertical_scale`, and are relative to a platform level of `stone_height`
    (see generate_hfield_scene_xml, which maps that level onto world z=0).
    """
    seeds = stone_centers(difficulty, cfg)
    centers = seeds["centers"]
    cur = seeds["cur"]
    hs = cfg.horizontal_scale

    nx = _cells(cfg.size[0], hs)
    ny = _cells(cfg.size[1], hs)
    quant = cfg.vertical_scale
    height = np.full((nx, ny), round(cfg.holes_depth / quant) * quant)

    platform_half = seeds["platform_half"]
    stone_top = round(cfg.stone_height / quant) * quant

    # Cell centres in metres, matching the origin-centred polygon frame.
    px = (np.arange(nx) + 0.5) * hs - cfg.size[0] / 2.0
    py = (np.arange(ny) + 0.5) * hs - cfg.size[1] / 2.0
    gx, gy = np.meshgrid(px, py, indexing="ij")

    if len(centers) > 0:
        pixels = np.column_stack([gx.ravel(), gy.ravel()])
        dists, indices = KDTree(centers).query(pixels, k=min(2, len(centers)))
        if len(centers) == 1:
            d_own = dists.reshape(-1)
            d_other = np.full_like(d_own, np.inf)
            owner = np.zeros(d_own.shape, dtype=np.int64)
        else:
            d_own, d_other = dists[:, 0], dists[:, 1]
            owner = indices[:, 0]

        on_stone = (d_other - d_own) >= cur["gap"]
        if len(seeds["kept"]) < len(centers):
            kept_mask = np.zeros(len(centers), dtype=bool)
            kept_mask[seeds["kept"]] = True
            on_stone &= kept_mask[owner]

        on_stone = on_stone.reshape(nx, ny)
        on_stone &= ~((np.abs(gx) <= platform_half + cur["gap"])
                      & (np.abs(gy) <= platform_half + cur["gap"]))
        on_stone &= ((np.abs(gx) <= cfg.size[0] / 2.0 - cfg.border_width)
                     & (np.abs(gy) <= cfg.size[1] / 2.0 - cfg.border_width))

        # Each cell takes the height of its own stone's top plane.
        own = owner.reshape(nx, ny)
        tops = stone_top + _stone_top_z(
            gx, gy, centers[own].transpose(2, 0, 1),
            seeds["heights"][own], seeds["slopes"][own].transpose(2, 0, 1))
        height[on_stone] = (np.round(tops / quant) * quant)[on_stone]

    height[(np.abs(gx) <= platform_half) & (np.abs(gy) <= platform_half)] = stone_top
    return height


def save_hfield(path: str, height_m: np.ndarray) -> None:
    """Write a height raster in MuJoCo's native heightfield format.

    Two int32 (nrow, ncol) followed by nrow*ncol float32, row-major, rows along
    +y and columns along +x - so the [x, y] raster is transposed on the way out.
    Used in preference to a PNG because it needs no image library at all, and
    because MuJoCo takes the float data verbatim instead of through an 8- or
    16-bit quantisation. MuJoCo renormalises the values to [0, 1] on load and
    scales them by the elevation given in the <hfield> size attribute.
    """
    data = np.ascontiguousarray(height_m.T, dtype="<f4")
    with open(path, "wb") as hfield:
        hfield.write(struct.pack("<ii", data.shape[0], data.shape[1]))
        hfield.write(data.tobytes())


# ─────────────────────────────────────────────────────────────────────────────
# MuJoCo scene XML
#
# Convention: the walkable surface (stone tops and platform) is at z=0 and the
# pit floor sits at z=-depth, so a robot's spawn height is the same as it would
# be on flat ground.
#
# The scene is written to stand in for mjcf/scene.xml, which xacro2mjcf.py
# symlinks into the MuJoCo working directory and <include>s. Pass one or the
# other, never both: they define the same skybox, groundplane material and
# floor geom, and MuJoCo rejects repeated names.
# ─────────────────────────────────────────────────────────────────────────────
_SCENE_HEADER = """\
<mujoco model="stepping stones scene">
    <statistic center="0 0 0.5" extent="4"/>

    <visual>
        <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
        <rgba haze="0.15 0.25 0.35 1"/>
        <global azimuth="120" elevation="-20"/>
    </visual>

    <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
                 width="512" height="3072"/>
        <texture type="2d" name="groundplane" builtin="checker" mark="edge"
                 rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
                 width="300" height="300"/>
        <material name="groundplane" texture="groundplane" texuniform="true"
                  texrepeat="8 8" reflectance="0.2"/>
        <material name="stone" rgba="0.55 0.57 0.6 1" reflectance="0.1"/>
"""

_WORLDBODY_HEADER = """\
    </asset>

    <worldbody>
        <light pos="0 0 3" dir="0 0 -1" directional="true"/>
"""


def _prism_vertices(poly_xy: np.ndarray, top_z: np.ndarray, z_bot: float) -> str:
    """The `vertex` attribute of an inline mesh.

    The footprint twice: once at the (possibly tilted) top plane, giving one
    height per vertex, and once flat at the pit floor so every stone is a solid
    column rather than a floating slab.
    """
    top = "  ".join(f"{x:.4f} {y:.4f} {z:.4f}"
                    for (x, y), z in zip(poly_xy, top_z))
    bottom = "  ".join(f"{x:.4f} {y:.4f} {z_bot:.4f}" for x, y in poly_xy)
    return top + "  " + bottom


def generate_mesh_scene_xml(polygons: list[np.ndarray], top_z: list[np.ndarray],
                            platform_half: float,
                            field_half: tuple[float, float], depth: float) -> str:
    """A scene whose stones are one inline convex mesh each.

    The meshes are inline `<mesh vertex="...">` rather than files, so the scene
    stays a single self-contained XML. That also sidesteps the meshdir
    xacro2mjcf.py sets on the merged model, which a relative file reference
    would be resolved against.
    """
    field_hx, field_hy = field_half
    z_bot = -depth
    parts = [_SCENE_HEADER]

    for k, (poly, tops) in enumerate(zip(polygons, top_z)):
        parts.append(f'        <mesh name="stone_{k}" '
                     f'vertex="{_prism_vertices(poly, tops, z_bot)}"/>\n')

    parts.append(_WORLDBODY_HEADER)
    # A missed step lands on the pit floor. MuJoCo planes are infinite for
    # collision; the size only sets how far the texture is drawn.
    parts.append(f'        <geom name="pit_floor" type="plane" material="groundplane"\n'
                 f'              size="{field_hx:.3f} {field_hy:.3f} 0.1" '
                 f'pos="0 0 {z_bot:.4f}"/>\n')
    parts.append(f'        <geom name="platform" type="box" material="stone"\n'
                 f'              size="{platform_half:.4f} {platform_half:.4f} '
                 f'{depth / 2.0:.4f}"\n'
                 f'              pos="0 0 {-depth / 2.0:.4f}"/>\n')
    for k in range(len(polygons)):
        parts.append(f'        <geom name="stone_{k}" type="mesh" mesh="stone_{k}" '
                     f'material="stone"/>\n')

    parts.append("    </worldbody>\n</mujoco>\n")
    return "".join(parts)


def generate_hfield_scene_xml(hfield_file: str, height_m: np.ndarray,
                              field_half: tuple[float, float],
                              zero_level: float) -> str:
    """A scene with one heightfield collider instead of per-stone meshes.

    One geom instead of hundreds, at the cost of crisp geometry: MuJoCo
    interpolates between samples, so every vertical stone wall becomes a slope
    one cell wide. nrow/ncol are read from the file, so they are not repeated
    here.

    MuJoCo renormalises the samples to [0, 1] and scales them by the elevation
    in `size`, so setting that elevation to the raster's own span makes one
    sample unit one metre again. The geom is then lowered so `zero_level` - the
    raster value of the flat spawn platform - lands on world z=0. Anchoring the
    platform rather than the highest sample is what keeps the spawn height
    independent of how tall the tallest stone happens to be.
    """
    field_hx, field_hy = field_half
    low = float(height_m.min())
    span = max(float(height_m.max()) - low, 1e-6)
    parts = [_SCENE_HEADER]
    parts.append(f'        <hfield name="terrain" file="{hfield_file}"\n'
                 f'                size="{field_hx:.4f} {field_hy:.4f} '
                 f'{span:.4f} 0.1"/>\n')
    parts.append(_WORLDBODY_HEADER)
    parts.append('        <geom name="terrain" type="hfield" hfield="terrain"\n'
                 f'              material="groundplane" '
                 f'pos="0 0 {low - zero_level:.4f}"/>\n')
    parts.append("    </worldbody>\n</mujoco>\n")
    return "".join(parts)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point used by the launch files
# ─────────────────────────────────────────────────────────────────────────────
def build_scene(
    out_dir: str,
    difficulty: float = DEFAULT_DIFFICULTY,
    seed: int | None = DEFAULT_SEED,
    field_half: float = DEFAULT_FIELD_HALF,
    horizontal_scale: float = DEFAULT_HORIZONTAL_SCALE,
    collider: str = "mesh",
    hfield_scale: float = DEFAULT_HFIELD_SCALE,
    scene_name: str = "terrain_scene.xml",
    **cfg_overrides,
) -> dict:
    """Write a stepping-stones scene into `out_dir` and describe what was written.

    `difficulty` (0..1) is the only difficulty knob; one call is one terrain.
    `field_half` gives a 2*field_half square field. `collider` picks "mesh" (one
    convex prism per stone, crisp vertical walls) or "hfield" (a single
    heightfield geom, sloped walls), rendered at the coarser `hfield_scale`
    because a full-resolution raster makes for a needlessly large MuJoCo
    heightfield. Remaining SteppingStonesCfg fields pass through as keywords.

    Returns dict(scene, collider, difficulty, n_stones, gap, depth, height_var,
    tilt_deg, drop_frac, files).
    """
    if collider not in ("mesh", "hfield"):
        raise ValueError(f"collider must be 'mesh' or 'hfield', got {collider!r}")

    os.makedirs(out_dir, exist_ok=True)
    scene_path = os.path.join(out_dir, scene_name)
    size = (2.0 * field_half, 2.0 * field_half)
    written = [scene_path]

    if collider == "hfield":
        cfg = make_cfg(size=size, horizontal_scale=hfield_scale, seed=seed,
                       **cfg_overrides)
        height_m = stone_heightfield(difficulty, cfg)
        hfield_path = os.path.join(out_dir, os.path.splitext(scene_name)[0] + ".hfield")
        save_hfield(hfield_path, height_m)
        written.append(hfield_path)
        # Absolute, because xacro2mjcf.py points the merged model's meshdir at
        # its own working directory before including this scene.
        xml = generate_hfield_scene_xml(os.path.abspath(hfield_path), height_m,
                                        (field_half, field_half),
                                        zero_level=cfg.stone_height)
        n_stones = 0
        geo = {"gap": resolve_curriculum(difficulty, cfg)["gap"],
               "depth": float(cfg.stone_height - cfg.holes_depth)}
    else:
        cfg = make_cfg(size=size, horizontal_scale=horizontal_scale, seed=seed,
                       **cfg_overrides)
        geo = stone_polygons(difficulty, cfg)
        xml = generate_mesh_scene_xml(geo["polygons"], geo["top_z"],
                                      geo["platform_half"],
                                      geo["field_half"], geo["depth"])
        n_stones = len(geo["polygons"])

    with open(scene_path, "w") as scene:
        scene.write(xml)

    cur = resolve_curriculum(difficulty, cfg)
    return {"scene": scene_path, "collider": collider,
            "difficulty": cur["difficulty"],
            "n_stones": n_stones, "gap": geo["gap"], "depth": geo["depth"],
            "height_var": cur["height_var"], "tilt_deg": cur["tilt_deg"],
            "drop_frac": cur["drop_frac"], "files": written}


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", default=".",
                        help="directory to write the scene into")
    parser.add_argument("--scene-name", default="terrain_scene.xml",
                        help="scene filename inside --out-dir")
    parser.add_argument("--difficulty", type=float, default=DEFAULT_DIFFICULTY,
                        help="0 (large, close stones) to 1 (small, far apart)")
    parser.add_argument("--num-levels", type=int, default=1,
                        help="snap the difficulty into N discrete bands (1 = continuous)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="RNG seed; the same seed always yields the same field")
    parser.add_argument("--field-half", type=float, default=DEFAULT_FIELD_HALF,
                        help="half-extent of the square field, m")
    parser.add_argument("--horizontal-scale", type=float,
                        default=DEFAULT_HORIZONTAL_SCALE,
                        help="sampling grid resolution, m")
    parser.add_argument("--collider", choices=["mesh", "hfield"], default="mesh",
                        help="one convex prism per stone, or a single heightfield geom")
    parser.add_argument("--hfield-scale", type=float, default=DEFAULT_HFIELD_SCALE,
                        help="heightfield raster resolution, m (--collider hfield)")
    parser.add_argument("--max-height-var", type=float,
                        default=SteppingStonesCfg.max_height_var,
                        help="stone top offset at difficulty 1, +-m (0 = all level)")
    parser.add_argument("--max-tilt-deg", type=float,
                        default=SteppingStonesCfg.max_tilt_deg,
                        help="stone top tilt at difficulty 1, degrees (0 = all flat)")
    parser.add_argument("--max-drop-frac", type=float,
                        default=SteppingStonesCfg.max_drop_frac,
                        help="fraction of stones dropped at difficulty 1, leaving "
                             "holes through to the pit (0 = no holes)")
    args = parser.parse_args()

    result = build_scene(
        out_dir=args.out_dir,
        scene_name=args.scene_name,
        difficulty=args.difficulty,
        seed=args.seed,
        field_half=args.field_half,
        horizontal_scale=args.horizontal_scale,
        collider=args.collider,
        hfield_scale=args.hfield_scale,
        num_levels=args.num_levels,
        max_height_var=args.max_height_var,
        max_tilt_deg=args.max_tilt_deg,
        max_drop_frac=args.max_drop_frac,
    )

    print(f"difficulty {args.difficulty:.2f} -> level {result['difficulty']:.2f} "
          f"of {args.num_levels}, seed {args.seed}, "
          f"{2 * args.field_half:.1f}x{2 * args.field_half:.1f} m field")
    print(f"  {result['gap']:.2f} m gaps, {result['depth']:.2f} m drop, "
          f"tops +-{result['height_var']:.2f} m and tilted up to "
          f"{result['tilt_deg']:.1f} deg, {result['drop_frac']:.0%} holes, "
          f"{result['collider']} collider"
          + (f", {result['n_stones']} stones" if result["collider"] == "mesh" else ""))
    for path in result["files"]:
        print(f"  wrote {path}")


if __name__ == "__main__":
    main()
