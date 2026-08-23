#!/bin/env python3
"""No-GL unit tests for scripts/prepare_terrain.py.

These never start MuJoCo - they only need numpy and scipy - so they run in CI
where no simulator is available. What they cover is the geometry MuJoCo would
otherwise have to be trusted to reveal: that neighbouring stones really are
`gap` apart, that none of them reaches into the spawn platform, that the spawn
platform lands on z=0 in both colliders however the stone tops vary, and that a
seed reproduces a field exactly. The unitree_g1 launch test covers actually loading a scene.
"""
import itertools
import os
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

# prepare_terrain lives in scripts/, a sibling of this test/ directory. Taken
# from the source tree rather than the install space, so editing the script and
# rerunning pytest directly cannot silently test a stale installed copy.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import prepare_terrain as pt  # noqa: E402

DIFFICULTIES = (0.0, 0.5, 1.0)


def _covered(polygons, pts):
    """Boolean mask of which of `pts` fall inside any of the convex `polygons`."""
    inside = np.zeros(len(pts), dtype=bool)
    for poly in polygons:
        edges = np.roll(poly, -1, axis=0) - poly
        # Cross product of each edge with the vector to the point; a convex
        # polygon contains the point when every sign agrees.
        rel = pts[:, None, :] - poly[None, :, :]
        cross = edges[None, :, 0] * rel[:, :, 1] - edges[None, :, 1] * rel[:, :, 0]
        inside |= np.all(cross >= -1e-12, axis=1) | np.all(cross <= 1e-12, axis=1)
    return inside


def _band_coverage_ratio(geo, axis, field_half=4.9, samples=260):
    """Stone coverage inside the band through the platform, over that outside it.

    The band is the strip of the field that shares the platform's extent on
    `axis` but lies well away from the platform along the other axis. It should
    be paved about as densely as the rest of the field; a platform edge applied
    as an infinite half-plane empties it instead.
    """
    guard = geo["platform_half"] + geo["gap"]
    grid = np.linspace(-field_half, field_half, samples)
    gx, gy = np.meshgrid(grid, grid, indexing="ij")
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    inside = _covered(geo["polygons"], pts)

    on_axis, off_axis = (pts[:, 0], pts[:, 1]) if axis == 0 else (pts[:, 1], pts[:, 0])
    far = np.abs(off_axis) > 2.0
    band = (np.abs(on_axis) < guard) & far
    rest = (np.abs(on_axis) >= guard) & far
    return inside[band].mean() / inside[rest].mean()


def _min_distance(poly_a, poly_b, samples=24):
    """Smallest distance between two convex polygons, by edge sampling.

    Exact enough for an assertion with a millimetre tolerance: sampling an edge
    at `samples` points can only ever over-estimate the distance, so a passing
    test is never a false pass.
    """
    best = np.inf
    for first, second in ((poly_a, poly_b), (poly_b, poly_a)):
        for i in range(len(first)):
            a, b = first[i], first[(i + 1) % len(first)]
            pts = a + (b - a) * np.linspace(0.0, 1.0, samples)[:, None]
            for j in range(len(second)):
                c, d = second[j], second[(j + 1) % len(second)]
                qts = c + (d - c) * np.linspace(0.0, 1.0, samples)[:, None]
                best = min(best, np.linalg.norm(pts[:, None] - qts[None], axis=-1).min())
    return best


class TestCurriculum(unittest.TestCase):
    def test_endpoints(self):
        cfg = pt.make_cfg()
        easy = pt.resolve_curriculum(0.0, cfg)
        hard = pt.resolve_curriculum(1.0, cfg)
        self.assertAlmostEqual(easy["gap"], cfg.min_gap)
        self.assertAlmostEqual(hard["gap"], cfg.max_gap)
        self.assertAlmostEqual(easy["stone_radius"], cfg.max_stone_radius)
        self.assertAlmostEqual(hard["stone_radius"], cfg.min_stone_radius)
        # Harder must mean smaller stones further apart, or the knob is inverted.
        self.assertGreater(hard["gap"], easy["gap"])
        self.assertLess(hard["stone_radius"], easy["stone_radius"])
        # Uneven tops scale in from nothing, so difficulty 0 stays perfectly flat.
        self.assertEqual(easy["height_var"], 0.0)
        self.assertEqual(easy["tilt_deg"], 0.0)
        self.assertAlmostEqual(hard["height_var"], cfg.max_height_var)
        self.assertAlmostEqual(hard["tilt_deg"], cfg.max_tilt_deg)

    def test_difficulty_is_clamped(self):
        cfg = pt.make_cfg()
        self.assertEqual(pt.resolve_curriculum(-1.0, cfg)["gap"], cfg.min_gap)
        self.assertEqual(pt.resolve_curriculum(7.0, cfg)["gap"], cfg.max_gap)

    def test_levels_snap_to_bands(self):
        cfg = pt.make_cfg(num_levels=5)
        bands = {pt.resolve_curriculum(d, cfg)["difficulty"]
                 for d in np.linspace(0.0, 1.0, 101)}
        self.assertEqual(sorted(bands), [0.0, 0.25, 0.5, 0.75, 1.0])

    def test_unknown_cfg_key_raises(self):
        # A launch-file typo must fail loudly, not silently use the defaults.
        with self.assertRaises(TypeError):
            pt.make_cfg(stone_radius=0.3)

    def test_scalar_size_makes_a_square(self):
        self.assertEqual(pt.make_cfg(size=8.0).size, (8.0, 8.0))

    def test_cell_count_does_not_truncate(self):
        # 2.3 / 0.01 is 229.99999999999997 in binary floating point; truncating
        # it drops a cell and shifts the field half a cell off centre.
        self.assertEqual(pt._cells(2.3, 0.01), 230)
        self.assertEqual(pt._cells(0.3, 0.05), 6)
        self.assertEqual(pt._cells(10.0, 0.01), 1000)


class TestStoneGeometry(unittest.TestCase):
    def test_field_is_populated(self):
        for difficulty in DIFFICULTIES:
            with self.subTest(difficulty=difficulty):
                geo = pt.stone_polygons(difficulty, pt.make_cfg())
                self.assertGreater(len(geo["polygons"]), 20)

    def test_polygons_are_convex_and_ccw_or_cw(self):
        geo = pt.stone_polygons(0.5, pt.make_cfg())
        for poly in geo["polygons"]:
            self.assertGreaterEqual(len(poly), 3)
            # A convex polygon's cross products all share one sign.
            edges = np.roll(poly, -1, axis=0) - poly
            cross = np.cross(edges, np.roll(edges, -1, axis=0))
            self.assertTrue(np.all(cross >= -1e-9) or np.all(cross <= 1e-9))

    def test_stones_are_a_gap_apart(self):
        """The whole point of the terrain: neighbours must not touch."""
        for difficulty in DIFFICULTIES:
            with self.subTest(difficulty=difficulty):
                geo = pt.stone_polygons(difficulty, pt.make_cfg())
                polys, gap = geo["polygons"], geo["gap"]
                centers = np.array([p.mean(axis=0) for p in polys])
                worst = np.inf
                for i, j in itertools.combinations(range(len(polys)), 2):
                    # Only near pairs can be the closest pair.
                    if np.linalg.norm(centers[i] - centers[j]) > 3.0:
                        continue
                    worst = min(worst, _min_distance(polys[i], polys[j]))
                self.assertGreaterEqual(worst, gap - 1e-3,
                                        f"stones {worst:.4f} m apart, want >= {gap:.4f} m")

    def test_stones_clear_the_spawn_platform(self):
        """A stone reaching over the platform would make the first step free."""
        for difficulty in DIFFICULTIES:
            with self.subTest(difficulty=difficulty):
                geo = pt.stone_polygons(difficulty, pt.make_cfg())
                half, gap = geo["platform_half"], geo["gap"]
                # Chebyshev distance from the platform square to each vertex.
                clearance = min(max(abs(v[0]) - half, abs(v[1]) - half)
                                for poly in geo["polygons"] for v in poly)
                self.assertGreaterEqual(clearance, gap - 1e-3,
                                        f"stone {clearance:.4f} m from the platform, "
                                        f"want >= {gap:.4f} m")

    def test_no_cross_shaped_void_through_the_field(self):
        """The platform must not carve a band across the whole field.

        Its edges bound a finite square; used as infinite half-planes they clip
        every stone whose centre is merely past one of them, leaving a
        cross-shaped hole through the middle of the terrain. Measured on the
        regression this guards: the band was paved to 0.57 of the surrounding
        density, against 0.95 once the clip is confined to stones that actually
        reach the platform.
        """
        # Holes off, so a dropped stone cannot be mistaken for the void.
        geo = pt.stone_polygons(1.0, pt.make_cfg(max_drop_frac=0.0))
        for axis, name in ((0, "x"), (1, "y")):
            ratio = _band_coverage_ratio(geo, axis)
            self.assertGreater(ratio, 0.80,
                               f"the |{name}| band through the platform is only "
                               f"{ratio:.2f} as densely paved as the rest of the field")

    def test_stones_near_the_platform_are_still_pushed_off_it(self):
        """Confining the platform clip must not stop it doing its job."""
        geo = pt.stone_polygons(1.0, pt.make_cfg(max_drop_frac=0.0))
        guard = geo["platform_half"] + geo["gap"]
        # No stone may cover the interior of the guard square. Probed strictly
        # inside it: a correctly clipped stone lies exactly on the boundary,
        # which _covered counts as contact.
        grid = np.linspace(-guard, guard, 41) * (1.0 - 1e-6)
        gx, gy = np.meshgrid(grid, grid, indexing="ij")
        pts = np.column_stack([gx.ravel(), gy.ravel()])
        self.assertFalse(_covered(geo["polygons"], pts).any(),
                         "a stone reaches into the platform's clearance square")

    def test_stones_stay_inside_the_border(self):
        cfg = pt.make_cfg()
        geo = pt.stone_polygons(0.5, cfg)
        limit_x = geo["field_half"][0] - cfg.border_width + 1e-6
        limit_y = geo["field_half"][1] - cfg.border_width + 1e-6
        for poly in geo["polygons"]:
            self.assertLessEqual(np.abs(poly[:, 0]).max(), limit_x)
            self.assertLessEqual(np.abs(poly[:, 1]).max(), limit_y)

    def test_seed_is_reproducible(self):
        first = pt.stone_polygons(0.5, pt.make_cfg(seed=7))["polygons"]
        again = pt.stone_polygons(0.5, pt.make_cfg(seed=7))["polygons"]
        self.assertEqual(len(first), len(again))
        for a, b in zip(first, again):
            np.testing.assert_allclose(a, b)

    def test_seed_changes_the_field(self):
        a = pt.stone_polygons(0.5, pt.make_cfg(seed=1))["polygons"]
        b = pt.stone_polygons(0.5, pt.make_cfg(seed=2))["polygons"]
        self.assertFalse(len(a) == len(b) and all(
            np.allclose(x, y) for x, y in zip(a, b)))

    def test_holes_can_be_turned_off_entirely(self):
        solid = pt.stone_polygons(1.0, pt.make_cfg(max_drop_frac=0.0))["polygons"]
        holey = pt.stone_polygons(1.0, pt.make_cfg(max_drop_frac=0.3))["polygons"]
        self.assertGreater(len(solid), len(holey))

    def test_hole_count_scales_with_the_knob(self):
        counts = [len(pt.stone_polygons(1.0, pt.make_cfg(max_drop_frac=f))["polygons"])
                  for f in (0.0, 0.15, 0.4)]
        self.assertEqual(counts, sorted(counts, reverse=True), counts)

    def test_holes_are_scattered_over_the_field(self):
        """Holes must be spread out, not lined up along the axes."""
        cfg = pt.make_cfg(max_drop_frac=0.3)
        kept = pt.stone_centers(1.0, cfg)
        centers, keep = kept["centers"], set(kept["kept"].tolist())
        dropped = np.array([c for i, c in enumerate(centers) if i not in keep])
        self.assertGreater(len(dropped), 4)
        # Every quadrant should lose at least one stone.
        quadrants = {(x > 0, y > 0) for x, y in dropped}
        self.assertEqual(len(quadrants), 4, f"holes only in {len(quadrants)} quadrants")

    def test_difficulty_zero_drops_nothing(self):
        cfg = pt.make_cfg(max_drop_frac=0.5)
        seeds = pt.stone_centers(0.0, cfg)
        self.assertEqual(len(seeds["kept"]), len(seeds["centers"]))

    def test_impossible_field_raises(self):
        # A radius that cannot fit must say so rather than return an empty field.
        with self.assertRaises(pt.TerrainError):
            pt.stone_polygons(0.0, pt.make_cfg(size=1.0, max_stone_radius=2.0,
                                               min_stone_radius=2.0))


class TestHeightfield(unittest.TestCase):
    def test_raster_shape(self):
        cfg = pt.make_cfg(size=10.0, horizontal_scale=0.05)
        self.assertEqual(pt.stone_heightfield(0.5, cfg).shape, (200, 200))

    def test_level_terrain_raster_has_two_levels(self):
        # With the uneven-top knobs off there is only stone and pit.
        cfg = pt.make_cfg(size=10.0, horizontal_scale=0.05,
                          max_height_var=0.0, max_tilt_deg=0.0)
        height = pt.stone_heightfield(1.0, cfg)
        self.assertEqual(sorted(np.unique(height).tolist()),
                         [cfg.holes_depth, cfg.stone_height])

    def test_uneven_raster_stays_within_the_curriculum_bounds(self):
        cfg = pt.make_cfg(size=10.0, horizontal_scale=0.05)
        cur = pt.resolve_curriculum(1.0, cfg)
        height = pt.stone_heightfield(1.0, cfg)
        stones = height[height > cfg.holes_depth] - cfg.stone_height
        self.assertGreater(len(np.unique(stones)), 10, "tops did not vary")
        # A top can be as far off level as its centre offset plus its own tilt
        # across the stone's radius.
        limit = cur["height_var"] + np.tan(np.radians(cur["tilt_deg"])) * (
            cur["stone_radius"] * 2.0) + cfg.vertical_scale
        self.assertLessEqual(np.abs(stones).max(), limit)

    def test_platform_is_flat_and_high(self):
        cfg = pt.make_cfg(size=10.0, horizontal_scale=0.05)
        height = pt.stone_heightfield(0.5, cfg)
        centre = height.shape[0] // 2
        # The platform stays flat and level whatever the stones do around it -
        # the spawn height depends on it.
        block = height[centre - 5:centre + 5, centre - 5:centre + 5]
        np.testing.assert_allclose(block, cfg.stone_height)

    def test_raster_and_polygons_agree_on_coverage(self):
        """Both colliders must describe the same terrain, within one raster cell."""
        cfg_mesh = pt.make_cfg(size=10.0, horizontal_scale=0.01)
        cfg_grid = pt.make_cfg(size=10.0, horizontal_scale=0.05)
        geo = pt.stone_polygons(0.5, cfg_mesh)
        height = pt.stone_heightfield(0.5, cfg_grid)

        raster_frac = float((height > cfg_grid.holes_depth).mean())
        area = sum(pt._polygon_area(p) for p in geo["polygons"])
        area += (2.0 * geo["platform_half"]) ** 2
        poly_frac = area / (cfg_mesh.size[0] * cfg_mesh.size[1])
        # The raster quantises every stone edge to a 5 cm cell, so a few percent
        # of disagreement is the discretisation, not a different terrain.
        self.assertAlmostEqual(raster_frac, poly_frac, delta=0.05)

    def test_hfield_file_roundtrips(self):
        height = pt.stone_heightfield(0.5, pt.make_cfg(size=4.0, horizontal_scale=0.1))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "t.hfield")
            pt.save_hfield(path, height)
            with open(path, "rb") as handle:
                nrow, ncol = struct.unpack("<ii", handle.read(8))
                data = np.frombuffer(handle.read(), dtype="<f4")
        # MuJoCo reads rows along +y and columns along +x, so the [x, y] raster
        # is stored transposed.
        self.assertEqual((nrow, ncol), (height.shape[1], height.shape[0]))
        self.assertEqual(data.size, nrow * ncol)
        np.testing.assert_allclose(data.reshape(nrow, ncol), height.T, atol=1e-6)


class TestSceneXml(unittest.TestCase):
    def _build(self, **kwargs):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        result = pt.build_scene(out_dir=tmp.name, **kwargs)
        return result, ET.parse(result["scene"]).getroot()

    def test_mesh_scene_is_self_contained(self):
        result, root = self._build(collider="mesh", field_half=4.0)
        meshes = root.findall("./asset/mesh")
        geoms = root.findall("./worldbody/geom")
        self.assertEqual(len(meshes), result["n_stones"])
        # Every stone mesh is inline, so the scene has no external file refs -
        # nothing to resolve against the meshdir of the merged model.
        for mesh in meshes:
            self.assertIn("vertex", mesh.attrib)
            self.assertNotIn("file", mesh.attrib)
        names = {geom.get("name") for geom in geoms}
        self.assertIn("platform", names)
        self.assertIn("pit_floor", names)
        self.assertEqual(len(geoms), result["n_stones"] + 2)

    def _mesh_top_rings(self, root):
        """Per stone, the top-face vertex heights (the mesh is top ring then bottom)."""
        for mesh in root.findall("./asset/mesh"):
            zs = np.array(mesh.get("vertex").split(), dtype=float).reshape(-1, 3)[:, 2]
            yield zs[: len(zs) // 2], zs[len(zs) // 2:]

    def test_mesh_prisms_reach_down_to_the_pit_floor(self):
        """However the top is offset or tilted, no stone may float."""
        result, root = self._build(collider="mesh", field_half=4.0)
        for _, bottom in self._mesh_top_rings(root):
            np.testing.assert_allclose(bottom, -result["depth"], atol=1e-4)

    def test_mesh_tops_vary_and_tilt_within_bounds(self):
        result, root = self._build(collider="mesh", difficulty=1.0, field_half=4.0)
        cfg = pt.make_cfg(size=8.0)
        cur = pt.resolve_curriculum(1.0, cfg)
        rises, levels = [], []
        for top, _ in self._mesh_top_rings(root):
            rises.append(top.max() - top.min())   # tilt across one stone
            levels.append(top.mean())             # roughly that stone's height
        self.assertGreater(max(rises), 1e-3, "every stone top is still level")
        self.assertGreater(np.std(levels), 1e-3, "every stone is at the same height")
        # A vertex can be as far off level as the centre offset plus the tilt
        # carried across the stone; the vertex mean is bounded by the same.
        limit = result["height_var"] + np.tan(np.radians(cur["tilt_deg"])) * (
            cur["stone_radius"] * 2.0)
        self.assertLessEqual(max(np.abs(levels)), limit)

    def test_level_terrain_keeps_every_top_at_zero(self):
        """With the uneven-top knobs off, the old single-level field comes back."""
        _, root = self._build(collider="mesh", difficulty=1.0, field_half=4.0,
                              max_height_var=0.0, max_tilt_deg=0.0)
        for top, _ in self._mesh_top_rings(root):
            np.testing.assert_allclose(top, 0.0, atol=1e-4)

    def test_mesh_platform_top_is_at_zero(self):
        result, root = self._build(collider="mesh", field_half=4.0)
        platform = root.find("./worldbody/geom[@name='platform']")
        half_z = float(platform.get("size").split()[2])
        pos_z = float(platform.get("pos").split()[2])
        self.assertAlmostEqual(pos_z + half_z, 0.0, places=4)
        self.assertAlmostEqual(half_z * 2.0, result["depth"], places=4)

    def test_hfield_scene_writes_its_data_file(self):
        result, root = self._build(collider="hfield", field_half=4.0)
        hfield = root.find("./asset/hfield")
        self.assertIsNotNone(hfield)
        # Absolute, because xacro2mjcf points the merged model's meshdir
        # elsewhere before including this scene.
        data_path = hfield.get("file")
        self.assertTrue(os.path.isabs(data_path))
        self.assertTrue(os.path.exists(data_path), f"{data_path} was never written")
        self.assertIn(data_path, result["files"])
        # nrow/ncol come from the file itself and must not be repeated here.
        self.assertNotIn("nrow", hfield.attrib)
        self.assertNotIn("ncol", hfield.attrib)

    def test_hfield_geom_puts_the_spawn_platform_at_zero(self):
        """Anchor the platform, not the tallest stone - the spawn sits on it."""
        result, root = self._build(collider="hfield", difficulty=1.0, field_half=4.0)
        hfield = root.find("./asset/hfield")
        elevation = float(hfield.get("size").split()[2])
        pos_z = float(root.find("./worldbody/geom[@name='terrain']").get("pos").split()[2])

        with open(hfield.get("file"), "rb") as handle:
            nrow, ncol = struct.unpack("<ii", handle.read(8))
            data = np.frombuffer(handle.read(), dtype="<f4")
        low, high = float(data.min()), float(data.max())
        # MuJoCo renormalises to [0, 1] and scales by `elevation`.
        self.assertAlmostEqual(elevation, high - low, places=4)

        def world_z(sample):
            return pos_z + elevation * (sample - low) / (high - low)

        platform_level = pt.SteppingStonesCfg.stone_height
        self.assertAlmostEqual(world_z(platform_level), 0.0, places=3)
        # The tallest stone really does end up above the platform.
        self.assertGreater(world_z(high), 0.0)
        self.assertAlmostEqual(world_z(low), -result["depth"], places=3)

    def test_build_scene_reports_what_it_wrote(self):
        result, _ = self._build(collider="mesh", difficulty=1.0, field_half=4.0)
        self.assertEqual(result["difficulty"], 1.0)
        self.assertGreater(result["n_stones"], 0)
        self.assertGreater(result["height_var"], 0.0)
        self.assertGreater(result["tilt_deg"], 0.0)
        for path in result["files"]:
            self.assertTrue(os.path.exists(path))

    def test_bad_collider_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                pt.build_scene(out_dir=tmp, collider="box")

    def test_scene_does_not_clash_with_the_default_scene(self):
        """It replaces mjcf/scene.xml; both included at once would repeat names."""
        _, root = self._build(collider="mesh", field_half=4.0)
        self.assertIsNotNone(root.find("./asset/material[@name='groundplane']"))
        self.assertEqual(root.tag, "mujoco")


if __name__ == "__main__":
    unittest.main()
