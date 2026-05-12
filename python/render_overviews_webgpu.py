"""MechProof — WebGPU scene preview renderer.

OpenGL-free replacement for `render_overviews.py`. Reads a MuJoCo MJCF
into `mujoco.MjModel` for the *geometry/pose* information only — never
touches `mujoco.Renderer` — and re-builds the scene as a `pygfx`
(WebGPU) scene graph. Renders to PNG via the offscreen WebGPU canvas.

Why bother?

  * MuJoCo's built-in renderer requires OpenGL (EGL / OSMesa / GLFW).
    Headless OpenGL on cloud CI runners is a moving target (no EGL
    device, OSMesa needs libGL anyway, etc.).
  * WebGPU has well-defined offscreen semantics. On Linux it routes
    through Vulkan + the Lavapipe software ICD that already ships in
    Mesa, so we don't need ANY GPU driver — just `vulkan-loader` +
    `mesa` (both in shell.nix).
  * The same code runs on macOS (Metal) and Windows (D3D12) too.

This wrapper is intentionally small — about 250 lines — and only
handles the primitive geom types (box / sphere / cylinder / capsule /
plane / ellipsoid). MJCF meshes are skipped with a one-line warning.

Output PNGs land at
    out/preview_<scene>_<angle>_webgpu.png
so they coexist with the legacy OpenGL renderer's output.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
from typing import Iterable

import numpy as np

import mujoco
import pygfx as gfx
import pylinalg as la
from rendercanvas.offscreen import OffscreenRenderCanvas

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "out"

SCENES = [
    "humanoid_scene",
    "humanoid_scene_v2",
    "humanoid_scene_subsea",
    "humanoid_scene_heavy",
    "arm_hand_scene",
    "grasp_scene",
    "finger",
]

# (azimuth°, elevation°, distance multiplier of model.stat.extent).
# Small positive elevations look down on the floor slightly so the
# robot's feet visibly contact the ground plane.
ANGLES = {
    "front": (90.0, 10.0, 1.6),
    "side":  ( 0.0, 10.0, 1.6),
    "iso":   (45.0, 20.0, 1.7),
    "top":   ( 0.0, 80.0, 1.6),
}

RENDER_WIDTH = 800
RENDER_HEIGHT = 600


# ─────────────────────────────────────────────────────────────────────
#  Primitive → pygfx geometry adapters
# ─────────────────────────────────────────────────────────────────────

# MuJoCo geom_type integer codes (from mjtGeom).
GT_PLANE     = 0
GT_HFIELD    = 1
GT_SPHERE    = 2
GT_CAPSULE   = 3
GT_ELLIPSOID = 4
GT_CYLINDER  = 5
GT_BOX       = 6
GT_MESH      = 7


def geom_to_pygfx(geom_type: int, size: np.ndarray) -> gfx.Geometry | None:
    """Translate a MuJoCo geom into the corresponding pygfx geometry.
    `size` is the 3-vector MuJoCo stores in `model.geom_size[i]`."""
    if geom_type == GT_BOX:
        # MuJoCo stores half-extents; pygfx wants full extents.
        sx, sy, sz = float(size[0]), float(size[1]), float(size[2])
        return gfx.box_geometry(2 * sx, 2 * sy, 2 * sz)
    if geom_type == GT_SPHERE:
        return gfx.sphere_geometry(float(size[0]))
    if geom_type == GT_CYLINDER:
        # MuJoCo stores (radius, half-length, _). Both MuJoCo and pygfx
        # use the +Z axis as the cylinder's long axis with the centroid
        # at the origin, so no axis remapping is needed.
        r, half_h = float(size[0]), float(size[1])
        return gfx.cylinder_geometry(
            radius_top=r, radius_bottom=r, height=2 * half_h)
    if geom_type == GT_CAPSULE:
        # MJ capsule: cylinder of (2 * half_h) along +Z, plus a hemisphere
        # of radius r at each end. We approximate as a single cylinder of
        # length 2 * half_h — the hemispherical caps are visually minor
        # for preview purposes and never affect physics.
        r, half_h = float(size[0]), float(size[1])
        return gfx.cylinder_geometry(
            radius_top=r, radius_bottom=r, height=2 * half_h)
    if geom_type == GT_ELLIPSOID:
        # pygfx has no ellipsoid; we scale a sphere via the transform
        # below. Returning a unit sphere here is correct; the per-axis
        # scaling happens in `attach_pose`.
        return gfx.sphere_geometry(1.0)
    if geom_type == GT_PLANE:
        # MJ plane: size[0] / size[1] are half-extents (0 means infinite).
        sx = float(size[0]) if size[0] > 0 else 5.0
        sy = float(size[1]) if size[1] > 0 else 5.0
        # Render as a thin box so it's visible.
        return gfx.box_geometry(2 * sx, 2 * sy, 0.01)
    # PLANE-with-size-0, HFIELD, MESH, etc. — skip silently.
    return None


def material_for(rgba: np.ndarray) -> gfx.MeshPhongMaterial:
    r, g, b, a = (float(rgba[0]), float(rgba[1]),
                  float(rgba[2]), float(rgba[3]))
    return gfx.MeshPhongMaterial(color=(r, g, b, a))


def attach_pose(mesh: gfx.Mesh, pos: np.ndarray, mat3: np.ndarray,
                geom_type: int, size: np.ndarray) -> None:
    """Set a pygfx Mesh's pose from MuJoCo's geom_xpos + geom_xmat.

    Mathematical setup. A point ``v_mj`` in MuJoCo world frame maps to
    ``v_pg = T · v_mj`` in pygfx world frame, where ``T = MJ_TO_PYGFX``.
    A rigid-body transform ``(R_mj, p_mj)`` in MuJoCo coords becomes
    ``(T R_mj T⁻¹, T p_mj)`` in pygfx coords.

    Cylinder/capsule primitives ship with their long axis along **local
    +Y** in pygfx but along **local +Z** in MuJoCo's geom_xmat
    convention.  We compensate by right-multiplying a fixed -90°-about-X
    rotation **before** the world-frame change of basis.  This rotates
    the primitive's local frame so what pygfx labels "+Y" coincides with
    what MuJoCo labels "+Z".
    """
    R_mj = np.asarray(mat3, dtype=np.float64).reshape(3, 3)

    # World-frame change of basis. pygfx's cylinder/capsule primitive
    # already uses +Z as its long axis (same as MuJoCo), so no
    # primitive-level rotation is required — `T` alone handles the
    # MJ +Z-up → pygfx +Y-up world swap.
    T = MJ_TO_PYGFX
    R_pg = T @ R_mj @ T.T
    p_pg = T @ np.asarray(pos, dtype=np.float64)

    # pygfx stores the local matrix as a 4×4 in *column-major* layout.
    # We assign through `mesh.local.matrix` which accepts a row-major
    # numpy array and handles the conversion.
    M = np.eye(4, dtype=np.float64)
    M[:3, :3] = R_pg
    M[:3, 3] = p_pg
    mesh.local.matrix = M

    if geom_type == GT_ELLIPSOID:
        mesh.local.scale = (float(size[0]), float(size[1]), float(size[2]))


# ─────────────────────────────────────────────────────────────────────
#  Scene assembly
# ─────────────────────────────────────────────────────────────────────

# We make the pygfx world coincide with the MuJoCo world: +Z up,
# +Y forward, +X right.  pygfx is happy with any "up" axis as long as
# we tell the camera, so there is no change of basis to apply on the
# geometry side — positions and rotations pass through unchanged.
MJ_TO_PYGFX = np.eye(3, dtype=np.float64)


def build_scene(model: mujoco.MjModel, data: mujoco.MjData) -> tuple:
    """Walk every visible geom in the model, instantiate a pygfx Mesh
    for it, parent everything to a single Scene. Returns (scene, lookat,
    bbox_extent) where `lookat` is the centroid of all geom positions
    and `bbox_extent` is the diagonal of the bounding box (used to
    place the camera)."""
    scene = gfx.Scene()
    scene.add(gfx.AmbientLight(intensity=0.4))
    key = gfx.DirectionalLight(intensity=2.0)
    key.local.position = (3, 5, 3)
    scene.add(key)

    positions: list[np.ndarray] = []
    n_skipped = 0

    for gid in range(model.ngeom):
        gtype = int(model.geom_type[gid])
        gsize = model.geom_size[gid]
        geometry = geom_to_pygfx(gtype, gsize)
        if geometry is None:
            n_skipped += 1
            continue
        rgba = model.geom_rgba[gid]
        material = material_for(rgba)
        mesh = gfx.Mesh(geometry, material)
        attach_pose(mesh, data.geom_xpos[gid], data.geom_xmat[gid],
                    gtype, gsize)
        scene.add(mesh)
        positions.append(np.asarray(data.geom_xpos[gid]))

    if not positions:
        return scene, np.zeros(3), 1.0

    P = np.stack(positions, axis=0)
    # Use MuJoCo's own scene-extent metric (set during mj_forward); it
    # accounts for primitive *radii*, not just centre points, so the
    # camera-distance heuristic frames the model rather than clipping
    # off thin limbs that stick out past the centroid bounding box.
    centroid_mj = P.mean(axis=0)
    extent = max(float(model.stat.extent),
                 float(np.linalg.norm(P.max(axis=0) - P.min(axis=0))),
                 1.0)
    if n_skipped:
        print(f"  ({n_skipped} unsupported geoms skipped)")
    centroid_pygfx = MJ_TO_PYGFX @ centroid_mj
    return scene, centroid_pygfx, extent


def camera_at(az_deg: float, el_deg: float,
              distance: float, lookat: np.ndarray) -> gfx.PerspectiveCamera:
    """Spherical → cartesian camera placement around `lookat`, in the
    MuJoCo-compatible +Z-up frame we set up in `build_scene`.

    Azimuth is measured from +X around the +Z axis (so az=0 looks
    toward +X, az=90 looks toward +Y). Elevation is the angle above
    the X-Y horizontal plane; +85° gives a near-top-down view.
    """
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    pos = lookat + distance * np.array([
        math.cos(el) * math.cos(az),
        math.cos(el) * math.sin(az),
        math.sin(el),                       # elevation lifts along +Z
    ])
    cam = gfx.PerspectiveCamera(fov=45, aspect=RENDER_WIDTH / RENDER_HEIGHT)
    cam.world.up = (0.0, 0.0, 1.0)
    cam.local.position = tuple(pos)
    cam.show_pos(tuple(lookat), up=(0.0, 0.0, 1.0))
    return cam


# ─────────────────────────────────────────────────────────────────────
#  Top-level rendering
# ─────────────────────────────────────────────────────────────────────

def render_scene(xml_path: pathlib.Path, basename: str) -> int:
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
    except Exception as e:
        print(f"  skip ({xml_path.name}): {e}")
        return 0

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    scene, centroid, extent = build_scene(model, data)

    canvas = OffscreenRenderCanvas(size=(RENDER_WIDTH, RENDER_HEIGHT))
    renderer = gfx.WgpuRenderer(canvas)

    n_written = 0
    for angle_name, (az, el, scale) in ANGLES.items():
        cam = camera_at(az, el, extent * scale, centroid)
        renderer.render(scene, cam)
        canvas.draw()
        image = np.asarray(canvas.draw())
        out_png = OUT_DIR / f"preview_{basename}_{angle_name}_webgpu.png"
        from PIL import Image
        Image.fromarray(image).save(out_png)
        n_written += 1
        print(f"  wrote {out_png.name}")

    return n_written


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # Make sure wgpu picks Vulkan (Lavapipe) on Linux.
    os.environ.setdefault("WGPU_BACKEND_TYPE", "Vulkan")

    total = 0
    for basename in SCENES:
        xml = OUT_DIR / f"{basename}.xml"
        if not xml.exists():
            continue
        print(f"Rendering {xml.name}…")
        total += render_scene(xml, basename)

    if total == 0:
        print("note: no MuJoCo scenes found in out/. Run a PoC first.")
    else:
        print(f"\nDone. {total} preview PNGs (WebGPU) in {OUT_DIR}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
