"""MechProof — headless preview renderer.

Loads every MuJoCo scene that the PoC pipelines emit and renders each
from a few standard "showroom" camera angles to PNG. The output files
are picked up by the GitHub Actions workflow as build artefacts so PR
reviewers can eyeball the geometry without needing a STEP viewer.

PNGs land at:
  out/preview_<scene>_<angle>.png

`<scene>`  is the basename of the MJCF (e.g. `humanoid_scene_heavy`).
`<angle>`  is one of `front`, `side`, `iso`, `top`.
"""

from __future__ import annotations

import os
import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "out"

# Each entry is the basename (without `.xml`) of an MJCF we expect to
# find in out/. Missing files are silently skipped, so the renderer is
# safe to invoke even when only a subset of PoCs has been built.
SCENES = [
    "humanoid_scene",         # PoC 8 stand
    "humanoid_scene_v2",      # PoC 11 teleop (camera + IMU)
    "humanoid_scene_subsea",  # PoC 13 subsea
    "humanoid_scene_heavy",   # PoC 15 4 m mech
    "arm_hand_scene",         # PoC 6 / 7
    "grasp_scene",            # PoC 3 grasp
    "finger",                 # PoC 2 single-finger sim
]

# Each angle = (azimuth°, elevation°, distance scaling factor).
# Distance is multiplied by the model's auto-computed extent so the
# robot fits in frame at every scale (humanoid 1.55 m or heavy 4 m).
ANGLES = {
    "front": (90.0, -10.0, 1.6),
    "side":  ( 0.0, -10.0, 1.6),
    "iso":   (45.0, -20.0, 1.7),
    "top":   ( 0.0, -85.0, 1.6),
}

# MuJoCo's default offscreen framebuffer is 640×480.  We stay inside
# that so we don't have to inject <global offwidth/offheight> into every
# generated MJCF.
RENDER_WIDTH = 640
RENDER_HEIGHT = 480


def render_scene(xml_path: pathlib.Path, basename: str,
                 renderer_size: tuple) -> int:
    """Render one scene from every angle; return count of PNGs written."""
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
    except Exception as e:
        print(f"  skip ({xml_path.name}): {e}")
        return 0

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    # Centre the camera on the model's bounding-box centroid.
    cam.lookat[:] = data.subtree_com[0]

    extent = float(model.stat.extent) if float(model.stat.extent) > 1e-3 else 1.5
    # mujoco.Renderer signature is (model, height, width).
    w, h = renderer_size
    renderer = mujoco.Renderer(model, h, w)
    n_written = 0
    try:
        for angle_name, (az, el, scale) in ANGLES.items():
            cam.azimuth = az
            cam.elevation = el
            cam.distance = extent * scale
            renderer.update_scene(data, camera=cam)
            frame = renderer.render()

            out_png = OUT_DIR / f"preview_{basename}_{angle_name}.png"
            try:
                from PIL import Image  # type: ignore
                Image.fromarray(frame).save(out_png)
            except ImportError:
                out_png = out_png.with_suffix(".rgb")
                out_png.write_bytes(frame.tobytes())
            n_written += 1
            print(f"  wrote {out_png.name}")
    finally:
        if hasattr(renderer, "close"):
            renderer.close()
    return n_written


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    # MuJoCo needs an offscreen GL backend on CI / headless boxes.
    os.environ.setdefault("MUJOCO_GL", "egl")

    total = 0
    for basename in SCENES:
        xml = OUT_DIR / f"{basename}.xml"
        if not xml.exists():
            continue
        print(f"Rendering {xml.name}…")
        total += render_scene(xml, basename, (RENDER_WIDTH, RENDER_HEIGHT))

    if total == 0:
        print("note: no MuJoCo scenes found in out/. Run a PoC first.")
    else:
        print(f"\nDone. {total} preview PNGs in {OUT_DIR}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
