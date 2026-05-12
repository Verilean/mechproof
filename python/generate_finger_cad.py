"""MechProof PoC 2 — multi-part finger CAD generator.

Reads `out/finger_params.json` (emitted only after the Lean kinematic proof
passes) and produces three STEP files — one per link — together with a
`physics_meta.json` summarising mass properties needed by the MuJoCo digital
twin downstream.

Each link is a rounded prism with a transverse hinge hole at the proximal end
and a rounded boss at the distal end. The hinge axis is X-aligned; the link
extends along +Y by `length`. The proximal pivot sits at the origin so the
MJCF generator can stack the links by translating along +Y.

All units are millimetres; physics_meta exports them in metres for MuJoCo.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import cadquery as cq

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "finger_params.json"
META_PATH = REPO_ROOT / "out" / "physics_meta.json"
OUT_DIR = REPO_ROOT / "out"

# Plastic density (PLA, ~1240 kg/m^3) — used to turn CadQuery volume (mm^3)
# into mass (kg) for MuJoCo's inertial element.
DENSITY_KG_PER_M3 = 1240.0


def build_link(length: float, thickness: float, hole_radius: float) -> cq.Workplane:
    """Build one finger link.

    Geometry: a rounded prism of size (thickness, length, thickness), with a
    through-hole at (0, 0, 0) along X (the hinge axis) and a half-cylinder
    boss at the distal end (y = length) along X. The proximal pivot is at the
    origin.
    """
    body = (
        cq.Workplane("XY")
        .box(thickness, length, thickness, centered=(True, False, True))
        .edges("|Y")
        .fillet(thickness * 0.25)
    )

    distal_boss = (
        cq.Workplane("YZ")
        .workplane(offset=length)
        .cylinder(thickness, thickness / 2, centered=(True, True, False))
    )

    proximal_boss = (
        cq.Workplane("YZ")
        .cylinder(thickness, thickness / 2, centered=(True, True, False))
    )

    link = body.union(distal_boss).union(proximal_boss)

    pivot_hole = cq.Workplane("YZ").cylinder(thickness * 2, hole_radius)
    link = link.cut(pivot_hole)

    return link


def link_physics(link: cq.Workplane, name: str) -> dict:
    """Extract mass properties for MuJoCo. Returns SI-unit metadata."""
    solid = link.val()
    volume_mm3 = float(solid.Volume())
    com = solid.Center()                # mm, in part-local frame
    bb = solid.BoundingBox()            # mm

    volume_m3 = volume_mm3 * 1e-9
    mass_kg = volume_m3 * DENSITY_KG_PER_M3

    return {
        "name": name,
        "volume_mm3": volume_mm3,
        "mass_kg": mass_kg,
        "com_m": [com.x * 1e-3, com.y * 1e-3, com.z * 1e-3],
        "bbox_m": {
            "xmin": bb.xmin * 1e-3, "xmax": bb.xmax * 1e-3,
            "ymin": bb.ymin * 1e-3, "ymax": bb.ymax * 1e-3,
            "zmin": bb.zmin * 1e-3, "zmax": bb.zmax * 1e-3,
        },
    }


def main() -> int:
    if not PARAMS_PATH.exists():
        print(f"error: {PARAMS_PATH} not found — run Lean verification first.",
              file=sys.stderr)
        return 1

    p = json.loads(PARAMS_PATH.read_text())
    lengths = [float(p["l1"]), float(p["l2"]), float(p["l3"])]
    thickness = float(p["thickness"])
    hole_radius = thickness * 0.25

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    meta = {
        "density_kg_per_m3": DENSITY_KG_PER_M3,
        "thickness_m": thickness * 1e-3,
        "links": [],
    }

    for i, L in enumerate(lengths, start=1):
        link = build_link(L, thickness, hole_radius)
        step_path = OUT_DIR / f"link{i}.step"
        link.val().exportStep(str(step_path))
        phys = link_physics(link, f"link{i}")
        phys["length_m"] = L * 1e-3
        meta["links"].append(phys)
        print(f"Wrote {step_path}  "
              f"(L={L:.1f}mm  mass={phys['mass_kg']*1000:.2f}g  "
              f"vol={phys['volume_mm3']:.1f}mm^3)")

    META_PATH.write_text(json.dumps(meta, indent=2))
    print(f"Wrote {META_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
