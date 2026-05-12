"""MechProof PoC 3 — tendon-routed finger CAD generator.

Each link gets the PoC 2 geometry (rounded prism + proximal/distal bosses + hinge
hole) **plus** a longitudinal cylindrical channel for the flexor tendon,
offset by the moment arm `r_i` from the joint axis toward the palmar (-Z)
surface. The channel is fully through the link along its long axis (Y) so
the tendon can be threaded through all three links.

Coordinate convention (matches PoC 2):
  * +Y is the long axis of the link, +X is the hinge axis, +Z is dorsal.
  * The hinge axis is at Z=0, so the tendon channel is at Z = -r_i.

Mass properties are re-extracted after the cut (mass and CoM shift because
of the missing material), and the moment arms `r_i` are persisted in
physics_meta.json so the MuJoCo composer can drive the <fixed> tendon
without re-reading tendon_params.json.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import cadquery as cq

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "tendon_params.json"
META_PATH = REPO_ROOT / "out" / "physics_meta.json"
OUT_DIR = REPO_ROOT / "out"

DENSITY_KG_PER_M3 = 1240.0  # PLA
TENDON_CHANNEL_RADIUS_MM = 0.6  # ~1.2 mm-diameter tendon cable bore


def build_link(length: float, thickness: float, hole_radius: float,
               moment_arm: float) -> cq.Workplane:
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

    # Tendon channel: longitudinal cylinder along +Y at z = -moment_arm.
    # Built on the XZ plane (normal = +Y) so the cylinder extends along Y.
    tendon_channel = (
        cq.Workplane("XZ")
        .workplane(offset=length / 2)            # centre the cylinder mid-link
        .center(0, -moment_arm)                  # palmar offset
        .cylinder(length + thickness * 2, TENDON_CHANNEL_RADIUS_MM)
    )
    link = link.cut(tendon_channel)

    return link


def link_physics(link: cq.Workplane, name: str) -> dict:
    solid = link.val()
    volume_mm3 = float(solid.Volume())
    com = solid.Center()
    bb = solid.BoundingBox()
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
    lengths_mm = [float(p["l1"]), float(p["l2"]), float(p["l3"])]
    moment_arms_mm = [float(p["r1"]), float(p["r2"]), float(p["r3"])]
    thickness = float(p["thickness"])
    hole_radius = thickness * 0.25

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    meta = {
        "density_kg_per_m3": DENSITY_KG_PER_M3,
        "thickness_m": thickness * 1e-3,
        "tendon_channel_radius_m": TENDON_CHANNEL_RADIUS_MM * 1e-3,
        "moment_arms_m": [r * 1e-3 for r in moment_arms_mm],
        "joint_range_rad": [
            math.radians(float(p["minAngle"])),
            math.radians(float(p["maxAngle"])),
        ],
        "links": [],
    }

    for i, (L, r) in enumerate(zip(lengths_mm, moment_arms_mm), start=1):
        link = build_link(L, thickness, hole_radius, r)
        step_path = OUT_DIR / f"link{i}.step"
        link.val().exportStep(str(step_path))
        phys = link_physics(link, f"link{i}")
        phys["length_m"] = L * 1e-3
        phys["moment_arm_m"] = r * 1e-3
        meta["links"].append(phys)
        print(f"Wrote {step_path}  "
              f"(L={L:.1f}mm  r={r:.2f}mm  "
              f"mass={phys['mass_kg']*1000:.2f}g  "
              f"vol={phys['volume_mm3']:.1f}mm^3)")

    META_PATH.write_text(json.dumps(meta, indent=2))
    print(f"Wrote {META_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
