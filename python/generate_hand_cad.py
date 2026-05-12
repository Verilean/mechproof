"""MechProof PoC 4 — full hand CAD generator.

Generates one `palm.step`, one `thumb_swivel_base.step`, and 3 tendon-channelled
links per finger (15 STEPs total for the 5 fingers, named like
`index_link1.step`). Also emits `hand_physics_meta.json`, which the MuJoCo
composer uses to build the kinematic tree (mount positions, link physics,
moment arms).

All distances are in metres (matches Lean's `hand_params.json`). PLA density
is used for mass; the tendon channel radius matches PoC 3.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys
from typing import Iterable

import cadquery as cq

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "hand_params.json"
META_PATH = REPO_ROOT / "out" / "hand_physics_meta.json"
OUT_DIR = REPO_ROOT / "out"

DENSITY_KG_PER_M3 = 1240.0

# Tendon channel radius and per-joint moment arms are read straight from
# `out/hand_params.json` (the file the Lean DFM proof writes). The Python
# layer does not redefine them — if the JSON exists, those numbers are
# already formally certified to satisfy the DFM rules.

FINGERS = ("index", "middle", "ring", "pinky", "thumb")


def build_palm(width_m: float, length_m: float, thickness_m: float) -> cq.Workplane:
    """A simple rounded slab. Y goes from 0 (wrist) to +length (knuckles)."""
    return (
        cq.Workplane("XY")
        .box(width_m, length_m, thickness_m, centered=(True, False, True))
        .edges("|Z")
        .fillet(thickness_m * 0.5)
    )


def build_swivel_base(thumb_thickness_m: float) -> cq.Workplane:
    """A short cylindrical hub on which the thumb's first link rotates about
    the +Z axis. Sized so its diameter ≈ twice the thumb thickness."""
    return cq.Workplane("XY").cylinder(
        thumb_thickness_m, thumb_thickness_m, centered=(True, True, True)
    )


def build_link(length_m: float, thickness_m: float, hole_radius_m: float,
               moment_arm_m: float, channel_radius_m: float) -> cq.Workplane:
    """Same shape as PoC 3, but every dimension comes from JSON."""
    L_mm = length_m * 1000
    th_mm = thickness_m * 1000
    hr_mm = hole_radius_m * 1000
    r_mm = moment_arm_m * 1000
    ch_mm = channel_radius_m * 1000

    body = (
        cq.Workplane("XY")
        .box(th_mm, L_mm, th_mm, centered=(True, False, True))
        .edges("|Y")
        .fillet(th_mm * 0.25)
    )
    distal_boss = (
        cq.Workplane("YZ")
        .workplane(offset=L_mm)
        .cylinder(th_mm, th_mm / 2, centered=(True, True, False))
    )
    proximal_boss = (
        cq.Workplane("YZ")
        .cylinder(th_mm, th_mm / 2, centered=(True, True, False))
    )
    link = body.union(distal_boss).union(proximal_boss)
    pivot_hole = cq.Workplane("YZ").cylinder(th_mm * 2, hr_mm)
    link = link.cut(pivot_hole)

    tendon_channel = (
        cq.Workplane("XZ")
        .workplane(offset=L_mm / 2)
        .center(0, -r_mm)
        .cylinder(L_mm + th_mm * 2, ch_mm)
    )
    link = link.cut(tendon_channel)
    return link


def solid_physics(solid) -> dict:
    volume_mm3 = float(solid.Volume())
    com = solid.Center()
    bb = solid.BoundingBox()
    volume_m3 = volume_mm3 * 1e-9
    mass_kg = volume_m3 * DENSITY_KG_PER_M3
    return {
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
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    palm = p["palm"]
    palm_solid = build_palm(palm["width"], palm["length"], palm["thickness"])
    palm_path = OUT_DIR / "palm.step"
    palm_solid.val().exportStep(str(palm_path))
    print(f"Wrote {palm_path}")

    thumb = p["thumb"]
    swivel_solid = build_swivel_base(thumb["thickness"])
    swivel_path = OUT_DIR / "thumb_swivel_base.step"
    swivel_solid.val().exportStep(str(swivel_path))
    print(f"Wrote {swivel_path}")

    dfm = p["dfm"]
    meta = {
        "density_kg_per_m3": DENSITY_KG_PER_M3,
        "min_wall_thickness_m": float(dfm["minWallThicknessM"]),
        "min_tendon_hole_dia_m": float(dfm["minTendonHoleDiaM"]),
        "palm": {
            **palm,
            **solid_physics(palm_solid.val()),
        },
        "swivelMaxRad": float(p["swivelMaxRad"]),
        "fingers": {},
    }

    for name in FINGERS:
        f = p[name]
        mount = p[f"{name}Mount"]
        thickness = float(f["thickness"])
        hole_radius = thickness * 0.25
        lengths = (float(f["l1"]), float(f["l2"]), float(f["l3"]))

        channels = dfm[f"{name}Channels"]
        moment_arms = (
            float(channels["ch1"]["momentArm"]),
            float(channels["ch2"]["momentArm"]),
            float(channels["ch3"]["momentArm"]),
        )
        channel_radii = (
            float(channels["ch1"]["channelRadius"]),
            float(channels["ch2"]["channelRadius"]),
            float(channels["ch3"]["channelRadius"]),
        )

        finger_meta = {
            "mount": mount,
            "thickness_m": thickness,
            "moment_arms_m": list(moment_arms),
            "channel_radii_m": list(channel_radii),
            "joint_range_rad": [
                math.radians(float(f["minAngle"])),
                math.radians(float(f["maxAngle"])),
            ],
            "links": [],
        }

        for i, (L, r, cr) in enumerate(
                zip(lengths, moment_arms, channel_radii), start=1):
            link = build_link(L, thickness, hole_radius, r, cr)
            step_path = OUT_DIR / f"{name}_link{i}.step"
            link.val().exportStep(str(step_path))
            phys = solid_physics(link.val())
            phys["name"] = f"{name}_link{i}"
            phys["length_m"] = L
            phys["moment_arm_m"] = r
            phys["channel_radius_m"] = cr
            finger_meta["links"].append(phys)
            print(f"Wrote {step_path}  "
                  f"(L={L*1000:.1f}mm  r={r*1000:.2f}mm  "
                  f"Φch={cr*2000:.2f}mm  "
                  f"mass={phys['mass_kg']*1000:.2f}g)")

        meta["fingers"][name] = finger_meta

    META_PATH.write_text(json.dumps(meta, indent=2))
    print(f"Wrote {META_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
