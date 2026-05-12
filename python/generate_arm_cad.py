"""MechProof PoC 6 — 6-DOF arm CAD generator.

Reads `out/arm_params.json` (only emitted after Lean's stall-torque proof
typechecks) and produces:
  * `arm_link1.step`, `arm_link2.step`, `arm_link3.step`  — tubular links
  * `arm_shoulder_bracket.step`, `arm_elbow_bracket.step` — motor housings
  * `arm_wrist_flange.step`                              — ISO 9409-1-50-4-M6
    style mounting plate at the wrist for bolting the hand on

Coordinate convention (arm's local frame):
  * the shoulder pivot sits at the world origin,
  * +Y is the long axis of each link in its rest pose,
  * +Z is the world "up".

All distances are in **metres** (matching arm_params.json). The link masses
emitted in `arm_physics_meta.json` are taken straight from the JSON, not
re-computed from CAD volume — Lean proved the stall torque using those
exact masses, and the simulator must reuse them so the physics being
simulated is the physics that was proven.
"""

from __future__ import annotations

import json
import pathlib
import sys

import cadquery as cq

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "arm_params.json"
META_PATH = REPO_ROOT / "out" / "arm_physics_meta.json"
OUT_DIR = REPO_ROOT / "out"

# Tubular link cross-section.
LINK_OUTER_R_MM = 22.0
LINK_INNER_R_MM = 18.0

# Motor housing brackets.
BRACKET_W_MM = 60.0
BRACKET_H_MM = 50.0
BRACKET_T_MM = 8.0

# ISO 9409-1-50-4-M6 wrist flange: a 63 mm-diameter plate, 8 mm thick, with
# four M6 clearance holes (Φ6.5) on a 50 mm bolt circle, plus a Φ31.5 spigot.
FLANGE_OD_MM = 63.0
FLANGE_T_MM = 8.0
FLANGE_BOLT_CIRCLE_MM = 50.0
FLANGE_BOLT_HOLE_DIA_MM = 6.5
FLANGE_SPIGOT_DIA_MM = 31.5
FLANGE_SPIGOT_DEPTH_MM = 2.0


def build_link(length_m: float) -> cq.Workplane:
    """Hollow tube oriented along +Y, proximal flange at Y=0."""
    L_mm = length_m * 1000.0
    tube = (
        cq.Workplane("XZ")
        .circle(LINK_OUTER_R_MM)
        .circle(LINK_INNER_R_MM)
        .extrude(L_mm)
    )
    end_cap = lambda y: (
        cq.Workplane("XZ")
        .workplane(offset=y)
        .circle(LINK_OUTER_R_MM)
        .extrude(BRACKET_T_MM)
    )
    return tube.union(end_cap(0.0)).union(end_cap(L_mm - BRACKET_T_MM))


def build_bracket() -> cq.Workplane:
    """A simple plate that hosts the motor at a joint. The fillet was omitted
    because OCP refuses it on this near-square cross-section; CAM-side post-
    processing can chamfer the edges if cosmetics matter."""
    return cq.Workplane("XY").box(
        BRACKET_W_MM, BRACKET_T_MM, BRACKET_H_MM, centered=(True, True, True))


def build_wrist_flange() -> cq.Workplane:
    """ISO 9409-1-50-4-M6 wrist-flange plate. The four bolt holes match the
    standard pattern that PoC 7 / commercial robot hands expect."""
    plate = (
        cq.Workplane("XY")
        .circle(FLANGE_OD_MM / 2)
        .extrude(FLANGE_T_MM)
    )
    spigot = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_T_MM)
        .circle(FLANGE_SPIGOT_DIA_MM / 2)
        .extrude(FLANGE_SPIGOT_DEPTH_MM)
    )
    plate = plate.union(spigot)

    # Drill four M6 clearance holes on a 50 mm bolt circle, indexed at
    # 45° / 135° / 225° / 315° (the standard offset).
    bolt_r = FLANGE_BOLT_CIRCLE_MM / 2
    centers = []
    import math
    for k in range(4):
        ang = math.radians(45.0 + 90.0 * k)
        centers.append((bolt_r * math.cos(ang), bolt_r * math.sin(ang)))
    plate = (
        plate.faces(">Z").workplane()
             .pushPoints(centers)
             .hole(FLANGE_BOLT_HOLE_DIA_MM)
    )
    return plate


def step_path(name: str) -> pathlib.Path:
    return OUT_DIR / f"{name}.step"


def main() -> int:
    if not PARAMS_PATH.exists():
        print(f"error: {PARAMS_PATH} not found — run Lean verification first.",
              file=sys.stderr)
        return 1

    p = json.loads(PARAMS_PATH.read_text())
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    lengths_m = (float(p["l1"]), float(p["l2"]), float(p["l3"]))
    masses_kg = (float(p["m1"]), float(p["m2"]), float(p["m3"]))

    # Links
    link_paths = []
    for i, L in enumerate(lengths_m, start=1):
        solid = build_link(L)
        path = step_path(f"arm_link{i}")
        solid.val().exportStep(str(path))
        link_paths.append(path)
        print(f"Wrote {path}  (L={L*1000:.1f}mm)")

    # Brackets at shoulder and elbow (the wrist brackets are integrated into
    # the wrist flange below).
    for name in ("arm_shoulder_bracket", "arm_elbow_bracket"):
        solid = build_bracket()
        path = step_path(name)
        solid.val().exportStep(str(path))
        print(f"Wrote {path}")

    # Wrist flange (standard mount for the PoC 5 hand).
    flange = build_wrist_flange()
    flange_path = step_path("arm_wrist_flange")
    flange.val().exportStep(str(flange_path))
    print(f"Wrote {flange_path}")

    # Physics metadata. We pass the Lean-declared masses through verbatim
    # so the simulator's inertial reasoning matches the proof.
    meta = {
        "gravity_m_per_s2": float(p["gravity"]),
        "payload_mass_kg":   float(p["payloadMassKg"]),
        "hand_mass_kg":      float(p["handMassKg"]),
        "links": [
            {"name": "arm_link1", "length_m": lengths_m[0], "mass_kg": masses_kg[0]},
            {"name": "arm_link2", "length_m": lengths_m[1], "mass_kg": masses_kg[1]},
            {"name": "arm_link3", "length_m": lengths_m[2], "mass_kg": masses_kg[2]},
        ],
        "torques_nm": {
            "shoulder_required": float(p["requiredShoulderTorque"]),
            "shoulder_supplied": float(p["tauShoulder"]),
            "elbow_required":    float(p["requiredElbowTorque"]),
            "elbow_supplied":    float(p["tauElbow"]),
            "wrist_required":    float(p["requiredWristTorque"]),
            "wrist_supplied":    float(p["tauWrist"]),
        },
        "wrist_flange": {
            "outer_diameter_m":   FLANGE_OD_MM * 1e-3,
            "thickness_m":        FLANGE_T_MM * 1e-3,
            "bolt_circle_dia_m":  FLANGE_BOLT_CIRCLE_MM * 1e-3,
            "bolt_hole_dia_m":    FLANGE_BOLT_HOLE_DIA_MM * 1e-3,
            "spigot_diameter_m":  FLANGE_SPIGOT_DIA_MM * 1e-3,
        },
    }
    META_PATH.write_text(json.dumps(meta, indent=2))
    print(f"Wrote {META_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
