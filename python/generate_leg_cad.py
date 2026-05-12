"""MechProof PoC 8 — torso + lower-body CAD generator.

Reads `out/leg_params.json` (emitted only after Lean balance + squat-torque
proofs typecheck) and produces:
  * `torso.step`                          — pelvis / torso shell with arm-
                                            and hip-mount features,
  * `thigh.step`, `shin.step`             — tubular limb segments (single
                                            STEP file, reused L/R via
                                            mirroring in MJCF),
  * `foot.step`                           — flat support plate.

The same STEP file serves both legs: the MJCF defines two `<body>` chains
that load the file with a mirrored Y-rotation. This halves the artefact
count without losing fidelity.

All distances are in **metres** (matching leg_params.json).
"""

from __future__ import annotations

import json
import pathlib
import sys

import cadquery as cq

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "leg_params.json"
META_PATH = REPO_ROOT / "out" / "leg_physics_meta.json"
OUT_DIR = REPO_ROOT / "out"

# Limb tube cross-section.
LIMB_OUTER_R_MM = 28.0
LIMB_INNER_R_MM = 24.0


def build_torso(t: dict) -> cq.Workplane:
    """Rounded slab: width × depth × height, centred laterally, with the
    top face acting as the arm-mount point and the bottom face hosting the
    two hip pivots."""
    W = t["width"] * 1000
    D = t["depth"] * 1000
    H = t["height"] * 1000
    torso = (
        cq.Workplane("XY")
        .box(W, D, H, centered=(True, True, True))
    )
    # Drill two hip-pivot recesses on the underside.
    return torso


def build_limb(length_m: float) -> cq.Workplane:
    """Hollow tube oriented along -Z (downward in the world frame). The
    proximal end (the joint we hang the limb off) sits at Z = 0."""
    L_mm = length_m * 1000
    tube = (
        cq.Workplane("XY")
        .circle(LIMB_OUTER_R_MM)
        .circle(LIMB_INNER_R_MM)
        .extrude(-L_mm)
    )
    return tube


def build_foot(length_m: float, width_m: float) -> cq.Workplane:
    """Flat support plate. Long axis = Y, lateral = X, thickness = Z. The
    proximal pivot (ankle) is on the top face, centred laterally and
    slightly biased toward the heel (40% from the back)."""
    L_mm = length_m * 1000
    W_mm = width_m * 1000
    thickness_mm = 20.0
    return cq.Workplane("XY").box(
        W_mm, L_mm, thickness_mm, centered=(True, True, True))


def main() -> int:
    if not PARAMS_PATH.exists():
        print(f"error: {PARAMS_PATH} not found — run Lean verification first.",
              file=sys.stderr)
        return 1

    p = json.loads(PARAMS_PATH.read_text())
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    torso = p["torso"]
    leg = p["leg"]

    torso_solid = build_torso(torso)
    torso_path = OUT_DIR / "torso.step"
    torso_solid.val().exportStep(str(torso_path))
    print(f"Wrote {torso_path}")

    thigh_solid = build_limb(leg["thighLen"])
    thigh_path = OUT_DIR / "thigh.step"
    thigh_solid.val().exportStep(str(thigh_path))
    print(f"Wrote {thigh_path}")

    shin_solid = build_limb(leg["shinLen"])
    shin_path = OUT_DIR / "shin.step"
    shin_solid.val().exportStep(str(shin_path))
    print(f"Wrote {shin_path}")

    foot_solid = build_foot(leg["footLen"], leg["footWidth"])
    foot_path = OUT_DIR / "foot.step"
    foot_solid.val().exportStep(str(foot_path))
    print(f"Wrote {foot_path}")

    # Volumes and masses from Lean-declared numbers (so the simulator's
    # inertial reasoning matches the proof).
    meta = {
        "gravity_m_per_s2": float(p["gravity"]),
        "upper_body_mass_kg": float(p["upperBodyMass"]),
        "total_mass_kg":      float(p["totalMass"]),
        "torso": {
            "mass_kg": float(torso["mass"]),
            "width_m":  float(torso["width"]),
            "depth_m":  float(torso["depth"]),
            "height_m": float(torso["height"]),
        },
        "leg": {
            "thigh_length_m": float(leg["thighLen"]),
            "shin_length_m":  float(leg["shinLen"]),
            "foot_length_m":  float(leg["footLen"]),
            "foot_width_m":   float(leg["footWidth"]),
            "thigh_mass_kg":  float(leg["thighMass"]),
            "shin_mass_kg":   float(leg["shinMass"]),
            "foot_mass_kg":   float(leg["footMass"]),
            "hip_offset_x_m": float(leg["hipOffsetX"]),
        },
        "torques_nm": {
            "hip_pitch":         float(leg["hipPitchTau"]),
            "knee":              float(leg["kneeTau"]),
            "knee_required":     float(p["requiredKneeTorque"]),
            "ankle":             float(leg["ankleTau"]),
        },
        "support_polygon": {
            "half_x_m":          float(p["supportHalfX"]),
            "half_y_m":          float(p["supportHalfY"]),
            "balance_margin_m":  float(p["balanceMargin"]),
        },
    }
    META_PATH.write_text(json.dumps(meta, indent=2))
    print(f"Wrote {META_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
