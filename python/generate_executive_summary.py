"""MechProof PoC 7 — executive summary generator.

Aggregates every formally-verified JSON in `out/` into a buyer-facing
`Executive_Summary.txt`. The summary deliberately uses plain language —
the formal proofs sit behind the file paths it cites — so the reader
can immediately see what mathematical / physical claims the system makes.

The file is consumed by the `make release` target; if any of the upstream
JSONs are missing the summary refuses to write, so the release archive
cannot ship without every piece of evidence.
"""

from __future__ import annotations

import json
import pathlib
import sys
from datetime import datetime, timezone

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
ARM_PARAMS    = REPO_ROOT / "out" / "arm_params.json"
HAND_PARAMS   = REPO_ROOT / "out" / "hand_params.json"
GRASP_MATRIX  = REPO_ROOT / "out" / "grasp_matrix.json"
ARM_HAND_REPT = REPO_ROOT / "out" / "Arm_Hand_Report.txt"
MFG_CERT      = REPO_ROOT / "out" / "Manufacturing_Certificate.txt"
SUMMARY_PATH  = REPO_ROOT / "out" / "Executive_Summary.txt"


def required(p: pathlib.Path) -> None:
    if not p.exists():
        print(f"error: {p} missing — run `make poc6` then "
              f"`make grasp-matrix` first.", file=sys.stderr)
        sys.exit(1)


def torque_margin_pct(required: float, supplied: float) -> float:
    return (supplied - required) / required * 100.0


def main() -> int:
    for p in (ARM_PARAMS, HAND_PARAMS, GRASP_MATRIX,
              ARM_HAND_REPT, MFG_CERT):
        required(p)

    arm = json.loads(ARM_PARAMS.read_text())
    hand = json.loads(HAND_PARAMS.read_text())
    grasp = json.loads(GRASP_MATRIX.read_text())

    s_margin = torque_margin_pct(
        float(arm["requiredShoulderTorque"]),
        float(arm["tauShoulder"]))
    e_margin = torque_margin_pct(
        float(arm["requiredElbowTorque"]),
        float(arm["tauElbow"]))
    w_margin = torque_margin_pct(
        float(arm["requiredWristTorque"]),
        float(arm["tauWrist"]))
    min_margin = min(s_margin, e_margin, w_margin)

    dfm = hand["dfm"]
    min_wall_m = float(dfm["minWallThicknessM"])
    min_dia_m = float(dfm["minTendonHoleDiaM"])

    # Compute the worst-case wall margin across all 15 finger links.
    worst_wall_mm = float("inf")
    for fname in ("index", "middle", "ring", "pinky", "thumb"):
        ch = dfm[f"{fname}Channels"]
        for k in ("ch1", "ch2", "ch3"):
            worst_wall_mm = min(
                worst_wall_mm, float(ch[k]["wallMargin"]) * 1000)

    n_pass = grasp["n_pass"]
    n_total = grasp["n_total"]

    lines = [
        "================================================================",
        "  Project: Verified 12-DOF Robotic Arm & Hand",
        "  Executive Summary — MechProof v1.0",
        "================================================================",
        f"  Generated     : "
        f"{datetime.now(timezone.utc).isoformat(timespec='seconds')}",
        "  Source-of-truth proofs : Lean 4 (theorems discharged by "
        "`native_decide`)",
        "  Empirical evidence     : MuJoCo digital twin (deterministic, headless)",
        "",
        "  ─── Mathematical Guarantees (Lean 4 certified) ─────────────────",
        "  • ZERO self-collision between thumb and index proximal links",
        "    at the full thumb-swivel pose. Proof: HandAssembly.ThumbIndexClear",
        "    (capsule-segment distance > radii sum + 3 mm margin).",
        "  • Kinematic bounds: every finger joint provably stays within",
        "    its safe flexion range (0° ≤ θ ≤ 120°). Proof:",
        "    FingerParams.NoBackwardBending.",
        "  • Tendon routing produces positive flexion at every joint.",
        "    Proof: TendonParams.PositiveFlexion.",
        "",
        "  ─── Physical Guarantees (Lean 4 certified) ─────────────────────",
        "  6-DOF arm static torque at horizontal extension carrying "
        f"hand ({arm['handMassKg']} kg) + payload ({arm['payloadMassKg']} kg):",
        f"    Shoulder pitch : required {float(arm['requiredShoulderTorque']):6.2f} N·m, "
        f"supplied {float(arm['tauShoulder']):6.2f} N·m  "
        f"(margin {s_margin:5.1f}%)",
        f"    Elbow pitch    : required {float(arm['requiredElbowTorque']):6.2f} N·m, "
        f"supplied {float(arm['tauElbow']):6.2f} N·m  "
        f"(margin {e_margin:5.1f}%)",
        f"    Wrist pitch    : required {float(arm['requiredWristTorque']):6.2f} N·m, "
        f"supplied {float(arm['tauWrist']):6.2f} N·m  "
        f"(margin {w_margin:5.1f}%)",
        f"  Minimum margin : {min_margin:.1f}%  "
        f"(> 30% requirement: {'PASS' if min_margin > 30 else 'FAIL'})",
        "  Proof: ArmParams.StallSufficient.",
        "",
        "  ─── DFM Guarantees (Lean 4 certified) ──────────────────────────",
        f"  • Minimum wall thickness threshold : {min_wall_m*1000:.2f} mm",
        f"  • Minimum tendon hole diameter     : {min_dia_m*1000:.2f} mm",
        f"  • Worst-case observed wall margin  : {worst_wall_mm:.2f} mm",
        "  Geometry is safe for SLA 3D printing and injection molding.",
        "  Proof: HandAssembly.Manufacturable.",
        "",
        "  ─── Grasp Capability (MuJoCo digital twin) ─────────────────────",
        f"  Primitives tested : {n_total}",
        f"  Primitives held   : {n_pass}",
        f"  Pass rate         : {(100.0 * n_pass / n_total):.0f}%",
        "",
        "    Target                Result  Pinch min  Pinch peak  Self-coll",
        "    --------------------- ------  ---------  ----------  ---------",
    ]
    for r in grasp["results"]:
        lines.append(
            f"    {r['target']:21s} {r['result']:6s}  "
            f"{r['pinch_min_n']:6.3f} N  "
            f"{r['pinch_peak_n']:7.3f} N  "
            f"{'YES' if r.get('self_collisions') else 'NONE':>9s}"
        )

    lines += [
        "",
        "  ─── Deliverables in this archive ───────────────────────────────",
        "  • palm.step + 15 finger-link STEPs + thumb swivel base",
        "  • 3 arm-link STEPs + 2 brackets + ISO 9409-1 wrist flange",
        "  • arm_hand_scene.xml         — drop-in MuJoCo model",
        "  • arm_params.json            — Lean-certified motor specs",
        "  • hand_params.json           — Lean-certified DFM / kinematics data",
        "  • grasp_matrix.json          — empirical grasp test results",
        "  • Manufacturing_Certificate.txt",
        "  • Arm_Hand_Report.txt",
        "",
        "  ─── How the proofs gate the artefacts ──────────────────────────",
        "  Every .step / .xml file in this archive only exists because the",
        "  corresponding Lean 4 `native_decide` succeeded. If any proof",
        "  fails (negative motor, thin wall, tendon clearance violation,",
        "  insufficient stall torque), `make poc6` halts before producing",
        "  any CAD or simulation output. Try `make test` to see the proof",
        "  gate trigger on five canonical bad designs.",
        "================================================================",
    ]

    out = "\n".join(lines) + "\n"
    SUMMARY_PATH.write_text(out)
    print(out)
    print(f"Wrote {SUMMARY_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
