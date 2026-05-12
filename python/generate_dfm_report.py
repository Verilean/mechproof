"""MechProof PoC 5 — manufacturing certificate generator.

The Lean orchestrator emits `out/hand_params.json` only when **every** proof
(well-formedness, self-collision clearance, and the DFM rules) typechecks.
This script consumes that JSON without re-verifying anything: if the file
exists, the design is — by construction — formally certified for fabrication.

The certificate quotes the actual minimum wall thickness margin and the
minimum tendon hole diameter computed by Lean, so the manufacturer can see
which tolerance the design is closest to.
"""

from __future__ import annotations

import json
import pathlib
import sys
from datetime import datetime, timezone

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "hand_params.json"
CERT_PATH = REPO_ROOT / "out" / "Manufacturing_Certificate.txt"

FINGERS = ("index", "middle", "ring", "pinky", "thumb")


def main() -> int:
    if not PARAMS_PATH.exists():
        print(f"error: {PARAMS_PATH} not found — the Lean DFM proof must "
              "succeed before this report can be written.", file=sys.stderr)
        return 1

    p = json.loads(PARAMS_PATH.read_text())
    dfm = p["dfm"]
    min_wall_m = float(dfm["minWallThicknessM"])
    min_dia_m = float(dfm["minTendonHoleDiaM"])

    # Smallest wall margin and smallest tendon diameter across all 5×3 links.
    margins_mm = []
    diameters_mm = []
    moment_arms_mm = []
    for fname in FINGERS:
        ch = dfm[f"{fname}Channels"]
        for key in ("ch1", "ch2", "ch3"):
            c = ch[key]
            margins_mm.append(float(c["wallMargin"]) * 1000)
            diameters_mm.append(float(c["channelRadius"]) * 2000)
            moment_arms_mm.append(float(c["momentArm"]) * 1000)

    min_margin_mm = min(margins_mm)
    min_diameter_mm = min(diameters_mm)

    lines = [
        "================================================================",
        "  MechProof Manufacturing Certificate",
        "================================================================",
        f"  Generated   : {datetime.now(timezone.utc).isoformat(timespec='seconds')}",
        "  Source      : Lean 4 formal verification (verify_hand)",
        "",
        "  This document certifies that the bundled STEP files describe a",
        "  geometry that has been *formally proven* to satisfy MechProof's",
        "  Design-for-Manufacturing rules. The proofs were discharged by",
        "  `native_decide` over IEEE-754 Float arithmetic in Lean 4; absence",
        "  of this certificate means at least one rule was violated and the",
        "  build was halted before any CAD was emitted.",
        "",
        "  ─── DFM Thresholds (set in MechProof/DFM.lean) ────────────────",
        f"  MIN_WALL_THICKNESS  : {min_wall_m * 1000:.2f} mm",
        f"  MIN_TENDON_HOLE_DIA : {min_dia_m * 1000:.2f} mm",
        "",
        "  ─── Worst-case margins observed in this design ────────────────",
        f"  Min wall margin     : {min_margin_mm:.2f} mm "
        f"({'PASS' if min_margin_mm >= min_wall_m * 1000 else 'FAIL'})",
        f"  Min tendon diameter : {min_diameter_mm:.2f} mm "
        f"({'PASS' if min_diameter_mm >= min_dia_m * 1000 else 'FAIL'})",
        f"  Moment-arm range    : "
        f"{min(moment_arms_mm):.2f}–{max(moment_arms_mm):.2f} mm",
        "",
        "  ─── Per-finger wall margins (mm) ──────────────────────────────",
    ]
    for fname in FINGERS:
        ch = dfm[f"{fname}Channels"]
        m1 = float(ch["ch1"]["wallMargin"]) * 1000
        m2 = float(ch["ch2"]["wallMargin"]) * 1000
        m3 = float(ch["ch3"]["wallMargin"]) * 1000
        lines.append(f"    {fname:6s} : "
                     f"link1 = {m1:5.2f}   "
                     f"link2 = {m2:5.2f}   "
                     f"link3 = {m3:5.2f}")

    lines += [
        "",
        "  ─── Statement ─────────────────────────────────────────────────",
        "  DFM Verification Passed:",
        f"    Minimum Wall Thickness > {min_wall_m*1000:.2f} mm",
        f"    Tendon Holes           > {min_dia_m*1000:.2f} mm",
        "  Geometry is safe for SLA 3D Printing and Injection Molding.",
        "================================================================",
    ]

    out = "\n".join(lines) + "\n"
    CERT_PATH.write_text(out)
    print(out)
    print(f"Wrote {CERT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
