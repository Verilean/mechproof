"""MechProof PoC 1 — CAD generator.

Consumes the verified parameters emitted by the Lean proof and produces a STEP
file of a drafted, hollow case. The geometry is built by lofting the bottom
rectangle to a slightly larger top rectangle, so each side wall slopes outward
by exactly `draftDeg` from the +Z pull axis — the very condition Lean proved.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import cadquery as cq

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "verified_params.json"
STEP_PATH = REPO_ROOT / "out" / "verified_case.step"


def main() -> int:
    if not PARAMS_PATH.exists():
        print(f"error: {PARAMS_PATH} not found — run Lean verification first.",
              file=sys.stderr)
        return 1

    p = json.loads(PARAMS_PATH.read_text())
    L = float(p["length"])
    W = float(p["width"])
    H = float(p["height"])
    T = float(p["thickness"])
    D = float(p["draftDeg"])

    delta = H * math.tan(math.radians(D))

    case = (
        cq.Workplane("XY")
        .rect(L, W)
        .workplane(offset=H)
        .rect(L + 2 * delta, W + 2 * delta)
        .loft(combine=True)
        .faces(">Z")
        .shell(-T)
    )

    STEP_PATH.parent.mkdir(parents=True, exist_ok=True)
    solid = case.val()
    solid.exportStep(str(STEP_PATH))
    print(f"Wrote {STEP_PATH}  (volume={solid.Volume():.1f} mm^3)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
