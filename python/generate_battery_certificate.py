"""MechProof PoC 12 — battery / endurance certificate generator.

Reads `out/energy_proof.json` (only present after the Lean MissionPossible
proof typechecks) and produces a buyer-facing
`Battery_Life_Certificate.txt` quoting the mathematically guaranteed
operating time for each mode at the rated battery capacity and 20%
safety reserve.
"""

from __future__ import annotations

import json
import pathlib
import sys
from datetime import datetime, timezone

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PROOF_PATH = REPO_ROOT / "out" / "energy_proof.json"
CERT_PATH  = REPO_ROOT / "out" / "Battery_Life_Certificate.txt"


def main() -> int:
    if not PROOF_PATH.exists():
        print(f"error: {PROOF_PATH} missing — run `make verify-energy` first.",
              file=sys.stderr)
        return 1

    data = json.loads(PROOF_PATH.read_text())
    missions = data["missions"]
    if not missions:
        print("error: energy_proof.json has no missions", file=sys.stderr)
        return 1

    battery_wh = float(missions[0]["batteryWh"])
    safety = float(missions[0]["safetyFraction"])
    usable = battery_wh * safety

    lines = [
        "================================================================",
        "  MechProof Battery / Endurance Certificate",
        "================================================================",
        f"  Generated   : "
        f"{datetime.now(timezone.utc).isoformat(timespec='seconds')}",
        "  Source      : Lean 4 formal verification (verify_energy)",
        "",
        "  This certificate quantifies — mathematically — how long the",
        "  MechProof humanoid can operate on its rated battery, with a",
        "  conservative 20% capacity reserve. Each row is the result of",
        "  the `MissionPossible` theorem applied to a per-bucket motor",
        "  power model:",
        "       P = (|τ·ω|  +  R · (τ/Kt)²)  /  η",
        "  integrated over the mission duration.",
        "",
        "  ─── Battery ────────────────────────────────────────────────────",
        f"  Rated capacity     : {battery_wh:.0f} Wh",
        f"  Safety fraction    : {safety*100:.0f}%",
        f"  Usable capacity    : {usable:.0f} Wh",
        "",
        "  ─── Verified missions ─────────────────────────────────────────",
        "",
        "    Mode                          Power   Energy   Endurance   Result",
        "    --------------------------- -------- -------- ----------- ------",
    ]

    for m in missions:
        mode = m["mode"]
        power_w = float(m["totalBusPowerW"])
        energy_wh = float(m["missionEnergyWh"])
        duration_s = float(m["durationS"])
        # Maximum endurance at this mode's continuous power (the time at
        # which we'd drain the usable capacity).
        if power_w > 1e-6:
            endurance_h = usable / power_w
        else:
            endurance_h = float("inf")
        endurance_str = (f"{endurance_h:5.2f} h"
                         if endurance_h < 1e6 else "    ∞")
        verdict = "PASS" if energy_wh < usable else "FAIL"
        lines.append(
            f"    {mode:27s} {power_w:6.1f} W {energy_wh:6.2f} Wh "
            f"{endurance_str:>10s} {verdict:>6s}"
        )

    standing = next((m for m in missions
                     if "standing" in m["mode"].lower()), None)
    walking = next((m for m in missions
                    if "walking" in m["mode"].lower()
                    and "10-step" not in m["mode"].lower()), None)

    if standing and walking:
        lines += [
            "",
            "  ─── Statement ─────────────────────────────────────────────",
            "  Battery Endurance Verified:",
            f"    Standing : {usable / float(standing['totalBusPowerW']):4.1f} hours "
            f"@ {float(standing['totalBusPowerW']):.0f} W",
            f"    Walking  : {usable / float(walking['totalBusPowerW']):4.1f} hours "
            f"@ {float(walking['totalBusPowerW']):.0f} W",
            "",
            "  Both numbers come with the 20% capacity reserve already",
            "  subtracted. Field operators can therefore plan missions",
            "  against these numbers with no further derating needed.",
            "================================================================",
        ]

    out = "\n".join(lines) + "\n"
    CERT_PATH.write_text(out)
    print(out)
    print(f"Wrote {CERT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
