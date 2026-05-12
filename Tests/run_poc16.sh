#!/usr/bin/env bash
# MechProof PoC 16 orchestrator — manned-mech safety.
# 1) PoC 15 (heavy 4 m mech scene) as prerequisite.
# 2) Lean safety proofs (override + survival brace).
# 3) MuJoCo two-phase simulation (reckless input + controlled drop).
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p out

./Tests/run_poc15.sh > /dev/null

echo
echo "[Safety 1/2] Lean 4 Manned-Mech Safety Verification..."
if ! ( lake build verify_safety && lake exe verify_safety ); then
  echo "Verification Failed: pilot safety (override or brace) did not typecheck." >&2
  exit 1
fi

echo
echo "[Safety 2/2] MuJoCo manned-safety simulation..."
./venv/bin/python python/simulate_manned.py

echo
echo "Done! PoC 16 (manned safety) artefacts:"
ls -1 out/safety_params.json out/Manned_Safety_Report.txt \
      out/cockpit_g_force.json 2>/dev/null \
    | sed 's/^/  - /'
