#!/usr/bin/env bash
# MechProof PoC 4 orchestrator.
# 1) Lean full-hand collision verification.   (fails → halt)
# 2) CadQuery: palm + 5 finger sets + thumb swivel base.
# 3) MuJoCo 6-DOF pinch simulation.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

echo "[1/3] Lean 4 Full Hand Collision Verification..."
if ! ( lake build && lake exe verify_hand ); then
  echo "Verification Failed: Hand self-collision proof did not typecheck." >&2
  exit 1
fi

echo "[2/3] Generating Palm and 5-Finger CAD..."
./venv/bin/python python/generate_hand_cad.py

echo "[3/3] MuJoCo 6-DOF Pinch Simulation..."
./venv/bin/python python/simulate_hand.py

echo "Done! 6-DOF Hand verified."
ls -1 out/ | sed 's/^/  - /'
