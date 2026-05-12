#!/usr/bin/env bash
# MechProof PoC 8 orchestrator.
# 1) Lean lower-body balance + squat-torque proof.   (fails → halt)
# 2) CadQuery: torso, thigh, shin, foot STEPs.
# 3) MuJoCo drop-and-stand simulation.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

echo "[1/3] Lean 4 Lower-Body Balance + Torque Verification..."
if ! ( lake build verify_legs && lake exe verify_legs ); then
  echo "Verification Failed: balance / squat-torque proof did not typecheck." >&2
  exit 1
fi

echo "[2/3] Generating Torso + Leg CAD..."
./venv/bin/python python/generate_leg_cad.py

echo "[3/3] MuJoCo Drop-and-Stand Simulation..."
./venv/bin/python python/simulate_stand.py

echo "Done! Humanoid lower body verified."
ls -1 out/ | sed 's/^/  - /'
