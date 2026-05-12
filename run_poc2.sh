#!/usr/bin/env bash
# MechProof PoC 2 orchestrator.
# 1) Lean kinematic verification.            (fails → halt, nothing else runs)
# 2) Multi-part CAD generation via CadQuery. (link1/2/3.step + physics_meta.json)
# 3) MuJoCo digital-twin simulation.         (finger.xml + Verification_Report.txt)
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

echo "[1/3] Lean 4 Kinematic Verification..."
if ! ( lake build && lake exe verify_finger ); then
  echo "Verification Failed: Finger kinematics are not provably safe." >&2
  exit 1
fi

echo "[2/3] Generating Multi-Part CAD (STEP)..."
./venv/bin/python python/generate_finger_cad.py

echo "[3/3] Generating MuJoCo Digital Twin and Running Simulation..."
./venv/bin/python python/simulate_finger.py

echo "Success! All artifacts are in out/"
ls -1 out/ | sed 's/^/  - /'
