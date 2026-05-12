#!/usr/bin/env bash
# MechProof PoC 6 orchestrator.
# 1) Lean arm stall-torque verification.       (fails on weak motor → halt)
# 2) Lean hand DFM + kinematic verification.   (PoC 5 chain, prerequisite)
# 3) CadQuery: arm STEPs + hand STEPs.
# 4) MuJoCo combined arm+hand digital twin.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

echo "[1/4] Lean 4 Arm Torque Verification..."
if ! ( lake build verify_arm && lake exe verify_arm ); then
  echo "Verification Failed: arm stall-torque proof did not typecheck." >&2
  exit 1
fi

echo "[2/4] Lean 4 Hand DFM + Kinematics..."
if ! ( lake exe verify_hand ); then
  echo "Verification Failed: hand DFM/kinematic proof did not typecheck." >&2
  exit 1
fi

echo "[3/4] Generating Arm + Hand CAD..."
./venv/bin/python python/generate_arm_cad.py
./venv/bin/python python/generate_hand_cad.py
./venv/bin/python python/generate_dfm_report.py

echo "[4/4] MuJoCo Full-System Simulation (arm + hand + payload)..."
./venv/bin/python python/simulate_arm_hand.py

echo "Done! Full robotic system verified."
ls -1 out/ | sed 's/^/  - /'
