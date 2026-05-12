#!/usr/bin/env bash
# MechProof PoC 10 orchestrator.
# 1) Lean ZMP stability proof for the planned gait.   (fails → halt)
# 2) Python trajectory generator (walking_trajectory.json).
# 3) PoC 8 humanoid scene (drop-and-stand) — prerequisite for sim.
# 4) MuJoCo quasi-static walking simulation.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

echo "[1/4] Lean 4 ZMP Stability Verification..."
if ! ( lake build verify_walking && lake exe verify_walking ); then
  echo "Verification Failed: walking ZMP proof did not typecheck." >&2
  exit 1
fi

echo "[2/4] Generating Walking Trajectory..."
./venv/bin/python python/generate_walking_trajectory.py

echo "[3/4] Ensuring humanoid scene is up to date (PoC 8)..."
./run_poc8.sh > /dev/null

echo "[4/4] MuJoCo Quasi-Static Walking Simulation..."
./venv/bin/python python/simulate_walking.py

echo "Done! Walking verified."
ls -1 out/Walking_Report.txt out/walking_trajectory.json out/walking_proof.json
