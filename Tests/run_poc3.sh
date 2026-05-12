#!/usr/bin/env bash
# MechProof PoC 3 orchestrator.
# 1) Lean tendon-routing verification.       (fails → halt)
# 2) CadQuery: links with tendon channels + physics_meta.json.
# 3) MuJoCo grasp simulation against a cylinder target.
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p out

echo "[1/3] Lean 4 Tendon Verification..."
if ! ( lake build && lake exe verify_tendon ); then
  echo "Verification Failed: Tendon routing is not provably valid." >&2
  exit 1
fi

echo "[2/3] Generating Tendon-Routed CAD (STEP)..."
./venv/bin/python python/generate_tendon_cad.py

echo "[3/3] Running MuJoCo Grasp Simulation..."
./venv/bin/python python/simulate_grasp.py

echo "Done! Grasp Report generated."
ls -1 out/ | sed 's/^/  - /'
