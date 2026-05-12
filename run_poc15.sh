#!/usr/bin/env bash
# MechProof PoC 15 orchestrator — 4 m heavy-machinery scale-up.
# 1) PoC 8 baseline (so leg_params.json exists).
# 2) Lean heavy proofs (torque + yield stress).
# 3) MuJoCo scaled stand-firm test.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

./run_poc8.sh > /dev/null

echo
echo "[Heavy 1/2] Lean 4 Heavy-Machinery Verification..."
if ! ( lake build verify_heavy && lake exe verify_heavy ); then
  echo "Verification Failed: scale-up exceeds actuator stall or yield strength." >&2
  exit 1
fi

echo
echo "[Heavy 2/2] MuJoCo 4 m stand-firm simulation..."
./venv/bin/python python/simulate_heavy.py

echo
echo "Done! PoC 15 (heavy machinery) artefacts:"
ls -1 out/heavy_params.json out/Heavy_Construction_Catalog.txt \
      out/humanoid_scene_heavy.xml out/heavy_grf_log.json 2>/dev/null \
    | sed 's/^/  - /'
