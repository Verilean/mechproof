#!/usr/bin/env bash
# MechProof PoC 13 orchestrator — subsea construction upgrade.
# 1) PoC 8 humanoid scene as prerequisite.
# 2) Lean subsea integrity proofs (pressure / buoyancy / drag).
# 3) MuJoCo current-face simulation in seawater.
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p out

./Tests/run_poc8.sh > /dev/null

echo
echo "[Subsea 1/2] Lean 4 Subsea Integrity Verification..."
if ! ( lake build verify_subsea && lake exe verify_subsea ); then
  echo "Verification Failed: subsea integrity proof did not typecheck." >&2
  exit 1
fi

echo
echo "[Subsea 2/2] MuJoCo current-face simulation..."
./venv/bin/python python/simulate_subsea.py

echo
echo "Done! PoC 13 (subsea) artefacts:"
ls -1 out/subsea_params.json out/Subsea_Mission_Report.txt \
      out/humanoid_scene_subsea.xml out/subsea_torque_log.json 2>/dev/null \
    | sed 's/^/  - /'
