#!/usr/bin/env bash
# MechProof PoC 12 orchestrator — power consumption + endurance.
# 1) PoC 11 (Capture-Point + URDF + teleop scene) as prerequisite.
# 2) Lean energy proof (verify_energy).
# 3) Battery / endurance certificate.
# 4) v2 teleop sim with per-step energy logging → energy_profile.json.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

./run_poc11.sh

echo
echo "[9/12] Lean 4 Energy / Mission Verification..."
if ! ( lake build verify_energy && lake exe verify_energy ); then
  echo "Verification Failed: mission energy exceeds the battery's reserve." >&2
  exit 1
fi

echo
echo "[10/12] Generating Battery / Endurance Certificate..."
./venv/bin/python python/generate_battery_certificate.py

echo
echo "[11/12] Running v2 teleop with energy logging..."
MUJOCO_GL=egl ./venv/bin/python python/simulate_v2.py

echo
echo "[12/12] Done! PoC 12 artefacts:"
ls -1 out/energy_proof.json out/Battery_Life_Certificate.txt \
      out/energy_profile.json out/Teleop_Report.txt 2>/dev/null \
    | sed 's/^/  - /'
