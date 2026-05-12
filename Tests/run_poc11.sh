#!/usr/bin/env bash
# MechProof PoC 11 orchestrator — standardisation + advanced balance + senses.
# 1) PoC 10 (Lean ZMP + walking sim) as prerequisite so the v2 scene has a
#    consistent humanoid model.
# 2) Lean Capture-Point proof (verify_capture).
# 3) Export URDF.
# 4) Headless teleop with head camera + IMU.
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p out

./Tests/run_poc10.sh

echo
echo "[5/8] Lean 4 Capture-Point Verification..."
if ! ( lake build verify_capture && lake exe verify_capture ); then
  echo "Verification Failed: Capture-Point proof did not typecheck." >&2
  exit 1
fi

# The URDF exporter needs arm + hand specs alongside the legs that
# poc10 already produced. Re-run the upstream proofs/generators here
# so a fresh `make clean && make poc11` works in one shot.
echo
echo "[5b/8] Lean 4 arm + hand re-verification (URDF prerequisites)..."
( lake exe verify_arm )
( lake exe verify_hand )
./venv/bin/python python/generate_hand_cad.py > /dev/null

echo
echo "[6/8] Generating URDF..."
./venv/bin/python python/export_urdf.py

echo
echo "[7/8] Running headless teleop (camera + IMU)..."
MUJOCO_GL=egl ./venv/bin/python python/simulate_v2.py

echo
echo "[8/8] Done! PoC 11 (v2) artefacts ready."
ls -1 out/mechproof_humanoid.urdf out/Teleop_Report.txt \
      out/imu_trace.json out/camera_*.png 2>/dev/null \
    | sed 's/^/  - /'
