#!/usr/bin/env bash
# MechProof PoC 5 orchestrator.
# 1) Lean DFM + kinematic verification.   (fails on any DFM rule → halt)
# 2) Tendon-routed multi-part CAD (palm + 5 fingers + thumb swivel base).
# 3) Manufacturing certificate.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

echo "[1/3] Lean 4 DFM & Kinematic Verification..."
if ! ( lake build && lake exe verify_hand ); then
  echo "Verification Failed: DFM/kinematic proof did not typecheck." >&2
  exit 1
fi

echo "[2/3] Generating Certified CAD..."
./venv/bin/python python/generate_hand_cad.py

echo "[3/3] Generating Manufacturing Certificate..."
./venv/bin/python python/generate_dfm_report.py

echo "Done! Ready for physical manufacturing."
ls -1 out/ | sed 's/^/  - /'
