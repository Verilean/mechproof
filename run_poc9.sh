#!/usr/bin/env bash
# MechProof PoC 9 orchestrator — full 30-DOF humanoid release.
#
# Stages:
#   1. PoC 6 — arm stall-torque proof + hand DFM/kinematics + arm-hand sim.
#   2. PoC 7 — grasp matrix (sphere/box/cylinder).
#   3. PoC 8 — leg balance/torque proof + humanoid drop-and-stand sim.
#   4. Humanoid executive summary (aggregates everything).
#   5. Bundle MechProof_Humanoid_v1.0.zip.
set -euo pipefail

cd "$(dirname "$0")"

# Stages 1-3 are full sub-pipelines; each halts the orchestrator on Lean
# proof failure thanks to its own `set -e`.
./run_poc6.sh

echo
echo "Running grasp matrix..."
./venv/bin/python python/simulate_grasp_matrix.py

./run_poc8.sh

echo
echo "Generating humanoid executive summary..."
./venv/bin/python python/generate_humanoid_summary.py

echo
echo "Building MechProof_Humanoid_v1.0 release archive..."
RELEASE_DIR=out/MechProof_Humanoid_v1.0
ARCHIVE=out/MechProof_Humanoid_v1.0.zip
rm -rf "$RELEASE_DIR" "$ARCHIVE"
mkdir -p "$RELEASE_DIR"

# CAD: every STEP file in out/.
cp out/*.step "$RELEASE_DIR/"

# Lean-emitted ground-truth specs.
cp out/arm_params.json   "$RELEASE_DIR/"
cp out/hand_params.json  "$RELEASE_DIR/"
cp out/leg_params.json   "$RELEASE_DIR/"
cp out/grasp_matrix.json "$RELEASE_DIR/"

# Digital twins (drop-in MuJoCo scenes).
cp out/humanoid_scene.xml "$RELEASE_DIR/"
cp out/arm_hand_scene.xml "$RELEASE_DIR/"

# Buyer-facing reports.
cp out/Humanoid_Executive_Summary.txt "$RELEASE_DIR/"
cp out/Manufacturing_Certificate.txt  "$RELEASE_DIR/"
cp out/Stand_Report.txt               "$RELEASE_DIR/"
cp out/Arm_Hand_Report.txt            "$RELEASE_DIR/"

./venv/bin/python -c "
import shutil
shutil.make_archive('out/MechProof_Humanoid_v1.0', 'zip',
                    root_dir='out', base_dir='MechProof_Humanoid_v1.0')
"

echo
echo "Done — MechProof_Humanoid_v1.0 release ready."
echo "Contents (${RELEASE_DIR}):"
ls -1 "$RELEASE_DIR" | sed 's/^/  - /'
echo
echo "Archive: $ARCHIVE  ($(du -h "$ARCHIVE" | cut -f1))"
