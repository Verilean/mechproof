#!/usr/bin/env bash
# MechProof PoC 7 orchestrator.
# 1) PoC 6 full pipeline (verifies + builds every upstream artefact).
# 2) MuJoCo grasp matrix (sphere / box / cylinder).
# 3) Executive summary.
# 4) Release zip (Verified_Robotic_System_v1.0.zip).
set -euo pipefail

cd "$(dirname "$0")"

./run_poc6.sh

echo
echo "[5/6] Running MuJoCo grasp matrix..."
./venv/bin/python python/simulate_grasp_matrix.py

echo
echo "[6/6] Generating executive summary..."
./venv/bin/python python/generate_executive_summary.py

echo
echo "Building release archive..."
RELEASE_DIR=out/release_v1.0
ARCHIVE=out/Verified_Robotic_System_v1.0.zip
rm -rf "$RELEASE_DIR" "$ARCHIVE"
mkdir -p "$RELEASE_DIR"

cp out/*.step                        "$RELEASE_DIR/"
cp out/arm_hand_scene.xml            "$RELEASE_DIR/"
cp out/Executive_Summary.txt         "$RELEASE_DIR/"
cp out/Manufacturing_Certificate.txt "$RELEASE_DIR/"
cp out/Arm_Hand_Report.txt           "$RELEASE_DIR/"
cp out/arm_params.json               "$RELEASE_DIR/"
cp out/hand_params.json              "$RELEASE_DIR/"
cp out/grasp_matrix.json             "$RELEASE_DIR/"

./venv/bin/python -c "
import shutil
shutil.make_archive('out/Verified_Robotic_System_v1.0', 'zip',
                    root_dir='out', base_dir='release_v1.0')
"

echo "Wrote $ARCHIVE"
echo "Contents:"
ls -1 "$RELEASE_DIR" | sed 's/^/  - /'
echo "Archive size: $(du -h "$ARCHIVE" | cut -f1)"
