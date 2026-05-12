#!/usr/bin/env bash
# MechProof PoC 14 orchestrator — universal hardware compiler.
# 1) PoC 13 (subsea) as prerequisite so all upstream artefacts exist.
# 2) Lean env-matrix proof (every shipping env passes).
# 3) Environment_Matrix.txt report.
# 4) Per-environment release ZIPs + master catalog ZIP.
set -euo pipefail

cd "$(dirname "$0")/.."
mkdir -p out

# PoC 12 builds arm + hand + legs + URDF + energy artefacts. PoC 13 is
# layered on top here so the catalog gets subsea-specific outputs too.
./Tests/run_poc12.sh > /dev/null
./Tests/run_poc13.sh > /dev/null

echo
echo "[14a] Lean 4 environment-matrix verification..."
if ! ( lake build verify_env_matrix && lake exe verify_env_matrix ); then
  echo "Verification Failed: one of the shipping environments is unsafe." >&2
  exit 1
fi

echo
echo "[14b] Generating Environment_Matrix.txt..."
./venv/bin/python python/generate_env_matrix.py

# ── Per-environment release bundling ────────────────────────────────
# Each ZIP carries the same CAD + Lean JSON outputs as the v1.0 release
# plus an environment-specific manifest pointing at the row in
# Environment_Matrix.txt that applies. The customer reading the zip
# never sees Lean — they see a coherent ROS-2-ready package per
# operating environment.

CATALOG_DIR=out/MechProof_Catalog_v1.0
CATALOG_ZIP=out/MechProof_Catalog_v1.0.zip
rm -rf "$CATALOG_DIR" "$CATALOG_ZIP"
mkdir -p "$CATALOG_DIR"

for env in factory_air subsea_500m lunar mars; do
  variant_dir="$CATALOG_DIR/MechProof_${env}_v1.0"
  mkdir -p "$variant_dir"

  # Shared artefacts.
  cp out/*.step                         "$variant_dir/"
  cp out/arm_params.json                "$variant_dir/"
  cp out/hand_params.json               "$variant_dir/"
  cp out/leg_params.json                "$variant_dir/"
  cp out/grasp_matrix.json              "$variant_dir/" 2>/dev/null || true
  cp out/Manufacturing_Certificate.txt  "$variant_dir/" 2>/dev/null || true
  cp out/Environment_Matrix.txt         "$variant_dir/"
  cp out/env_matrix.json                "$variant_dir/"

  # Environment-specific extras.
  case "$env" in
    subsea_500m)
      cp out/subsea_params.json            "$variant_dir/"
      cp out/humanoid_scene_subsea.xml     "$variant_dir/"
      cp out/Subsea_Mission_Report.txt     "$variant_dir/" 2>/dev/null || true
      ;;
    lunar|mars)
      # No special scene yet; the URDF + Lean JSONs are env-agnostic.
      cp out/humanoid_scene.xml            "$variant_dir/"
      cp out/Stand_Report.txt              "$variant_dir/" 2>/dev/null || true
      ;;
    factory_air|*)
      cp out/humanoid_scene.xml            "$variant_dir/"
      cp out/arm_hand_scene.xml            "$variant_dir/" 2>/dev/null || true
      cp out/Stand_Report.txt              "$variant_dir/" 2>/dev/null || true
      cp out/Arm_Hand_Report.txt           "$variant_dir/" 2>/dev/null || true
      ;;
  esac

  # URDF if present.
  cp out/mechproof_humanoid.urdf "$variant_dir/" 2>/dev/null || true

  # Variant manifest.
  cat > "$variant_dir/Manifest.txt" <<EOF
MechProof Humanoid v1.0  —  ${env} variant
Environment: ${env}
Verified by: Lean 4 (\`verify_env_matrix\`)

This package is a regulated derivative of the canonical MechProof
30-DOF humanoid. The Environment_Matrix.txt in the same directory
documents how this variant's row was certified.
EOF

done

./venv/bin/python -c "
import shutil
shutil.make_archive('out/MechProof_Catalog_v1.0', 'zip',
                    root_dir='out', base_dir='MechProof_Catalog_v1.0')
"

echo
echo "Done! PoC 14 catalog ready."
echo "Variants:"
for v in "$CATALOG_DIR"/*/; do
  n=$(ls "$v" | wc -l)
  echo "  $(basename "$v")  ($n files)"
done
echo "Master archive: $CATALOG_ZIP  ($(du -h "$CATALOG_ZIP" | cut -f1))"
