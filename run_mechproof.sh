#!/usr/bin/env bash
# MechProof PoC 1 orchestrator.
# 1) Run the Lean proof. If `lake build` fails, the proof fails — halt.
# 2) Run the Lean exe to emit `out/verified_params.json`.
# 3) Run CadQuery to materialise `out/verified_case.step`.
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p out

echo "[1/2] Lean verification…"
if ! ( lake build && lake exe mechproof ); then
  echo "Verification Failed: Geometry is not moldable." >&2
  exit 1
fi

echo "[2/2] CadQuery STEP generation…"
./venv/bin/python python/generate_cad.py

echo "Done. Artefact: out/verified_case.step"
