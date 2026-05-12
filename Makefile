# MechProof Makefile.
#
#   make <target>
#
# All targets transparently invoke the project's `nix-shell` so the user never
# has to remember to enter it manually.  Inside each recipe we run a single
# command via `$(NIX_RUN)` which wraps `nix-shell --run …`.
#
# Targets fall into four groups:
#   * PoC end-to-end (poc1..poc5)
#   * Per-step helpers (lean, hand-cad, hand-sim, cert, …)
#   * Negative tests (test-dfm-wall, test-dfm-hole, test-collision,
#       test-tendon, test-moldability) — each mutates a Lean source file,
#       checks the build fails, and restores it.
#   * Housekeeping (clean, shell, build, help, all)

REPO       := $(CURDIR)
LEAN_DIR   := $(REPO)
OUT_DIR    := $(REPO)/out
VENV_PY    := $(REPO)/venv/bin/python

# Literal comma so we can pass commas through `$(call ...)` without it
# being parsed as an argument separator.
comma      := ,

# Run an argument inside the project's nix-shell. Bash is used so the
# heredocs and `set -e` semantics inside negative tests behave predictably.
NIX_RUN    := nix-shell $(REPO)/shell.nix --run

# MuJoCo's offscreen renderer (`mujoco.Renderer`) only runs PoC 11's
# `simulate_v2.py`, which uses MuJoCo's own head-camera. We default to
# EGL; on the rare CI box without an EGL device, set MUJOCO_GL=osmesa.
# The standalone scene-preview renderer (`make preview`) does **not**
# use MuJoCo's GL stack — it routes through WebGPU instead.
MUJOCO_GL ?= egl

.DEFAULT_GOAL := help

.PHONY: help all clean shell build \
        poc1 poc2 poc3 poc4 poc5 poc6 poc7 poc8 poc9 poc10 poc11 poc12 poc13 poc14 poc15 poc16 \
        lean hand-cad hand-sim cert arm-cad full-sim \
        grasp-matrix summary release release-arm-hand release-all humanoid-summary \
        leg-cad humanoid-sim \
        walking-trajectory walk-sim urdf teleop \
        heavy-sim safety-sim preview \
        verify-finger verify-tendon verify-hand verify-arm verify-legs \
        verify-walking verify-capture verify-energy verify-subsea \
        verify-env-matrix verify-heavy verify-safety verify-all env-matrix \
        battery-cert energy-sim subsea-sim \
        finger-cad tendon-cad \
        finger-sim grasp-sim \
        test test-moldability test-tendon test-collision \
        test-dfm-wall test-dfm-hole test-torque test-balance test-zmp \
        test-capture test-energy test-crush test-square-cube test-lethal-crash

# ───────────────────────────────────────────────────────────────────
#  help / housekeeping
# ───────────────────────────────────────────────────────────────────

help: ## Show this help.
	@awk 'BEGIN{FS=":.*## "; printf "MechProof targets:\n\n"} \
	      /^[a-zA-Z0-9_-]+:.*## /{printf "  %-22s %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo
	@echo "Common flows:"
	@echo "  make poc16           # manned-mech safety (override + crash brace)"
	@echo "  make poc15           # 4 m / 2.3 t heavy-machinery scale-up"
	@echo "  make poc14           # multi-env catalog (factory / subsea / lunar / mars)"
	@echo "  make verify-all      # every Lean proof across every env + scale + safety"
	@echo "  make safety-sim      # manned-mech sim only"
	@echo "  make test            # run every negative test (13 cases)"
	@echo "  make clean           # wipe generated artefacts in out/"

shell: ## Drop into the project's nix-shell.
	nix-shell $(REPO)/shell.nix

build: ## Build all Lean executables (no run).
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build"

clean: ## Remove every generated artefact in out/.
	rm -rf $(OUT_DIR)/*

all: release ## Alias for the most-current end-to-end pipeline (PoC 9 humanoid release).

# ───────────────────────────────────────────────────────────────────
#  Per-step helpers
# ───────────────────────────────────────────────────────────────────

lean: build ## Rebuild every Lean target.

verify-finger: ## Run the PoC 2 finger kinematic proof and emit JSON.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_finger && lake exe verify_finger"

verify-tendon: ## Run the PoC 3 tendon proof and emit JSON.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_tendon && lake exe verify_tendon"

verify-hand: ## Run the PoC 4/5 hand proof (kinematics + collision + DFM).
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_hand && lake exe verify_hand"

verify-arm: ## PoC 6: prove the arm motors can hold hand+payload at horizontal pose.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_arm && lake exe verify_arm"

verify-legs: ## PoC 8: prove static balance + knee squat-torque sufficient.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_legs && lake exe verify_legs"

verify-walking: ## PoC 10: prove the planned ZMP trajectory stays inside the support polygon.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_walking && lake exe verify_walking"

verify-capture: ## PoC 11: prove every keyframe is within its capture region.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_capture && lake exe verify_capture"

verify-energy: ## PoC 12: prove standing + walking + 10-step missions fit within 80% battery.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_energy && lake exe verify_energy"

verify-subsea: ## PoC 13: prove pressure clearance + buoyancy + current drag at 500 m depth.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_subsea && lake exe verify_subsea"

verify-env-matrix: ## PoC 14: prove every shipping environment passes the standard checks.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_env_matrix && lake exe verify_env_matrix"

verify-heavy: ## PoC 15: prove the 4 m / 2.3 t robot survives torque + yield at scale.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_heavy && lake exe verify_heavy"

verify-safety: ## PoC 16: prove the pilot survives reckless input + a 1 m crash.
	$(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_safety && lake exe verify_safety"

verify-all: verify-finger verify-tendon verify-hand verify-arm verify-legs \
            verify-walking verify-capture verify-energy verify-subsea \
            verify-env-matrix verify-heavy verify-safety ## Run every Lean verify_* exe.
	@echo "All Lean proofs typechecked across every shipping environment + scale + safety."

finger-cad: ## PoC 2 multi-part finger CAD.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_finger_cad.py"

tendon-cad: ## PoC 3 tendon-routed finger CAD.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_tendon_cad.py"

hand-cad: ## PoC 4/5 full-hand CAD (consumes hand_params.json).
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_hand_cad.py"

arm-cad: ## PoC 6 arm STEPs (links, brackets, ISO 9409-1 wrist flange).
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_arm_cad.py"

leg-cad: ## PoC 8 torso + thigh/shin/foot STEPs.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_leg_cad.py"

finger-sim: ## PoC 2 single-finger MuJoCo sim.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_finger.py"

grasp-sim: ## PoC 3 tendon-driven grasp sim.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_grasp.py"

hand-sim: ## PoC 4 6-DOF hand pinch sim.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_hand.py"

full-sim: ## PoC 6 combined arm+hand pinch with 2 kg payload.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_arm_hand.py"

humanoid-sim: ## PoC 8 drop-and-stand simulation of the full lower body.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_stand.py"

walking-trajectory: ## PoC 10 keyframe trajectory generator.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_walking_trajectory.py"

walk-sim: ## PoC 10 quasi-static walking simulation.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_walking.py"

urdf: ## PoC 11 export the humanoid as a single ROS-2 URDF file.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/export_urdf.py"

teleop: ## PoC 11 headless teleop with head camera + IMU + PNG snapshots.
	$(NIX_RUN) "MUJOCO_GL=$(MUJOCO_GL) $(VENV_PY) $(REPO)/python/simulate_v2.py"

battery-cert: ## PoC 12 Battery_Life_Certificate.txt (consumes verify-energy JSON).
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_battery_certificate.py"

energy-sim: ## PoC 12 teleop sim with per-step energy integration → energy_profile.json.
	$(NIX_RUN) "MUJOCO_GL=$(MUJOCO_GL) $(VENV_PY) $(REPO)/python/simulate_v2.py"

subsea-sim: ## PoC 13 MuJoCo current-face simulation in seawater.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_subsea.py"

env-matrix: ## PoC 14 Environment_Matrix.txt report (consumes Lean env_matrix.json).
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_env_matrix.py"

heavy-sim: ## PoC 15 MuJoCo 4 m stand-firm simulation + Heavy_Construction_Catalog.txt.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_heavy.py"

safety-sim: ## PoC 16 manned-mech safety sim (override + crash brace).
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_manned.py"

preview: ## Render every generated MuJoCo scene to PNGs via WebGPU.
	$(NIX_RUN) "WGPU_BACKEND_TYPE=Vulkan $(VENV_PY) $(REPO)/python/render_overviews.py"

grasp-matrix: ## PoC 7 grasp-matrix sim (sphere/box/cylinder → grasp_matrix.json).
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/simulate_grasp_matrix.py"

summary: ## PoC 7 arm-hand executive summary report.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_executive_summary.py"

humanoid-summary: ## PoC 9 full humanoid executive summary.
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_humanoid_summary.py"

release-arm-hand: ## PoC 7 archive: Verified_Robotic_System_v1.0.zip (arm + hand only).
	$(NIX_RUN) "$(REPO)/Tests/run_poc7.sh"

release: ## PoC 9 archive: MechProof_Humanoid_v1.0.zip (full 30-DOF system).
	$(NIX_RUN) "$(REPO)/Tests/run_poc9.sh"

cert: ## PoC 5 manufacturing certificate (consumes Lean DFM JSON).
	$(NIX_RUN) "$(VENV_PY) $(REPO)/python/generate_dfm_report.py"

# ───────────────────────────────────────────────────────────────────
#  PoC end-to-end orchestrators
# ───────────────────────────────────────────────────────────────────

poc1: ## PoC 1: drafted moldable case.
	$(NIX_RUN) "$(REPO)/Tests/run_mechproof.sh"

poc2: ## PoC 2: kinematic finger + MuJoCo digital twin.
	$(NIX_RUN) "$(REPO)/Tests/run_poc2.sh"

poc3: ## PoC 3: tendon-driven grasping.
	$(NIX_RUN) "$(REPO)/Tests/run_poc3.sh"

poc4: ## PoC 4: full 6-DOF hand + collision proof.
	$(NIX_RUN) "$(REPO)/Tests/run_poc4.sh"

poc5: ## PoC 5: DFM-certified hand (the headline pipeline).
	$(NIX_RUN) "$(REPO)/Tests/run_poc5.sh"

poc6: ## PoC 6: 6-DOF arm + 5-finger hand carrying a 2 kg payload.
	$(NIX_RUN) "$(REPO)/Tests/run_poc6.sh"

poc7: release ## PoC 7: grasp matrix + executive summary + release archive.

poc8: ## PoC 8: humanoid lower-body balance + drop-and-stand.
	$(NIX_RUN) "$(REPO)/Tests/run_poc8.sh"

poc9: release ## PoC 9: full 30-DOF humanoid release archive.

poc10: ## PoC 10: dynamic walking (ZMP proof + quasi-static gait).
	$(NIX_RUN) "$(REPO)/Tests/run_poc10.sh"

poc11: ## PoC 11: Capture-Point proof + URDF + headless teleop.
	$(NIX_RUN) "$(REPO)/Tests/run_poc11.sh"

poc12: ## PoC 12: energy proof + Battery_Life_Certificate + power-aware teleop.
	$(NIX_RUN) "$(REPO)/Tests/run_poc12.sh"

poc13: ## PoC 13: subsea pressure / buoyancy / drag proofs + seawater sim.
	$(NIX_RUN) "$(REPO)/Tests/run_poc13.sh"

poc14: ## PoC 14: env-matrix proof + per-env release ZIPs + master catalog.
	$(NIX_RUN) "$(REPO)/Tests/run_poc14.sh"

poc15: ## PoC 15: 4 m heavy-machinery scale-up (hydraulics + steel).
	$(NIX_RUN) "$(REPO)/Tests/run_poc15.sh"

poc16: ## PoC 16: piloted-mech safety proofs + override/crash sim.
	$(NIX_RUN) "$(REPO)/Tests/run_poc16.sh"

release-all: poc14 ## Alias for the multi-environment master catalog.

# ───────────────────────────────────────────────────────────────────
#  Negative tests
#
#  Each test mutates a Lean source file via `sed`, runs `make poc<N>` and
#  asserts that the build *fails*, then restores the file from backup.
#  The bash idiom `if ...; then echo FAIL; ...; fi` is used so that the
#  expected non-zero exit code from the orchestrator is captured rather
#  than aborting the recipe.
# ───────────────────────────────────────────────────────────────────

# Expect a Lean build to fail.  Usage: $(call expect_fail,<file>,<sed-script>,<lake-cmd>)
# `sed-script` is passed verbatim to `sed -i`; restoration uses .bak.
# Args are NOT split across lines so make does not introduce stray whitespace.
expect_fail = @set -e; FILE="$(1)"; SCRIPT='$(2)'; CMD='$(3)'; \
cp "$$FILE" "$$FILE.bak"; \
trap 'mv "$$FILE.bak" "$$FILE"' EXIT INT TERM; \
sed -i "$$SCRIPT" "$$FILE" || { echo "[negative-test] FAIL — sed could not apply '$$SCRIPT'"; exit 1; }; \
if ! cmp -s "$$FILE" "$$FILE.bak"; then \
    echo "[negative-test] mutated $$FILE with: $$SCRIPT"; \
else \
    echo "[negative-test] FAIL — sed matched nothing (pattern not found)"; exit 1; \
fi; \
if $(NIX_RUN) "$$CMD" >/tmp/mp-neg.log 2>&1; then \
    echo "[negative-test] FAIL — Lean did NOT halt the build."; \
    tail -20 /tmp/mp-neg.log; \
    exit 1; \
else \
    echo "[negative-test] PASS — Lean correctly rejected the bad design."; \
fi

test: test-moldability test-tendon test-collision test-dfm-wall test-dfm-hole test-torque test-balance test-zmp test-capture test-energy test-crush test-square-cube test-lethal-crash ## Run every negative test.
	@echo "All negative tests passed: Lean correctly rejected every bad design."

test-moldability: ## PoC 1: setting draftDeg = -1° must fail.
	$(call expect_fail,$(LEAN_DIR)/Tests/Main.lean,s|draftDeg := 2.0|draftDeg := -1.0|,cd $(LEAN_DIR) && lake build mechproof)

test-tendon: ## PoC 3: a negative tendon moment arm must fail.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyTendon.lean,s|r2 := 2.5|r2 := -1.0|,cd $(LEAN_DIR) && lake build verify_tendon)

test-collision: ## PoC 4: thumb mounted on top of index must fail clearance.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyHand.lean,s|px :=  0.045$(comma) py := 0.020|px :=  0.026$(comma) py := 0.055|,cd $(LEAN_DIR) && lake build verify_hand)

test-dfm-wall: ## PoC 5: a 2 mm finger fails the MIN_WALL_THICKNESS rule.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyHand.lean,s|thickness := 10.0|thickness := 2.0|,cd $(LEAN_DIR) && lake build verify_hand)

test-dfm-hole: ## PoC 5: a 0.6 mm-diameter tendon hole fails MIN_TENDON_HOLE_DIA.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyHand.lean,s|channelRadius := 0.0006|channelRadius := 0.0003|g,cd $(LEAN_DIR) && lake build verify_hand)

test-torque: ## PoC 6: a 5 N·m shoulder stall torque cannot hold the payload.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyArm.lean,s|tauShoulder := 30.0|tauShoulder := 5.0|,cd $(LEAN_DIR) && lake build verify_arm)

test-balance: ## PoC 8: a 20 mm-long foot shrinks the support polygon below the balance margin.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyLegs.lean,s|footLen     := 0.16|footLen     := 0.02|,cd $(LEAN_DIR) && lake build verify_legs)

test-zmp: ## PoC 10: an over-aggressive accel (5 m/s²) drives the ZMP outside the foot.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyWalking.lean,s|accX := 0.0|accX := 5.0|g,cd $(LEAN_DIR) && lake build verify_walking)

test-capture: ## PoC 11: a 5 m/s forward velocity flies the capture point outside the next foot.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyCapture.lean,s|velY := 0.30|velY := 5.00|g,cd $(LEAN_DIR) && lake build verify_capture)

test-energy: ## PoC 12: a 100 Wh battery is too small for a 1-h walking mission.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyEnergy.lean,s|BATTERY_WH       : Float := 800.0|BATTERY_WH       : Float := 100.0|,cd $(LEAN_DIR) && lake build verify_energy)

test-crush: ## PoC 13: at Mariana-trench pressure the joint gaps close → Lean rejects.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifySubsea.lean,s|EnvironmentParams.subsea500m|EnvironmentParams.marianaTrench|,cd $(LEAN_DIR) && lake build verify_subsea)

test-square-cube: ## PoC 15: a 4 m robot with PoC 8 small motors fails Lean's torque proof.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifyHeavy.lean,s|Actuator.heavyDutyHydraulic|Actuator.smallElectric|,cd $(LEAN_DIR) && lake build verify_heavy)

test-lethal-crash: ## PoC 16: stiff arms (braceStroke 10 mm) → cockpit G 100 > 15 → Lean rejects.
	$(call expect_fail,$(LEAN_DIR)/Tests/VerifySafety.lean,s|braceStrokeM := 0.10|braceStrokeM := 0.01|,cd $(LEAN_DIR) && lake build verify_safety)
