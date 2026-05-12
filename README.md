# MechProof

> **Formally-verified hardware compiler.** Robot designs that don't pass
> the Lean 4 proofs simply never generate any CAD or simulation
> artefacts — the build halts. Designs that do are guaranteed by
> construction to satisfy every geometric, kinematic, energetic,
> environmental, and safety property the library defines.

```
$ make poc16                # full headline pipeline (PoC 1 → 16, ~5 min)
$ make test                 # 13 mutated bad designs — Lean must reject all
$ make help                 # every available target
```

------------------------------------------------------------------------

## Why MechProof exists

Most robot specs live in Word documents, then get translated by humans
into CAD, then into URDF, then into MuJoCo, then into firmware — drifting
at every step. MechProof flips that:

1. The spec is a **Lean 4 structure** with theorems attached.
2. Lean's `native_decide` discharges those theorems against concrete
   numbers.
3. **Only if all theorems pass** does the build emit CAD (STEP files),
   URDF, MuJoCo XML, and ROS-compatible JSON metadata.
4. The same numbers Lean proved are the numbers the simulator runs.
   Mismatches are impossible by construction.

This is exactly the discipline `cargo build` enforces for Rust types,
but applied to physical correctness conditions like
"the thumb doesn't crush the index finger when the swivel goes to max"
or "the knee motor can hold up 2.3 tonnes."

------------------------------------------------------------------------

## Repo layout

```
mechproof/
├── MechProof.lean              ← library entry point (re-exports modules)
├── MechProof/                  ← reusable Lean modules (the *library*)
│   ├── Basic.lean              ← PoC 1   moldability
│   ├── Finger.lean             ← PoC 2   3-link finger kinematics
│   ├── TendonFinger.lean       ← PoC 3   tendon-driven flexion
│   ├── HandAssembly.lean       ← PoC 4   5-finger capsule clearance
│   ├── DFM.lean                ← PoC 5   manufacturing rules
│   ├── ArmAssembly.lean        ← PoC 6   arm static torque
│   ├── LegAssembly.lean        ← PoC 8   leg balance + squat torque
│   ├── Walking.lean            ← PoC 10  ZMP / LIPM
│   ├── CapturePoint.lean       ← PoC 11  capture-point stability
│   ├── Energy.lean             ← PoC 12  power + endurance
│   ├── Environment.lean        ← PoC 13  EnvironmentParams (air/subsea/lunar/mars/…)
│   ├── Subsea.lean             ← PoC 13  pressure / buoyancy / drag
│   ├── HeavyMachinery.lean     ← PoC 15  scale-up + yield
│   └── PilotSafety.lean        ← PoC 16  override + crash brace
│
├── Tests/                      ← per-PoC entrypoints (one Lean exe + one orchestrator each)
│   ├── Main.lean               ← `verify_mechproof`
│   ├── VerifyFinger.lean       ← `verify_finger`
│   ├── …                       ← VerifyTendon / VerifyHand / VerifyArm / VerifyLegs /
│   │                             VerifyWalking / VerifyCapture / VerifyEnergy /
│   │                             VerifySubsea / VerifyEnvMatrix / VerifyHeavy /
│   │                             VerifySafety
│   └── run_poc*.sh             ← orchestrators (Lean → Python → simulator → report)
│
├── python/                     ← CAD generators + MuJoCo simulators + report writers
├── schema/                     ← JSON schemas for Lean-emitted ground-truth files
├── lakefile.toml               ← Lake project root (Lean 4 build config)
├── lean-toolchain              ← pinned Lean version (leanprover/lean4:v4.x.y)
├── Makefile                    ← human-facing interface (`make help`)
├── shell.nix                   ← Nix dev shell (lean, OCP runtime libs, etc.)
└── out/                        ← generated artefacts (gitignored)
```

------------------------------------------------------------------------

## How a MechProof theorem is structured

Every PoC follows the same pattern. Here's the **DFM** check (PoC 5) as
the canonical example.

### 1.  Declare the data the theorem operates on

In `MechProof/DFM.lean`:

```lean
structure TendonChannelGeom where
  channelRadius : Float    -- m
  momentArm     : Float    -- m, offset from joint axis
  deriving Repr, Inhabited
```

### 2.  Encode the physical rule as a `Prop`

```lean
abbrev linkManufacturable
    (linkThickness : Float) (g : TendonChannelGeom) : Prop :=
  MIN_TENDON_HOLE_DIA_M / 2 ≤ g.channelRadius ∧
  MIN_WALL_THICKNESS_M ≤
    linkThickness / 2 - g.momentArm - g.channelRadius
```

* `abbrev` rather than `def` so the inequality is **reducible** during
  typeclass inference — that's what lets `native_decide` see it as a
  computable boolean.
* The body is a strict inequality on `Float`. `Float.<` is decidable in
  Lean, so the whole proposition is decidable; `native_decide` evaluates
  it at compile time using compiled IR.

### 3.  Instantiate concrete numbers in `Tests/VerifyHand.lean`

```lean
def stdChannels : FingerChannels :=
  { ch1 := { channelRadius := 0.0006, momentArm := 0.0025 },
    ch2 := { channelRadius := 0.0006, momentArm := 0.0020 },
    ch3 := { channelRadius := 0.0006, momentArm := 0.0015 } }

def candidateManufacturable :
    candidate.Manufacturable candidateChannels := by
  native_decide
```

If any number ever drifts to a non-compliant value (say
`channelRadius := 0.0003` — too small for injection moulding),
`native_decide` evaluates the conjunction to `false` and the build
aborts with

```
error: Tactic `native_decide` evaluated that the proposition
       candidate.Manufacturable candidateChannels is false
```

### 4.  Emit the proven numbers as JSON so downstream tools can consume them

```lean
def main : IO Unit := do
  let _ : candidate.WellFormed := candidateWellFormed
  let _ : candidate.Manufacturable candidateChannels := candidateManufacturable
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/hand_params.json"
    (renderJson candidate candidateChannels)
```

Crucially `IO.FS.writeFile` only runs *after* the `let _ : … := …`
lines, and those lines only typecheck if the proofs hold. So **the JSON
file exists ⇔ the proofs passed**.

### 5.  Python tools blindly trust the JSON

`python/generate_hand_cad.py` reads `hand_params.json` and turns it into
STEP files. It does **not** re-validate the numbers — the file's
existence is the validation.

------------------------------------------------------------------------

## How a PoC orchestrator (run script) is structured

`Tests/run_pocN.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."     # always run from repo root
mkdir -p out

# 1. Discharge the Lean proof. `set -e` makes the script abort if
#    `lake exe` exits non-zero (which happens when `native_decide`
#    rejects).
if ! ( lake build verify_xxx && lake exe verify_xxx ); then
  echo "Verification Failed: ..." >&2
  exit 1
fi

# 2. Generate CAD from the JSON Lean just wrote.
./venv/bin/python python/generate_xxx_cad.py

# 3. Run MuJoCo simulation against the CAD.
./venv/bin/python python/simulate_xxx.py
```

The Makefile wraps every script in `nix-shell --run …` so users never
have to think about the Nix environment.

------------------------------------------------------------------------

## Quick-start (5 minutes)

```bash
# 1. Enter the Nix dev shell (one-time setup).
nix-shell

# 2. Build the Lean library (downloads the toolchain on first run).
make build

# 3. Run the full headline pipeline.
make poc16
# → out/Manned_Safety_Report.txt   "RESULT: PASS"
# → out/safety_params.json         (Lean-certified pilot safety params)
# → out/cockpit_g_force.json       (MuJoCo 1 kHz accelerometer trace)

# 4. Prove the proof gate actually works by deliberately breaking
#    13 designs.
make test
# → 13/13 PASS — every bad design correctly rejected by Lean.

# 5. List every available target.
make help
```

------------------------------------------------------------------------

## Negative tests (the gate's gate)

`make test` runs 13 sed-mutated builds, one per PoC, each demonstrating
that a deliberately-broken design **cannot** pass Lean. The mutations:

| Test                | Mutation                                                | Theorem that catches it      |
|---------------------|---------------------------------------------------------|------------------------------|
| `test-moldability`  | `draftDeg := 2.0` → `-1.0`                              | `wellFormed_implies_moldable` |
| `test-tendon`       | `r2 := 2.5` → `-1.0`                                    | `PositiveFlexion`            |
| `test-collision`    | thumb mount onto index finger                            | `ThumbIndexClear`            |
| `test-dfm-wall`     | finger thickness 10 mm → 2 mm                            | `Manufacturable`             |
| `test-dfm-hole`     | tendon channel ⌀ 1.2 mm → 0.6 mm                         | `Manufacturable`             |
| `test-torque`       | shoulder stall 30 N·m → 5 N·m                            | `StallSufficient`            |
| `test-balance`      | foot length 160 mm → 20 mm                               | `Balanced`                   |
| `test-zmp`          | CoM accel 0 → 5 m/s²                                     | `allStable` (ZMP)            |
| `test-capture`      | CoM velocity 0.3 → 5.0 m/s                               | `allCapturable`              |
| `test-energy`       | battery 800 Wh → 100 Wh                                  | `MissionPossible`            |
| `test-crush`        | subsea 500 m → Mariana trench (110 MPa)                  | `PressureClearance`          |
| `test-square-cube`  | 4 m mech with PoC 8's 40 N·m motors                      | `HeavyTorqueSufficient`      |
| `test-lethal-crash` | brace stroke 100 mm → 10 mm (cockpit G = 100)            | `SurvivalBrace`              |

Each script-mutated file is restored from a backup via `trap`, so
failed tests never leave the repo in a bad state.

------------------------------------------------------------------------

## How to add a new theorem

1. **Define the structure + Prop** in a new file under `MechProof/`.
   Mark the `Prop` `abbrev` so it's decidable.
2. **Instantiate concrete numbers** in `Tests/VerifyYourThing.lean`:

   ```lean
   import MechProof
   open MechProof

   def candidate : YourStruct := …
   def candidateOk : candidate.YourProperty := by native_decide

   def main : IO Unit := do
     let _ : candidate.YourProperty := candidateOk
     IO.FS.writeFile "out/your_params.json" (renderJson candidate)
   ```

3. **Wire it into Lake**: add an `[[lean_exe]]` entry to `lakefile.toml`:

   ```toml
   [[lean_exe]]
   name = "verify_your_thing"
   root = "Tests.VerifyYourThing"
   ```

4. **Add a Make target** in `Makefile`:

   ```make
   verify-your-thing: ## one-line description
     $(NIX_RUN) "cd $(LEAN_DIR) && lake build verify_your_thing && lake exe verify_your_thing"
   ```

5. **Add a negative test** so `make test` keeps proving the gate works:

   ```make
   test-your-thing:
     $(call expect_fail,$(LEAN_DIR)/Tests/VerifyYourThing.lean, \
            s|good_value|bad_value|, \
            cd $(LEAN_DIR) && lake build verify_your_thing)
   ```

That's the whole pipeline. Each PoC in `MechProof/` and `Tests/` is a
worked example of this five-step recipe.

------------------------------------------------------------------------

## Coverage matrix

| PoC | Topic                            | Library file             | Verify exe              |
|-----|----------------------------------|--------------------------|-------------------------|
| 1   | Injection-mould draft            | `Basic.lean`             | `mechproof`             |
| 2   | 3-link finger kinematics         | `Finger.lean`            | `verify_finger`         |
| 3   | Tendon-driven flexion            | `TendonFinger.lean`      | `verify_tendon`         |
| 4   | 5-finger capsule clearance       | `HandAssembly.lean`      | `verify_hand`           |
| 5   | DFM (wall thickness + hole dia)  | `DFM.lean`               | (in `verify_hand`)      |
| 6   | Arm static torque                | `ArmAssembly.lean`       | `verify_arm`            |
| 7   | Grasp matrix + v1 packaging      | (Python-only)            | —                       |
| 8   | Lower-body balance + squat       | `LegAssembly.lean`       | `verify_legs`           |
| 9   | Humanoid v1.0 release archive    | (Python-only)            | —                       |
| 10  | LIPM / ZMP walking               | `Walking.lean`           | `verify_walking`        |
| 11  | Capture-Point + URDF + IMU       | `CapturePoint.lean`      | `verify_capture`        |
| 12  | Battery / endurance              | `Energy.lean`            | `verify_energy`         |
| 13  | Subsea 500 m operation           | `Subsea.lean`            | `verify_subsea`         |
| 14  | Multi-environment catalog        | `Environment.lean`       | `verify_env_matrix`     |
| 15  | 4 m / 2.3 t heavy machinery      | `HeavyMachinery.lean`    | `verify_heavy`          |
| 16  | Piloted-mech safety              | `PilotSafety.lean`       | `verify_safety`         |

------------------------------------------------------------------------

## Dependencies

Captured declaratively in `shell.nix`:

* **Lean 4** (`elan default stable` on first entry)
* **CadQuery / OCP** native runtime libs (`libstdc++`, `libGL`,
  `libGLU`, `fontconfig`, `libXi`, …) surfaced via `LD_LIBRARY_PATH`
* **MuJoCo 3.8**, **Pillow**, **mujoco-python** in `./venv/`
* **Verilator / iverilog / yosys** + RISC-V cross-compilers (legacy
  HDL toolchain — unused by the active PoCs but kept for parity with
  earlier hardware tooling work)

`./venv/` is created via `pip install cadquery mujoco Pillow` and is
gitignored — re-create with the same `pip install` calls you'll see in
the PoC scripts' comments if you're starting fresh.

------------------------------------------------------------------------

## License

TBD.
