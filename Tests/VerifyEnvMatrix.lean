import MechProof
open MechProof

/-! Sweeps the PoC 13 subsea-style proofs across every environment in
    `EnvironmentParams.all` and emits a JSON record per environment so
    the Python aggregator can table it.

    Strategy:
      * The PressureClearance + CurrentStable theorems are universally
        applicable (any fluid / any gravity).
      * Hydrostatic balance only makes sense when the environment has a
        non-trivial fluid — we gate it on `buoyancyEnabled`.

    The Lean side outputs the **numeric outcome** for every environment;
    `make verify-env-matrix` discharges a single combined theorem
    (`allShippingEnvSafe`) that asserts every *shipping* environment
    (airSurface, subsea500m, lunar, mars) passes pressure + drag. The
    Mariana-trench environment is *not* in that list — `test-crush`
    keeps demonstrating that the proof gate fires there. -/

/-- Component materials and drag model for the standard 30-DOF humanoid
    (same as VerifySubsea.lean). The matrix re-uses these so every
    environment row is comparing apples to apples. -/
def stdMaterials : List ComponentMaterial := [
  { name := "arm_link_aluminium",
    bulkModulus := 7.6e10, linkLengthM := 0.250,
    nominalGapM := 0.0005, minGapM := 0.0001 },
  { name := "finger_link_nylon",
    bulkModulus := 4.0e9,  linkLengthM := 0.040,
    nominalGapM := 0.0005, minGapM := 0.0001 },
  { name := "palm_nylon",
    bulkModulus := 4.0e9,  linkLengthM := 0.080,
    nominalGapM := 0.0005, minGapM := 0.0001 }
]

def stdDrag : DragModel :=
  { projectedAreaM2 := 0.15,
    dragCoeff       := 1.2,
    momentArmM      := 0.50,
    motorTorqueNm   := 140.0 }

/-! ### Per-environment Bool decisions -/

def materialsSafeB (e : EnvironmentParams) : Bool :=
  stdMaterials.all (ComponentMaterial.pressureSafeB e)

def dragSafeB (e : EnvironmentParams) : Bool :=
  decide (stdDrag.dragMomentNm e < stdDrag.motorTorqueNm)

/-- A shipping environment is "safe" iff materials survive its pressure
    and the standard drag model is sub-stall.  Buoyancy is checked
    separately because it doesn't apply in vacuum/atmosphere. -/
def envSafeB (e : EnvironmentParams) : Bool :=
  materialsSafeB e && dragSafeB e

/-! ### Combined theorem -/

abbrev allShippingEnvSafe : Prop :=
  (EnvironmentParams.all.all envSafeB) = true

def matrixTheorem : allShippingEnvSafe := by native_decide

/-! ### JSON rendering -/

/-- The materials in `stdMaterials` are aluminium / nylon-finger / palm.
    These accessors pull each by index without depending on `List.get!`. -/
def stdMaterialAl     : ComponentMaterial := stdMaterials[0]!
def stdMaterialNylon  : ComponentMaterial := stdMaterials[1]!
def stdMaterialPalm   : ComponentMaterial := stdMaterials[2]!

def envEntry (e : EnvironmentParams) : String :=
  let dragF := stdDrag.dragForceN e
  let dragM := stdDrag.dragMomentNm e
  let alShr   := stdMaterialAl.shrinkageM e
  let nyShr   := stdMaterialNylon.shrinkageM e
  let palmShr := stdMaterialPalm.shrinkageM e
  "{\"name\":\"" ++ e.name ++ "\"" ++
  ",\"gravityMS2\":" ++ toString e.gravity ++
  ",\"densityKgM3\":" ++ toString e.density ++
  ",\"pressurePa\":" ++ toString e.pressure ++
  ",\"currentVelMS\":" ++ toString e.currentVel ++
  ",\"buoyancyEnabled\":" ++ (if e.buoyancyEnabled then "true" else "false") ++
  ",\"materialsSafe\":" ++ (if materialsSafeB e then "true" else "false") ++
  ",\"dragSafe\":" ++ (if dragSafeB e then "true" else "false") ++
  ",\"envSafe\":" ++ (if envSafeB e then "true" else "false") ++
  ",\"alLinkShrinkageM\":" ++ toString alShr ++
  ",\"nylonFingerShrinkageM\":" ++ toString nyShr ++
  ",\"palmShrinkageM\":" ++ toString palmShr ++
  ",\"dragForceN\":" ++ toString dragF ++
  ",\"dragMomentNm\":" ++ toString dragM ++
  ",\"motorTorqueNm\":" ++ toString stdDrag.motorTorqueNm ++
  "}"

def renderAll : List EnvironmentParams → List String
  | [] => []
  | e :: rest => envEntry e :: renderAll rest

def main : IO Unit := do
  let _ : allShippingEnvSafe := matrixTheorem
  IO.FS.createDirAll "out"
  -- We also emit the Mariana entry so the Python aggregator can show it
  -- as a deliberately-failing row, but it doesn't gate the build.
  let environments : List EnvironmentParams :=
    EnvironmentParams.all ++ [EnvironmentParams.marianaTrench]
  let body := String.intercalate "," (renderAll environments)
  let json := "{\"environments\":[" ++ body ++ "]}"
  IO.FS.writeFile "out/env_matrix.json" json
  IO.println "Lean environment matrix verified — every shipping env is safe."
  for e in environments do
    let tag := if envSafeB e then "PASS" else "FAIL"
    IO.println s!"  {e.name}  gravity={e.gravity}  density={e.density}  pressure_Pa={e.pressure}  → {tag}"
  IO.println "Wrote ../out/env_matrix.json"
