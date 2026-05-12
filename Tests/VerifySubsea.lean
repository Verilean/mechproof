import MechProof
open MechProof

/-- Operating environment for this proof.  Swap `EnvironmentParams.subsea500m`
    for `marianaTrench` (or any future environment) to re-run every
    theorem against a different profile. -/
def env : EnvironmentParams := EnvironmentParams.subsea500m

/-! ### Material list (the parts that must keep their joint gaps) -/

/-- Aluminium-housed arm link (PoC 6: 22 mm Φ tube, 250 mm long).
    Joint gap = 0.5 mm designed, must stay above 0.1 mm. -/
def aluminiumArmLink : ComponentMaterial :=
  { name := "arm_link_aluminium",
    bulkModulus := 7.6e10,     -- aluminium 6061
    linkLengthM := 0.250,
    nominalGapM := 0.0005,
    minGapM     := 0.0001 }

/-- Nylon-printed finger link (PoC 3: 40 mm long).  Plastic is much more
    compressible than aluminium and is therefore the worst-case material
    for the clearance proof. -/
def nylonFingerLink : ComponentMaterial :=
  { name := "finger_link_nylon",
    bulkModulus := 4.0e9,      -- nylon-12
    linkLengthM := 0.040,
    nominalGapM := 0.0005,
    minGapM     := 0.0001 }

/-- Plastic-printed palm shell (PoC 4: 80 mm long). -/
def nylonPalm : ComponentMaterial :=
  { name := "palm_nylon",
    bulkModulus := 4.0e9,
    linkLengthM := 0.080,
    nominalGapM := 0.0005,
    minGapM     := 0.0001 }

def subseaMaterials : List ComponentMaterial :=
  [aluminiumArmLink, nylonFingerLink, nylonPalm]

/-! ### Hydrostatic body — buoyancy + righting moment -/

def humanoidBody : HydroBody :=
  { totalMassKg       := 21.2,
    displacedVolumeM3 := 0.0208,   -- ≈ 20.8 L (slight positive buoyancy)
    comHeightM        := 0.75,
    cobHeightM        := 0.80,     -- 5 cm above CoM → righting moment
    buoyancyTolerance := 0.05,     -- ≤ 5% mismatch
    rightingMarginM   := 0.02 }    -- ≥ 2 cm CoB-above-CoM

/-! ### Drag model in a crouched current-bracing pose

    Projected area is roughly half of the standing pose; moment arm is
    the height of the CoM above the ankle pivot.  Cd ≈ 1.2 is the
    Engineering-Toolbox value for a roughly cylindrical human silhouette. -/
def crouchedDrag : DragModel :=
  { projectedAreaM2 := 0.15,
    dragCoeff       := 1.2,
    momentArmM      := 0.50,
    -- 2 legs × (hipPitchTau + ankleTau)  (from VerifyLegs.lean)
    motorTorqueNm   := 2.0 * (50.0 + 20.0) }

/-! ### Compile-time proofs -/

def pressureSafe :
    allPressureSafe env subseaMaterials := by native_decide

def hydroBalanced :
    humanoidBody.HydrostaticBalanced env := by native_decide

def currentStable :
    crouchedDrag.CurrentStable env := by native_decide

/-! ### JSON rendering -/

def renderMaterial
    (e : EnvironmentParams) (c : ComponentMaterial) : String :=
  "{\"name\":\"" ++ c.name ++ "\"" ++
  ",\"bulkModulusPa\":" ++ toString c.bulkModulus ++
  ",\"linkLengthM\":" ++ toString c.linkLengthM ++
  ",\"nominalGapM\":" ++ toString c.nominalGapM ++
  ",\"minGapM\":" ++ toString c.minGapM ++
  ",\"shrinkageM\":" ++ toString (c.shrinkageM e) ++
  ",\"gapAfterPressureM\":" ++ toString (c.gapAfterPressureM e) ++ "}"

def renderMaterials :
    EnvironmentParams → List ComponentMaterial → List String
  | _, [] => []
  | e, c :: rest => renderMaterial e c :: renderMaterials e rest

def renderEnvironment (e : EnvironmentParams) : String :=
  "{\"name\":\"" ++ e.name ++ "\"" ++
  ",\"densityKgM3\":" ++ toString e.density ++
  ",\"gravityMS2\":" ++ toString e.gravity ++
  ",\"pressurePa\":" ++ toString e.pressure ++
  ",\"currentVelMS\":" ++ toString e.currentVel ++
  ",\"viscosityPas\":" ++ toString e.viscosity ++
  ",\"temperatureK\":" ++ toString e.temperature ++
  ",\"buoyancyEnabled\":" ++ (if e.buoyancyEnabled then "true" else "false") ++
  "}"

def main : IO Unit := do
  let _ : allPressureSafe env subseaMaterials := pressureSafe
  let _ : humanoidBody.HydrostaticBalanced env := hydroBalanced
  let _ : crouchedDrag.CurrentStable env := currentStable
  IO.FS.createDirAll "out"
  let materialJson := String.intercalate ","
    (renderMaterials env subseaMaterials)
  let json :=
    "{\"environment\":" ++ renderEnvironment env ++
    ",\"materials\":[" ++ materialJson ++ "]" ++
    ",\"hydroBody\":{" ++
      "\"totalMassKg\":" ++ toString humanoidBody.totalMassKg ++
      ",\"displacedVolumeM3\":" ++ toString humanoidBody.displacedVolumeM3 ++
      ",\"comHeightM\":" ++ toString humanoidBody.comHeightM ++
      ",\"cobHeightM\":" ++ toString humanoidBody.cobHeightM ++
      ",\"weightN\":" ++ toString (humanoidBody.weightN env) ++
      ",\"buoyancyN\":" ++ toString (humanoidBody.buoyancyN env) ++
    "}" ++
    ",\"drag\":{" ++
      "\"projectedAreaM2\":" ++ toString crouchedDrag.projectedAreaM2 ++
      ",\"dragCoeff\":" ++ toString crouchedDrag.dragCoeff ++
      ",\"momentArmM\":" ++ toString crouchedDrag.momentArmM ++
      ",\"motorTorqueNm\":" ++ toString crouchedDrag.motorTorqueNm ++
      ",\"dragForceN\":" ++ toString (crouchedDrag.dragForceN env) ++
      ",\"dragMomentNm\":" ++ toString (crouchedDrag.dragMomentNm env) ++
    "}}"
  IO.FS.writeFile "out/subsea_params.json" json
  IO.println "Lean subsea proofs passed:"
  IO.println s!"  environment        = {env.name}"
  IO.println s!"  ambient pressure   = {env.pressure / 1.0e6} MPa"
  IO.println "  • Every material's joint gap survives the pressure:"
  for m in subseaMaterials do
    IO.println s!"      {m.name}: shrinkage = {m.shrinkageM env * 1.0e6} μm, "
    IO.println s!"                gap after  = {m.gapAfterPressureM env * 1.0e6} μm  "
    IO.println s!"                (min required {m.minGapM * 1.0e6} μm)"
  IO.println s!"  • Hydrostatic balance OK (CoB {humanoidBody.cobHeightM} m above CoM {humanoidBody.comHeightM} m)."
  IO.println s!"  • Drag moment {crouchedDrag.dragMomentNm env} N·m < motor capacity {crouchedDrag.motorTorqueNm} N·m."
  IO.println "Wrote ../out/subsea_params.json"
