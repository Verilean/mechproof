import MechProof
open MechProof

/-! A 4-metre / steel-construction humanoid powered by hydraulic
    actuators.  Mass scaling factor:
        s³ · ρ_ratio  =  2.581³ · 6.3  ≈  17.2 · 6.3  ≈  108×
    So our 21.2 kg PoC 8 baseline becomes ~2.3 tonnes — Patlabor class. -/

def heavyScale : RobotScale :=
  { linearScale       := 2.581,        -- 4.0 m / 1.55 m baseline
    densityRatio      := 6.3,          -- steel / PLA
    thighDiameterM    := 0.145,        -- 145 mm OD scaled thigh tube
    thighWallM        := 0.0075,       -- 7.5 mm wall
    yieldStressPa     := 2.5e8,        -- mild steel σ_y = 250 MPa
    baselineMassKg    := 21.2,
    baselineThighLenM := 0.30,
    baselineUpperKg   := 17.0 }

def heavyBuild : HeavyBuild :=
  { scale  := heavyScale,
    knee   := Actuator.heavyDutyHydraulic,
    margin := 0.20 }                    -- require 20% headroom

/-- The compile-time gate.  If the configured actuator class is too
    weak for the scale, or the chosen tube section yields, the build
    halts and no CAD / simulation is emitted. -/
def heavyStable : heavyBuild.HeavyStandStable := by
  refine ⟨?_, ?_⟩ <;> native_decide

def renderActuator (a : Actuator) : String :=
  "{\"name\":\"" ++ a.name ++ "\"" ++
  ",\"stallTorqueNm\":" ++ toString a.stallTorqueNm ++
  ",\"ratedTorqueNm\":" ++ toString a.ratedTorqueNm ++ "}"

def renderScale (r : RobotScale) : String :=
  "{\"linearScale\":" ++ toString r.linearScale ++
  ",\"densityRatio\":" ++ toString r.densityRatio ++
  ",\"thighDiameterM\":" ++ toString r.thighDiameterM ++
  ",\"thighWallM\":" ++ toString r.thighWallM ++
  ",\"yieldStressPa\":" ++ toString r.yieldStressPa ++
  ",\"baselineMassKg\":" ++ toString r.baselineMassKg ++
  ",\"baselineThighLenM\":" ++ toString r.baselineThighLenM ++
  ",\"baselineUpperKg\":" ++ toString r.baselineUpperKg ++
  ",\"totalMassKg\":" ++ toString r.totalMassKg ++
  ",\"thighLenM\":" ++ toString r.thighLenM ++
  ",\"upperBodyKg\":" ++ toString r.upperBodyKg ++
  ",\"requiredKneeTorqueNm\":" ++ toString r.requiredKneeTorqueNm ++
  ",\"thighBendingStressPa\":" ++ toString r.thighBendingStressPa ++ "}"

def main : IO Unit := do
  let _ : heavyBuild.HeavyStandStable := heavyStable
  IO.FS.createDirAll "out"
  let json :=
    "{\"scale\":" ++ renderScale heavyBuild.scale ++
    ",\"knee\":" ++ renderActuator heavyBuild.knee ++
    ",\"margin\":" ++ toString heavyBuild.margin ++
    "}"
  IO.FS.writeFile "out/heavy_params.json" json
  IO.println "Lean heavy-machinery proofs passed:"
  IO.println s!"  scale       = {heavyBuild.scale.linearScale}× linear "
  IO.println s!"  total mass  = {heavyBuild.scale.totalMassKg} kg (~"
  IO.println s!"                  {heavyBuild.scale.totalMassKg / 1000.0} t)"
  IO.println s!"  thigh len   = {heavyBuild.scale.thighLenM} m"
  IO.println s!"  required knee τ = {heavyBuild.scale.requiredKneeTorqueNm} N·m"
  IO.println s!"  actuator  τ_stall = {heavyBuild.knee.stallTorqueNm} N·m"
  IO.println s!"  thigh σ_bending = {heavyBuild.scale.thighBendingStressPa / 1.0e6} MPa"
  IO.println s!"  yield σ_y       = {heavyBuild.scale.yieldStressPa / 1.0e6} MPa"
  IO.println "Wrote ../out/heavy_params.json"
