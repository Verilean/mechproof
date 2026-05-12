import MechProof
open MechProof

/-- Pilot-input filter sized to the 4 m heavy mech.

    Support polygon at scale 2.581× → ±284 mm (PoC 8's ±110 mm × 2.581).
    CoM height = 1.95 m (the steady-state torso Z reported by
    simulate_heavy.py). The filter clips pilot-requested ẍ to 1.0 m/s²;
    the LIPM accel-limit is 9.81·(0.284 − 0.05)/1.95 ≈ 1.18 m/s², so
    1.0 m/s² is a safe pass-through cap with ~15% margin. -/
def mechFilter : InputFilter :=
  { comHeightM   := 1.95,
    supportHalfX := 0.284,
    zmpMargin    := 0.05,
    maxAccelMS2  := 1.0 }

/-- Bracing posture: the cockpit drops 1 m, arms collapse over 100 mm of
    hydraulic stroke. Resulting deceleration is 10 G — well below the
    ejection-seat survival limit of 15 G. -/
def mechBrace : BracingPosture :=
  { fallHeightM  := 1.0,
    braceStrokeM := 0.10 }

def pilotedMech : PilotedMech :=
  { filter := mechFilter,
    brace  := mechBrace,
    pilot  := stdPilotLimits }

/-- **Compile-time safety proof.** Both the input-filter and the
    crash-bracing guarantees discharge by `native_decide`. -/
def mechSafe : pilotedMech.SafePilotedOperation := by
  refine ⟨?_, ?_⟩
  · refine ⟨?_, ?_⟩ <;> native_decide
  · native_decide

def renderJson : String :=
  let f := pilotedMech.filter
  let b := pilotedMech.brace
  let p := pilotedMech.pilot
  "{\"filter\":{\"comHeightM\":" ++ toString f.comHeightM ++
  ",\"supportHalfX\":" ++ toString f.supportHalfX ++
  ",\"zmpMargin\":" ++ toString f.zmpMargin ++
  ",\"maxAccelMS2\":" ++ toString f.maxAccelMS2 ++
  ",\"accelLimitMS2\":" ++ toString f.accelLimitMS2 ++ "}" ++
  ",\"brace\":{\"fallHeightM\":" ++ toString b.fallHeightM ++
  ",\"braceStrokeM\":" ++ toString b.braceStrokeM ++
  ",\"impactG\":" ++ toString b.impactG ++ "}" ++
  ",\"pilotLimits\":{\"maxSafeG\":" ++ toString p.maxSafeG ++ "}}"

def main : IO Unit := do
  let _ : pilotedMech.SafePilotedOperation := mechSafe
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/safety_params.json" renderJson
  IO.println "Lean piloted-mech safety proofs passed:"
  IO.println s!"  Filter cap        = {pilotedMech.filter.maxAccelMS2} m/s²"
  IO.println s!"  LIPM accel limit  = {pilotedMech.filter.accelLimitMS2} m/s²"
  IO.println s!"  Fall height       = {pilotedMech.brace.fallHeightM} m"
  IO.println s!"  Brace stroke      = {pilotedMech.brace.braceStrokeM} m"
  IO.println s!"  Impact G          = {pilotedMech.brace.impactG} g"
  IO.println s!"  Pilot G limit     = {pilotedMech.pilot.maxSafeG} g"
  IO.println "Wrote ../out/safety_params.json"
