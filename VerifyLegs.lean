import MechProof
open MechProof

/-- The candidate humanoid lower body.

    All distances in metres, all masses in kg. The `upperBodyMass` lumps
    together the torso payload (arm + hand + payload + electronics).
    Together with `torso.mass` this represents everything that hangs off
    the hips.

    Foot footprint: 160 mm long × 80 mm wide. With hipOffsetX = 70 mm,
    the support polygon extends ±110 mm in X and ±80 mm in Y from the
    body midline — comfortably beyond the 20 mm balance margin.

    Knee stall torque: 24.3 N·m required at 90° squat; we supply 40 N·m
    (~65% margin) — well above the realistic harmonic-drive-servo range. -/
def candidateLowerBody : LowerBody :=
  { torso := { width  := 0.20,
               depth  := 0.15,
               height := 0.30,
               mass   := 5.0 },
    leg := { thighLen    := 0.30,
             shinLen     := 0.28,
             footLen     := 0.16,
             footWidth   := 0.08,
             thighMass   := 1.5,
             shinMass    := 1.2,
             footMass    := 0.4,
             hipOffsetX  := 0.07,
             hipPitchTau := 50.0,
             kneeTau     := 40.0,
             ankleTau    := 20.0 },
    upperBodyMass := 10.0 }

/-- Compile-time well-formedness. -/
def candidateLowerBodyWellFormed : candidateLowerBody.WellFormed := by
  refine ⟨?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_⟩ <;>
    native_decide

/-- **Compile-time static balance proof.** The systemic CoM projects
    inside the support polygon (with the 2 cm margin baked in). -/
def candidateBalanced : candidateLowerBody.Balanced := by native_decide

/-- **Compile-time squat-torque proof.** The knee stall torque exceeds
    the static demand at 90° flexion carrying half the upper-body mass. -/
def candidateSquatTorqueSufficient :
    candidateLowerBody.SquatTorqueSufficient := by native_decide

def renderLegsJson (lb : LowerBody) : String :=
  let t := lb.torso
  let l := lb.leg
  "{\"torso\":{\"width\":"      ++ toString t.width ++
  ",\"depth\":"                 ++ toString t.depth ++
  ",\"height\":"                ++ toString t.height ++
  ",\"mass\":"                  ++ toString t.mass ++ "}" ++
  ",\"leg\":{\"thighLen\":"     ++ toString l.thighLen ++
  ",\"shinLen\":"               ++ toString l.shinLen ++
  ",\"footLen\":"               ++ toString l.footLen ++
  ",\"footWidth\":"             ++ toString l.footWidth ++
  ",\"thighMass\":"             ++ toString l.thighMass ++
  ",\"shinMass\":"              ++ toString l.shinMass ++
  ",\"footMass\":"              ++ toString l.footMass ++
  ",\"hipOffsetX\":"            ++ toString l.hipOffsetX ++
  ",\"hipPitchTau\":"           ++ toString l.hipPitchTau ++
  ",\"kneeTau\":"               ++ toString l.kneeTau ++
  ",\"ankleTau\":"              ++ toString l.ankleTau ++ "}" ++
  ",\"upperBodyMass\":"         ++ toString lb.upperBodyMass ++
  ",\"totalMass\":"             ++ toString lb.totalMass ++
  ",\"supportHalfX\":"          ++ toString lb.supportHalfX ++
  ",\"supportHalfY\":"          ++ toString lb.supportHalfY ++
  ",\"balanceMargin\":"         ++ toString staticBalanceMarginM ++
  ",\"requiredKneeTorque\":"    ++ toString lb.requiredKneeTorque ++
  ",\"gravity\":"               ++ toString GRAVITY_M_PER_S2 ++ "}"

def main : IO Unit := do
  let _ : candidateLowerBody.WellFormed := candidateLowerBodyWellFormed
  let _ : candidateLowerBody.Balanced := candidateBalanced
  let _ : candidateLowerBody.SquatTorqueSufficient :=
    candidateSquatTorqueSufficient
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/leg_params.json" (renderLegsJson candidateLowerBody)
  IO.println "Lean proofs passed:"
  IO.println "  • Lower body is well-formed."
  IO.println s!"  • Total mass = {candidateLowerBody.totalMass} kg."
  IO.println "  • CoM projects strictly inside the support polygon:"
  IO.println s!"      support half-extent : ±{candidateLowerBody.supportHalfX} m (X), "
  IO.println s!"                            ±{candidateLowerBody.supportHalfY} m (Y),"
  IO.println s!"      margin              : {staticBalanceMarginM} m."
  IO.println "  • Knee stall torque sufficient at 90° squat:"
  IO.println s!"      required = {candidateLowerBody.requiredKneeTorque} N·m, "
  IO.println s!"      supplied = {candidateLowerBody.leg.kneeTau} N·m."
  IO.println "Wrote ../out/leg_params.json"
