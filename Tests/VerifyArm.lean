import MechProof
open MechProof

/-- Candidate 6-DOF arm.

    Link lengths and masses correspond to a small/medium cobot:
      L1 = 300 mm (shoulder→elbow)
      L2 = 250 mm (elbow→wrist)
      L3 = 100 mm (wrist→hand flange)

    Motor stall torques are selected so the static demand at full horizontal
    extension carrying HAND_MASS + PAYLOAD_MASS is comfortably below the
    motor capability. Margins are quoted in the JSON output for the
    manufacturing certificate. -/
def candidateArm : ArmParams :=
  { l1 := 0.30, l2 := 0.25, l3 := 0.10,
    m1 := 1.2,  m2 := 0.8,  m3 := 0.3,
    tauShoulder := 30.0,
    tauElbow    := 15.0,
    tauWrist    :=  4.0 }

/-- Compile-time well-formedness. -/
def candidateArmWellFormed : candidateArm.WellFormed := by
  refine ⟨?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_⟩ <;> native_decide

/-- **Compile-time stall-torque proof.** Every pitch motor is strong enough
    to hold the arm + hand + 2 kg payload at horizontal extension. -/
def candidateArmStallSufficient : candidateArm.StallSufficient := by
  native_decide

def renderArmJson (a : ArmParams) : String :=
  "{\"l1\":"            ++ toString a.l1 ++
  ",\"l2\":"            ++ toString a.l2 ++
  ",\"l3\":"            ++ toString a.l3 ++
  ",\"m1\":"            ++ toString a.m1 ++
  ",\"m2\":"            ++ toString a.m2 ++
  ",\"m3\":"            ++ toString a.m3 ++
  ",\"tauShoulder\":"   ++ toString a.tauShoulder ++
  ",\"tauElbow\":"      ++ toString a.tauElbow ++
  ",\"tauWrist\":"      ++ toString a.tauWrist ++
  ",\"payloadMassKg\":" ++ toString PAYLOAD_MASS_KG ++
  ",\"handMassKg\":"    ++ toString HAND_MASS_KG ++
  ",\"gravity\":"       ++ toString GRAVITY_M_PER_S2 ++
  ",\"requiredShoulderTorque\":" ++ toString a.requiredShoulderTorque ++
  ",\"requiredElbowTorque\":"    ++ toString a.requiredElbowTorque ++
  ",\"requiredWristTorque\":"    ++ toString a.requiredWristTorque ++
  "}"

def main : IO Unit := do
  let _ : candidateArm.WellFormed := candidateArmWellFormed
  let _ : candidateArm.StallSufficient := candidateArmStallSufficient
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/arm_params.json" (renderArmJson candidateArm)
  IO.println "Lean proofs passed:"
  IO.println "  • Arm geometry is well-formed."
  IO.println "  • Stall torque sufficient at horizontal extension:"
  IO.println s!"      shoulder needs {candidateArm.requiredShoulderTorque} N·m, has {candidateArm.tauShoulder}"
  IO.println s!"      elbow    needs {candidateArm.requiredElbowTorque} N·m, has {candidateArm.tauElbow}"
  IO.println s!"      wrist    needs {candidateArm.requiredWristTorque} N·m, has {candidateArm.tauWrist}"
  IO.println "Wrote ../out/arm_params.json"
