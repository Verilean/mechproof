import MechProof
open MechProof

/-- Candidate tendon-driven finger.

    Finger geometry is reused from PoC 2 (40/30/20 mm, 6 mm thick, 0°→90°).
    Moment arms taper distally: 3.0 / 2.5 / 2.0 mm — physically reasonable for
    a 6 mm-thick link with the tendon routed near the palmar surface. -/
def candidate : TendonFinger :=
  { finger := { l1 := 40.0, l2 := 30.0, l3 := 20.0,
                thickness := 6.0, minAngle := 0.0, maxAngle := 90.0 },
    tendon := { r1 := 3.0, r2 := 2.5, r3 := 2.0 } }

/-- Compile-time well-formedness check for the full assembly. -/
def candidateWellFormed : candidate.WellFormed := by
  refine ⟨?_, ?_, ?_⟩
  · refine ⟨?_, ?_, ?_, ?_, ?_⟩ <;> native_decide
  · refine ⟨?_, ?_⟩ <;> native_decide
  · refine ⟨?_, ?_, ?_⟩ <;> native_decide

/-- Derived guarantees. -/
def candidateGuarantees :
    candidate.finger.NoBackwardBending ∧ candidate.tendon.PositiveFlexion :=
  wellFormed_finger_and_tendon candidate candidateWellFormed

def renderJson (tf : TendonFinger) : String :=
  let f := tf.finger
  let t := tf.tendon
  "{" ++
    s!"\"l1\":{f.l1}," ++
    s!"\"l2\":{f.l2}," ++
    s!"\"l3\":{f.l3}," ++
    s!"\"thickness\":{f.thickness}," ++
    s!"\"minAngle\":{f.minAngle}," ++
    s!"\"maxAngle\":{f.maxAngle}," ++
    s!"\"r1\":{t.r1}," ++
    s!"\"r2\":{t.r2}," ++
    s!"\"r3\":{t.r3}" ++
  "}"

def main : IO Unit := do
  let _ : candidate.WellFormed := candidateWellFormed
  let _ : candidate.finger.NoBackwardBending ∧ candidate.tendon.PositiveFlexion :=
    candidateGuarantees
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/tendon_params.json" (renderJson candidate)
  IO.println "Lean proof passed: tendon routing is valid and flexion is positive."
  IO.println "Wrote ../out/tendon_params.json"
