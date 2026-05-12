import MechProof
open MechProof

/-- Candidate 3-link finger: 40/30/20 mm links, 6 mm thick, joints 0°→90°. -/
def candidateFinger : FingerParams :=
  { l1 := 40.0, l2 := 30.0, l3 := 20.0,
    thickness := 6.0, minAngle := 0.0, maxAngle := 90.0 }

/-- Compile-time well-formedness check. -/
def candidateWellFormed : candidateFinger.WellFormed := by
  refine ⟨?_, ?_, ?_, ?_, ?_⟩ <;> native_decide

/-- Compile-time joint-bound check (the discharged hypothesis). -/
def candidateBounds :
    0 ≤ candidateFinger.minAngle ∧ candidateFinger.maxAngle ≤ 120 := by
  refine ⟨?_, ?_⟩ <;> native_decide

/-- Derived theorem witness: no backward bending. -/
def candidateNoBackwardBending : candidateFinger.NoBackwardBending :=
  wellFormed_noBackwardBending candidateFinger candidateWellFormed candidateBounds

def renderFingerJson (f : FingerParams) : String :=
  "{" ++
    s!"\"l1\":{f.l1}," ++
    s!"\"l2\":{f.l2}," ++
    s!"\"l3\":{f.l3}," ++
    s!"\"thickness\":{f.thickness}," ++
    s!"\"minAngle\":{f.minAngle}," ++
    s!"\"maxAngle\":{f.maxAngle}" ++
  "}"

def main : IO Unit := do
  -- Force all three proofs into the runtime image. If any fails to typecheck,
  -- `lake build` errors out and `run_poc2.sh` halts before any CAD is emitted.
  let _ : candidateFinger.WellFormed := candidateWellFormed
  let _ : candidateFinger.NoBackwardBending := candidateNoBackwardBending
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/finger_params.json" (renderFingerJson candidateFinger)
  IO.println "Lean proof passed: finger is well-formed and free of backward bending."
  IO.println "Wrote ../out/finger_params.json"
