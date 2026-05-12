import MechProof
open MechProof

/-- The candidate case we want to fabricate. -/
def candidate : DraftedCase :=
  { length := 60.0, width := 40.0, height := 25.0,
    thickness := 2.0, draftDeg := 2.0 }

/-- Compile-time proof obligation: the candidate is well-formed.
    `Float` comparisons are opaque to the kernel reducer but are decidable at
    runtime, so we discharge with `native_decide`. If any conjunct fails,
    `lake build` errors out — the proof IS the gate. -/
def candidateWellFormed : candidate.WellFormed := by
  refine ⟨?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_⟩ <;> native_decide

/-- Derived moldability proof. If the moldability theorem ever breaks,
    this definition fails to typecheck and `lake build` errors out. -/
def candidateMoldable : candidate.Moldable :=
  wellFormed_implies_moldable candidate candidateWellFormed

/-- Render the case as a one-line JSON object. -/
def renderJson (c : DraftedCase) : String :=
  "{" ++
    s!"\"length\":{c.length}," ++
    s!"\"width\":{c.width}," ++
    s!"\"height\":{c.height}," ++
    s!"\"thickness\":{c.thickness}," ++
    s!"\"draftDeg\":{c.draftDeg}" ++
  "}"

def main : IO Unit := do
  -- Force the proof into the runtime image. If `candidateMoldable` doesn't
  -- typecheck, this file won't compile and the orchestrator halts.
  let _ : candidate.Moldable := candidateMoldable
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/verified_params.json" (renderJson candidate)
  IO.println "Lean proof passed: candidate is well-formed and moldable."
  IO.println "Wrote ../out/verified_params.json"
