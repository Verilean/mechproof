import MechProof
open MechProof

/-- LIPM parameters. `comHeight` matches the steady-state torso Z observed
    in the PoC 8 simulator (≈ 0.76 m); `gravity` is shared with the arm
    proof; `zmpMargin` mirrors the static balance margin. -/
def candidateWalkParams : WalkingParams :=
  { comHeight := 0.756,
    gravity   := GRAVITY_M_PER_S2,
    zmpMargin := 0.02 }

/-- A quasi-static two-step gait. Accelerations are zero everywhere so the
    ZMP coincides with the CoM, which makes the inequality trivial.
    Real-world PoC would interpolate non-zero accelerations; here the
    point is to wire the theorem and the proof gate end-to-end.

    Step sequence:
      0. Double-support (both feet at y=0), CoM at origin.
      1. Shift CoM over left foot.
      2. Right foot lifted (still single-support on left).
      3. Right foot planted forward at y=0.10.
      4. Double-support, CoM shifted to y=0.05.
      5. Shift CoM over right foot (forward).
      6. Left foot lifted.
      7. Left foot planted forward at y=0.10.
      8. Double-support, CoM at y=0.10. -/
def candidateTrajectory : List TrajectoryStep := [
  -- 0. Double support, CoM at origin.
  { comX := 0.0,   comY := 0.0,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX :=  0.11,
    loY := -0.08,  hiY :=  0.08 },
  -- 1. Shift CoM over left foot.
  { comX := -0.07, comY := 0.0,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX :=  0.11,
    loY := -0.08,  hiY :=  0.08 },
  -- 2. Right foot lifted: support = left foot only.
  { comX := -0.07, comY := 0.0,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX := -0.03,
    loY := -0.08,  hiY :=  0.08 },
  -- 3. Right foot planted forward (y=+0.10), CoM still over left foot.
  --    Support polygon: convex hull of left foot (y∈[-0.08,0.08]) and right
  --    foot at y∈[0.02,0.18] = the union spans y∈[-0.08, 0.18].
  { comX := -0.07, comY := 0.0,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX :=  0.11,
    loY := -0.08,  hiY :=  0.18 },
  -- 4. Shift CoM to midpoint (y=0.05, x=0).
  { comX := 0.0,   comY := 0.05,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX :=  0.11,
    loY := -0.08,  hiY :=  0.18 },
  -- 5. Shift CoM over right (forward) foot.
  { comX := 0.07,  comY := 0.10,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX :=  0.11,
    loY := -0.08,  hiY :=  0.18 },
  -- 6. Left foot lifted: support = right foot only at (x=+0.07, y=+0.10).
  { comX := 0.07,  comY := 0.10,
    accX := 0.0,   accY := 0.0,
    loX :=  0.03,  hiX :=  0.11,
    loY :=  0.02,  hiY :=  0.18 },
  -- 7. Left foot planted forward (y=+0.10): convex hull spans x∈[-0.11,0.11],
  --    y∈[0.02, 0.18] now that both feet are at y=+0.10.
  { comX := 0.07,  comY := 0.10,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX :=  0.11,
    loY :=  0.02,  hiY :=  0.18 },
  -- 8. Final double-support, CoM at (0, 0.10).
  { comX := 0.0,   comY := 0.10,
    accX := 0.0,   accY := 0.0,
    loX := -0.11,  hiX :=  0.11,
    loY :=  0.02,  hiY :=  0.18 }
]

/-- **Compile-time ZMP stability proof.** Every keyframe of the candidate
    gait places the projected ZMP strictly inside the support polygon
    (with the 20 mm margin baked in). If any keyframe fails — e.g. the
    user wires up an over-aggressive acceleration that swings the ZMP
    outside the foot — `native_decide` reports `false` and the build
    aborts before any walking simulation runs. -/
def candidateGaitStable :
    allStable candidateWalkParams candidateTrajectory := by
  native_decide

def renderStep (i : Nat) (s : TrajectoryStep) (w : WalkingParams) : String :=
  "{\"index\":" ++ toString i ++
  ",\"comX\":" ++ toString s.comX ++
  ",\"comY\":" ++ toString s.comY ++
  ",\"accX\":" ++ toString s.accX ++
  ",\"accY\":" ++ toString s.accY ++
  ",\"loX\":" ++ toString s.loX ++ ",\"hiX\":" ++ toString s.hiX ++
  ",\"loY\":" ++ toString s.loY ++ ",\"hiY\":" ++ toString s.hiY ++
  ",\"zmpX\":" ++ toString (s.zmpX w) ++
  ",\"zmpY\":" ++ toString (s.zmpY w) ++ "}"

def renderSteps : List TrajectoryStep → WalkingParams → Nat → List String
  | [],     _, _ => []
  | s :: rest, w, i => renderStep i s w :: renderSteps rest w (i + 1)

def main : IO Unit := do
  let _ : allStable candidateWalkParams candidateTrajectory :=
    candidateGaitStable
  IO.FS.createDirAll "out"
  let stepJson := String.intercalate ","
    (renderSteps candidateTrajectory candidateWalkParams 0)
  let json :=
    "{\"comHeightM\":" ++ toString candidateWalkParams.comHeight ++
    ",\"gravityMPerS2\":" ++ toString candidateWalkParams.gravity ++
    ",\"zmpMarginM\":" ++ toString candidateWalkParams.zmpMargin ++
    ",\"nKeyframes\":" ++ toString candidateTrajectory.length ++
    ",\"keyframes\":[" ++ stepJson ++ "]}"
  IO.FS.writeFile "out/walking_proof.json" json
  IO.println s!"Lean ZMP proof passed: every keyframe ZMP-stable."
  IO.println s!"  comHeight = {candidateWalkParams.comHeight} m"
  IO.println s!"  zmpMargin = {candidateWalkParams.zmpMargin} m"
  IO.println s!"  keyframes = {candidateTrajectory.length}"
  IO.println "Wrote ../out/walking_proof.json"
