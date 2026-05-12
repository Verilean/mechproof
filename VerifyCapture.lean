import MechProof
open MechProof

/-- The same LIPM parameters as PoC 10 (comHeight = steady-state torso Z). -/
def walkParams : WalkingParams :=
  { comHeight := 0.756,
    gravity   := GRAVITY_M_PER_S2,
    zmpMargin := 0.02 }

/-- A 4-keyframe capture-point gait.

    Convention: the CoM is **already inside the next footstep's support
    polygon** when the step happens — this is the "captureable" regime
    where the swing leg arrives in time to catch the CoM. With √(H/g)
    ≈ 0.278 s, a CoM velocity of ~0.3 m/s shifts the capture point by
    ~0.08 m, so polygons of order 80 mm × 160 mm comfortably contain it. -/
def captureGait : List CaptureStep := [
  -- 1. End of left-foot single support: CoM at the right side of the body
  --    (x = +0.06) moving forward+right, capture point lands inside the
  --    right foot at (x≈+0.072, y≈+0.083).
  { comX :=  0.06, comY := 0.00,
    velX :=  0.05, velY := 0.30,
    loX :=  0.03,  hiX := 0.11,
    loY :=  0.02,  hiY := 0.18 },
  -- 2. Mid-stance on right foot: CoM further forward (y=+0.06), still
  --    inside the right foot polygon.
  { comX :=  0.07, comY := 0.06,
    velX :=  0.02, velY := 0.20,
    loX :=  0.03,  hiX := 0.11,
    loY :=  0.02,  hiY := 0.18 },
  -- 3. End of right-foot single support: CoM swung to the left of body
  --    midline (x=-0.06), moving forward+left toward the next left foot.
  --    Next polygon = left foot landing zone at y∈[0.12, 0.28], x∈[-0.11, -0.03].
  { comX := -0.06, comY := 0.10,
    velX := -0.05, velY := 0.30,
    loX := -0.11,  hiX := -0.03,
    loY :=  0.12,  hiY :=  0.28 },
  -- 4. Mid-stance on left foot: CoM further forward (y=+0.16), still
  --    inside the left foot polygon.
  { comX := -0.07, comY := 0.16,
    velX := -0.02, velY := 0.20,
    loX := -0.11,  hiX := -0.03,
    loY :=  0.12,  hiY :=  0.28 }
]

/-- **Compile-time Capture-Point stability proof.** Every keyframe of
    the candidate gait places the capture point strictly inside the
    next footstep's support polygon (with the configured margin). -/
def captureStable :
    allCapturable walkParams captureGait := by
  native_decide

def renderCaptureStep
    (i : Nat) (s : CaptureStep) (w : WalkingParams) : String :=
  "{\"index\":" ++ toString i ++
  ",\"comX\":" ++ toString s.comX ++
  ",\"comY\":" ++ toString s.comY ++
  ",\"velX\":" ++ toString s.velX ++
  ",\"velY\":" ++ toString s.velY ++
  ",\"loX\":" ++ toString s.loX ++ ",\"hiX\":" ++ toString s.hiX ++
  ",\"loY\":" ++ toString s.loY ++ ",\"hiY\":" ++ toString s.hiY ++
  ",\"cpX\":" ++ toString (s.cpX w) ++
  ",\"cpY\":" ++ toString (s.cpY w) ++
  ",\"orbitalEnergy\":" ++ toString (s.orbitalEnergy w) ++ "}"

def renderCaptureList :
    List CaptureStep → WalkingParams → Nat → List String
  | [],     _, _ => []
  | s :: rest, w, i => renderCaptureStep i s w :: renderCaptureList rest w (i + 1)

def main : IO Unit := do
  let _ : allCapturable walkParams captureGait := captureStable
  IO.FS.createDirAll "out"
  let stepJson := String.intercalate ","
    (renderCaptureList captureGait walkParams 0)
  let json :=
    "{\"comHeightM\":" ++ toString walkParams.comHeight ++
    ",\"gravityMPerS2\":" ++ toString walkParams.gravity ++
    ",\"zmpMarginM\":" ++ toString walkParams.zmpMargin ++
    ",\"sqrtHOverG\":" ++ toString (floatSqrt (walkParams.comHeight / walkParams.gravity)) ++
    ",\"nKeyframes\":" ++ toString captureGait.length ++
    ",\"keyframes\":[" ++ stepJson ++ "]}"
  IO.FS.writeFile "out/capture_proof.json" json
  IO.println s!"Lean Capture-Point proof passed: every keyframe stays inside the next footstep's capture region."
  IO.println s!"  √(H/g)    = {floatSqrt (walkParams.comHeight / walkParams.gravity)} s"
  IO.println s!"  margin    = {walkParams.zmpMargin} m"
  IO.println s!"  keyframes = {captureGait.length}"
  IO.println "Wrote ../out/capture_proof.json"
